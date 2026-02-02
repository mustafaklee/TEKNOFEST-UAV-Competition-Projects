import cv2
from picamera2 import Picamera2
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading
from pymavlink import mavutil
import logging
import math
from enum import Enum, auto
import numpy as np
from detect_tracking5 import detect_target_and_get_output
import kutu_acma
import pence
class MissionParameters:
    def __init__(self):
        self.patrol_altitude = 5.0
        self.hover_altitude = 1.0
        self.patrol_speed = 3.0
        self.cargo_delivery_altitude = 1.0
        self.square_hover_altitude = 0.6
        
        self.target_lost_timeout = 5.0
        self.hover_duration = 10.0
        self.move_interval = 0.2
        self.min_detection_frames = 3
        self.stable_detection_threshold = 5
        self.saved_hexagon_location = None
        self.require_dynamic_waypoint = False
        self.saved_triangle_location = None
        
        self.pid_params = {
            'x': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000},
            'y': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000}
        }
        
        self.image_center = (320, 240)
        self.dynamic_tolerance_enabled = True
        self.tolerance_levels = [55, 35, 25, 20]
        self.current_tolerance_index = 0
        self.target_tolerance = self.tolerance_levels[0]
        self.strict_tolerance_levels = [80, 60, 40, 35]
        
        self.altitude_thresholds = [4.0, 2.5, 1.5, 1.0]
        self.last_tolerance_change_alt = None
        self.descent_steps = [0.8, 0.5, 0.3, 0.2]

        self.require_hexagon = True
        self.require_triangle = True
        self.hexagon_completed = False
        self.triangle_completed = False
        self.square_completed = False
        self.cargo_loaded = False
        self.square_search_timeout = 30.0

    def update_tolerance_by_altitude(self, current_altitude):
        if not self.dynamic_tolerance_enabled:
            return
            
        new_index = 0
        for i, threshold in enumerate(self.altitude_thresholds):
            if current_altitude <= threshold:
                new_index = min(i + 1, len(self.tolerance_levels) - 1)
                break
        
        if new_index != self.current_tolerance_index:
            old_tolerance = self.target_tolerance
            self.current_tolerance_index = new_index
            self.target_tolerance = self.tolerance_levels[new_index]
            self.last_tolerance_change_alt = current_altitude
            return old_tolerance, self.target_tolerance
        
        return None, self.target_tolerance

    def reset_tolerance(self):
        self.current_tolerance_index = 0
        self.target_tolerance = self.tolerance_levels[0]
        self.last_tolerance_change_alt = None

class PID:
    def __init__(self, kp, ki, kd, imax=1000):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax = imax
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = None

    def update(self, err):
        now = time.time()
        if self.prev_t is None:
            self.prev_t = now
            self.prev_err = err
            return 0.0
        
        dt = now - self.prev_t
        de = err - self.prev_err
        self.integral += err * dt
        self.integral = max(min(self.integral, self.imax), -self.imax)

        out = (self.kp * err) + (self.ki * self.integral) + (self.kd * de / dt)
        self.prev_err, self.prev_t = err, now
        return out

class DroneState(Enum):
    TAKEOFF = auto()
    PATROL = auto()
    TARGET_TRACK = auto()
    DESCENDING = auto()
    HOVERING = auto()
    RETURNING = auto()
    SEARCHING_HEXAGON = auto()
    SEARCHING_TRIANGLE = auto()
    SEARCHING_SQUARE = auto()
    
class PatrolDrone:
    def __init__(self, patrol_coordinates):
        self.params = MissionParameters()
        print("camera başlatılıyor...")
        self.camera = Picamera2()
        self.camera.preview_configuration.main.size = (640, 480)
        self.camera.preview_configuration.main.format = "RGB888"
        self.camera.configure("preview")
        try:
            self.camera.start()
            print("✅ Kamera başlatıldı")
        except Exception as e:
            print(f"❌ Kamera açılamadı: {e}")
            return
        print("Kamera başlatıldı, DroneKit bağlantısı kuruluyor...")
        self.vehicle = connect('/dev/ttyS0/', wait_ready=True)
        print("DroneKit bağlantısı kuruldu.")
        self.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        
        self.patrol_coordinates = patrol_coordinates
        self.current_waypoint_index = 0

        self.tolerance_update_interval = 0.5
        self.last_tolerance_check = 0
        
        self.state = DroneState.TAKEOFF
        self.current_target_type = None
        
        self.target_history = []
        self.max_history = 8
        self.target_lost_time = 0
        self.last_target_time = 0
        self.last_move_time = 0
        self.hover_start_time = 0
        
        self.detection_counter = 0
        self.stable_target_detected = False
        self.last_stable_detection_time = 0
        
        self.pid_x = PID(**self.params.pid_params['x'])
        self.pid_y = PID(**self.params.pid_params['y'])
        
        self.square_search_start_time = None
        
        self.control_thread = threading.Thread(target=self.main_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.search_for = None
        self.hiz_limit = 0.3
        self.patrol_coordinates = patrol_coordinates
        self.original_patrol_coordinates = patrol_coordinates.copy()
        self.current_waypoint_index = 0

        self.image_thread = threading.Thread(target=self.image_loop)
        self.image_thread.daemon = True
        self.image_thread.start()

    def image_loop(self):
        while True:
            try:
                frame = self.camera.capture_array()
                cv_image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.image_callback(cv_image)
                time.sleep(0.033)
            except Exception as e:
                print(f"Kamera okuma hatası: {e}")
                time.sleep(1)

    def get_dynamic_descent_step(self, current_alt):
        if current_alt > 4.0:
            return self.params.descent_steps[0]
        elif current_alt > 2.5:
            return self.params.descent_steps[1]
        elif current_alt > 1.5:
            return self.params.descent_steps[2]
        elif current_alt > 1.0:
            return self.params.descent_steps[3]
        else:
            return 0.1

    def get_strict_tolerance_by_altitude(self, current_alt):
        for i, threshold in enumerate(self.params.altitude_thresholds):
            if current_alt <= threshold:
                return self.params.strict_tolerance_levels[min(i, len(self.params.strict_tolerance_levels) - 1)]
        return self.params.strict_tolerance_levels[0]

    def update_dynamic_tolerance(self):
        current_time = time.time()
        if current_time - self.last_tolerance_check < self.tolerance_update_interval:
            return
            
        current_alt = self.vehicle.location.global_relative_frame.alt
        old_tol, new_tol = self.params.update_tolerance_by_altitude(current_alt)
        
        if old_tol is not None:
            print(f"🎯 Tolerans güncellendi: {old_tol}px → {new_tol}px (İrtifa: {current_alt:.1f}m)")
        
        self.last_tolerance_check = current_time 

    def reset_detection_state(self):
        self.detection_counter = 0
        self.stable_target_detected = False
        self.current_target_type = None
        self.target_lost_time = 0

    def main_control_loop(self):
        try:
            self.arm_and_takeoff(self.params.patrol_altitude)
            self.state = DroneState.PATROL
            
            while True:
                if self.state is None:
                    time.sleep(1)
                    continue
                
                current_state = self.state
                
                if current_state == DroneState.TAKEOFF:
                    pass
                
                elif current_state == DroneState.PATROL:
                    self.patrol_behavior()
                
                elif current_state == DroneState.TARGET_TRACK:
                    self.target_tracking_behavior()
                
                elif current_state == DroneState.DESCENDING:
                    self.descending_behavior()
                
                elif current_state == DroneState.HOVERING:
                    self.hovering_behavior()
                
                elif current_state == DroneState.RETURNING:
                    self.returning_behavior()
                
                elif current_state == DroneState.SEARCHING_HEXAGON:
                    self.search_behavior(target_type='hexagon')
                
                elif current_state == DroneState.SEARCHING_TRIANGLE:
                    self.search_behavior(target_type='triangle')

                elif current_state == DroneState.SEARCHING_SQUARE:
                    self.square_search_behavior()

                time.sleep(0.1)
                
        except Exception as e:
            print(f"Kontrol döngüsünde hata: {e}")
            self.emergency_land()

    def patrol_behavior(self):
        current_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        current_location = self.vehicle.location.global_relative_frame
        
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon, 
            current_waypoint[0], current_waypoint[1]
        )
        
        if distance < 8.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
            next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
            print(f"📍 WP{self.current_waypoint_index+1}: {next_waypoint}")
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        if self.stable_target_detected:
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

    def target_tracking_behavior(self):
        current_time = time.time()
        
        self.update_dynamic_tolerance()
        
        if not self.stable_target_detected: 
            if self.target_lost_time == 0:
                self.target_lost_time = current_time
                print("⚠️ Hedef kaybedildi...")
            
            elif current_time - self.target_lost_time > self.params.target_lost_timeout:
                print("🔄 Hedef kayıp - Devriyeye dönülüyor...")
                self.state = DroneState.RETURNING
                self.target_lost_time = 0
                self.detection_counter = 0
                self.target_history.clear()
                self.params.reset_tolerance()
        else:
            self.target_lost_time = 0
            self.last_target_time = current_time

    def descending_behavior(self):
        current_alt = self.vehicle.location.global_relative_frame.alt
        current_time = time.time()
        
        self.update_dynamic_tolerance()

        if not self.stable_target_detected:
            print("❌ İniş sırasında hedef kayıp - takibe geri dönülüyor")
            self.state = DroneState.TARGET_TRACK
            self.target_lost_time = current_time
            return

        if len(self.target_history) < 3:
            print("⏳ Hedef ortalama hesaplanıyor... bekleniyor")
            time.sleep(0.2)
            return

        avg_dx = sum([h[0] for h in self.target_history[-3:]]) / 3
        avg_dy = sum([h[1] for h in self.target_history[-3:]]) / 3
        current_dx, current_dy = self.target_history[-1]
        
        current_tolerance = self.params.target_tolerance
        avg_centered = abs(avg_dx) < current_tolerance and abs(avg_dy) < current_tolerance
        strict_tol = self.get_strict_tolerance_by_altitude(current_alt)
        current_centered = abs(current_dx) < strict_tol and abs(current_dy) < strict_tol
                
        print(f"📊 Tolerans: {current_tolerance}px | Ortalama: dx={avg_dx:.1f}, dy={avg_dy:.1f} | Anlık: dx={current_dx:.1f}, dy={current_dy:.1f}")
        
        if avg_centered and current_centered:
            target_hover_alt = self.params.square_hover_altitude if self.current_target_type == 'square' else self.params.hover_altitude
            if current_alt > target_hover_alt + 0.3:
                descent_step = self.get_dynamic_descent_step(current_alt)
                descent_step = min(descent_step, current_alt - target_hover_alt - 0.2)
                new_alt = current_alt - descent_step
                
                print(f"⬇️ Hedef ortalı (Tol:{current_tolerance}px) - İrtifa: {current_alt:.1f}m → {new_alt:.1f}m")
                
                location = LocationGlobalRelative(
                    self.vehicle.location.global_relative_frame.lat,
                    self.vehicle.location.global_relative_frame.lon,
                    new_alt
                )
                self.vehicle.simple_goto(location, groundspeed=self.hiz_limit)
                time.sleep(1.5)
            else:
                target_hover_alt = self.params.square_hover_altitude if self.current_target_type == 'square' else self.params.hover_altitude
                print(f"🚁 Hedef irtifaya ulaşıldı - Bekleme moduna geçiliyor! ({target_hover_alt}m)")
                self.state = DroneState.HOVERING
                self.hover_start_time = time.time()
                
                location = LocationGlobalRelative(
                    self.vehicle.location.global_relative_frame.lat,
                    self.vehicle.location.global_relative_frame.lon,
                    target_hover_alt
                )
                self.vehicle.simple_goto(location, groundspeed=self.hiz_limit)
        else:
            print(f"🎯 Hedef sapması yüksek (Tol:{current_tolerance}px) - Takibe geri dönülüyor")
            self.state = DroneState.TARGET_TRACK
            
            current_location = self.vehicle.location.global_relative_frame
            hold_location = LocationGlobalRelative(
                current_location.lat,
                current_location.lon,
                current_location.alt
            )
            self.vehicle.simple_goto(hold_location, groundspeed=self.hiz_limit)

    def square_search_behavior(self):
        current_time = time.time()
        
        if self.stable_target_detected and self.current_target_type == 'square':
            print("🔴 ZOOM'da kırmızı kare tespit edildi, takip başlıyor!")
            self.square_search_start_time = None
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()
            return
        
        if self.square_search_start_time is None:
            self.square_search_start_time = current_time
            print("🔍 Kırmızı kare arama başladı - 30 saniye timeout...")
        
        elapsed_time = current_time - self.square_search_start_time
        if elapsed_time > self.params.square_search_timeout:
            print("⏰ TIMEOUT! 30 saniye içinde kırmızı kare tespit edilemedi - RTL moduna geçiliyor...")
            self.vehicle.mode = VehicleMode("RTL")
            self.state = None
            return
        
        remaining_time = self.params.square_search_timeout - elapsed_time
        if int(remaining_time) % 5 == 0 and remaining_time != self.params.square_search_timeout:
            print(f"🔍 Kırmızı kare aranıyor... Kalan süre: {remaining_time:.1f}s")
        
        current_location = self.vehicle.location.global_relative_frame
        
        hold_location = LocationGlobalRelative(
            current_location.lat,
            current_location.lon,
            self.params.cargo_delivery_altitude
        )
        self.vehicle.simple_goto(hold_location, groundspeed=0.2)
        
        self.search_for = "square"

    def hovering_behavior(self):
        current_time = time.time()
        elapsed_time = current_time - self.hover_start_time
        remaining_time = self.params.hover_duration - elapsed_time
        
        target_hover_alt = self.params.square_hover_altitude if self.current_target_type == 'square' else self.params.hover_altitude
        current_location = self.vehicle.location.global_relative_frame
        altitude_diff = abs(current_location.alt - target_hover_alt)
        
        if altitude_diff > 0.3:
            print(f"🔧 İrtifa düzeltmesi: {current_location.alt:.1f}m → {target_hover_alt:.1f}m")
            location = LocationGlobalRelative(
                current_location.lat,
                current_location.lon,
                target_hover_alt
            )
            self.vehicle.simple_goto(location, groundspeed=0.2)
            time.sleep(0.5)
        else:
            self.send_body_velocity(0, 0, 0, 0)
        
        if remaining_time > 0:
            print(f"⏳ {self.current_target_type} üzerinde bekleniyor... {remaining_time:.1f}s kaldı")
        else:
            if self.current_target_type == 'hexagon':
                self.params.hexagon_completed = True
                print("✅ Altıgen görevi tamamlandı!")
                
                #pence acma
                kutu_acma.box_open(self.vehicle,channel=10,degree=200, wait=2.0)
                kutu_acma.box_close(self.vehicle,channel=10,degree=0, wait=2.0)
                time.sleep(3)

                current_location = self.vehicle.location.global_relative_frame
                self.params.saved_hexagon_location = (current_location.lat, current_location.lon)
                
                self.state = DroneState.RETURNING

            elif self.current_target_type == 'triangle':
                self.params.triangle_completed = True

                print("✅ Üçgen görevi tamamlandı!")

                #kutu acma
                kutu_acma.box_open(self.vehicle,channel=9, wait=2.0,degree=180)
                kutu_acma.box_close(self.vehicle,channel=9, wait=2.0,degree=0)

                current_location = self.vehicle.location.global_relative_frame
                self.params.saved_triangle_location = (current_location.lat, current_location.lon)
                self.search_for = None

                if self.params.saved_hexagon_location:
                    self.params.require_dynamic_waypoint = True
                
                self.state = DroneState.RETURNING
            
            elif self.current_target_type == 'square':
                self.params.square_completed = True
                self.params.cargo_loaded = True

                
                pence.rotate_sequence(self.vehicle,11)

                print("✅ Kırmızı kare görevi tamamlandı!")
                print("📦 YÜK ALINDI!")
                
                self.search_for = None
                self.state = DroneState.RETURNING
                return

    def returning_behavior(self):
        self.square_search_start_time = None
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        self.reset_detection_state()
        self.params.reset_tolerance()
        
        if current_alt < self.params.patrol_altitude - 1.0:
            print(f"⬆️ Yükseliyor... {current_alt:.1f}m → {self.params.patrol_altitude}m")
            location = LocationGlobalRelative(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.params.patrol_altitude
            )
            self.vehicle.simple_goto(location, groundspeed=self.hiz_limit)
            time.sleep(1.0)
            return
        
        if self.params.hexagon_completed and not self.params.triangle_completed and not self.params.require_dynamic_waypoint:
            print("🔍 Üçgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_TRIANGLE
            self.find_nearest_waypoint_and_go()
            self.target_history.clear()
            self.target_lost_time = 0
            return
        
        if self.params.require_dynamic_waypoint:
            if not self.params.square_completed:
                print("🔄 Üçgen tamamlandı, kaydedilen altıgen konumuna gidiliyor...")
                lat1, lon1 = self.params.saved_hexagon_location
                self.goto_waypoint(lat1, lon1, self.params.cargo_delivery_altitude)
                self.wait_until_arrival(lat1, lon1)
                
                self.search_for = "square"
                self.reset_detection_state()
                
                print("🔍🔍 Altıgen konumunda ZOOM ile kırmızı kare aranıyor...")
                self.state = DroneState.SEARCHING_SQUARE
                return
            
            else:
                print("🔄 Yük ile üçgen konumuna gidiliyor...")
                lat2, lon2 = self.params.saved_triangle_location
                self.goto_waypoint(lat2, lon2, self.params.cargo_delivery_altitude)
                self.wait_until_arrival(lat2, lon2)
                
                print("📦 Yük bırakılıyor, bekleniyor...")
                time.sleep(5)
                
                #kırmızı kareyi bırak
                kutu_acma.box_open(self.vehicle,channel=10,degree=200, wait=2.0)
                kutu_acma.box_close(self.vehicle,channel=10,degree=0, wait=2.0)

                print("🛬 Görev tamamlandı, RTL moduna geçiliyor...")
                self.vehicle.mode = VehicleMode("RTL")
                self.state = None
                return
        
        elif not self.params.hexagon_completed:
            print("🔍 Altıgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_HEXAGON
            self.find_nearest_waypoint_and_go()
        else:
            self.state = DroneState.PATROL
            self.find_nearest_waypoint_and_go()

        self.target_history.clear()
        self.target_lost_time = 0

    def wait_until_arrival(self, target_lat, target_lon, tolerance=2.0, timeout=20):
        start_time = time.time()
        while time.time() - start_time < timeout:
            current = self.vehicle.location.global_relative_frame
            distance = self.get_distance_metres(current.lat, current.lon, target_lat, target_lon)
            if distance < tolerance:
                print(f"✅ Hedefe ulaşıldı: {distance:.1f}m")
                return True
            time.sleep(1)
        print(f"⚠️ Hedefe varılamadı: {distance:.1f}m")
        return False

    def search_behavior(self, target_type):
        original_patrol = self.patrol_coordinates[:4]
        
        current_idx = self.current_waypoint_index % len(original_patrol)
        current_waypoint = original_patrol[current_idx]
        
        current_location = self.vehicle.location.global_relative_frame
        
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon,
            current_waypoint[0], current_waypoint[1]
        )
        
        if distance < 8.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(original_patrol)
            next_idx = self.current_waypoint_index % len(original_patrol)
            next_waypoint = original_patrol[next_idx]
            print(f"🔍 {target_type} aranıyor - WP{next_idx+1}: {next_waypoint}")
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        if (self.stable_target_detected and 
            self.current_target_type == target_type and 
            self.search_for == target_type):
            print(f"🎯 {target_type} kararlı şekilde tespit edildi!")
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

    def image_callback(self, cv_image):
        if self.state == DroneState.SEARCHING_SQUARE:
            self.search_for = "square"
        elif self.state == DroneState.SEARCHING_HEXAGON:
            self.search_for = "hexagon"
        elif self.state == DroneState.SEARCHING_TRIANGLE:
            self.search_for = "triangle"
        elif self.state == DroneState.PATROL:
            if not self.params.hexagon_completed:
                self.search_for = "hexagon"
            elif not self.params.triangle_completed:
                self.search_for = "triangle"
            else:
                self.search_for = "disabled"
        elif self.state == DroneState.RETURNING:
            self.search_for = "disabled"
        elif self.state in [DroneState.TARGET_TRACK, DroneState.DESCENDING, DroneState.HOVERING]:
            pass
        else:
            self.search_for = "disabled"

        vis, t_type, center = detect_target_and_get_output(cv_image, model=None, search_for=self.search_for)

        if t_type in ['hexagon', 'triangle', 'square']:
            self.detection_counter += 1
            self.current_target_type = t_type
            
            if self.detection_counter >= self.params.min_detection_frames:
                self.stable_target_detected = True
                self.last_stable_detection_time = time.time()
        else:
            self.detection_counter = max(0, self.detection_counter - 2)
            if self.detection_counter == 0:
                self.stable_target_detected = False
                self.current_target_type = None

        if self.stable_target_detected and center:
            err_x = center[0] - self.params.image_center[0]
            err_y = center[1] - self.params.image_center[1]
            
            self.target_history.append((err_x, err_y))
            if len(self.target_history) > self.max_history:
                self.target_history.pop(0)
            
            if self.state == DroneState.TARGET_TRACK:
                current_time = time.time()
                if current_time - self.last_move_time > self.params.move_interval:
                    current_tolerance = self.params.target_tolerance
                    if abs(err_x) < current_tolerance and abs(err_y) < current_tolerance:
                        current_alt = self.vehicle.location.global_relative_frame.alt
                        if current_alt > self.params.hover_altitude + 1.0:
                            self.state = DroneState.DESCENDING
                        else:
                            self.state = DroneState.HOVERING
                            self.hover_start_time = time.time()
                    else:
                        vy_body = self.pid_y.update(err_y)
                        vx_body = self.pid_x.update(err_x)

                        vy = max(min(vy_body, self.hiz_limit), -self.hiz_limit)
                        vx = max(min(vx_body, self.hiz_limit), -self.hiz_limit)

                        if self.current_target_type == 'square':
                            vy = vy / 5.0
                            vx = vx / 5.0
                            print(f"KARE HASSAS HAREKET: vx={vx:.2f}, vy={vy:.2f}")

                        self.send_body_velocity(vx, vy, 0, 0)
                        print(f"{self.current_target_type} Tol:{current_tolerance}px vx={vx:.2f}, vy={vy:.2f}")
                    self.last_move_time = current_time

    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def arm_and_takeoff(self, target_altitude):
        print("drone  Başlatılıyor...")
        while not self.vehicle.is_armable:
            time.sleep(1)
            
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        print("mode guided olarak ayarlandı, drone arm ediliyor...")
        while not self.vehicle.armed:
            time.sleep(1)
        print("Drone arm edildi, kalkışa hazırlanılıyor...")
        print("🛫 Kalkış...")
        self.vehicle.simple_takeoff(target_altitude)
        
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                print(f"✅ Yükseklik: {alt:.1f}m")
                break
            time.sleep(1)
        
        first_waypoint = self.patrol_coordinates[0]
        self.goto_waypoint(first_waypoint[0], first_waypoint[1])

    def goto_waypoint(self, lat, lon, altitude=None):
        if altitude is None:
            altitude = self.params.patrol_altitude
        location = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.simple_goto(location, groundspeed=self.params.patrol_speed)
        
    def find_nearest_waypoint_and_go(self):
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
        
        next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        print(f"📍 Sıradaki waypoint: WP{self.current_waypoint_index+1}: {next_waypoint}")
    
    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)

    def emergency_land(self):
        print("🆘 Acil iniş yapılıyor!")
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        self.vehicle.close()

def main():
    patrol_coordinates = [
        (2, 1),
    ]
    
    patrol_drone = PatrolDrone(patrol_coordinates)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("🛑 Sistem kapatılıyor...")
        patrol_drone.emergency_land()
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()