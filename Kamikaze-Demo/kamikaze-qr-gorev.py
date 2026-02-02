#!/usr/bin/env python3
from pymavlink import mavutil
import time
import threading
import math
import sys

# ROS2 ve OpenCV için gerekli kütüphaneler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar  # QR kod okuma için
import numpy as np

# Global değişkenler
current_agl = 0.0
current_airspeed = 0.0
agl_updated = 0
running = True

# QR kod tarama durumu
qr_scan_active = False
qr_scan_lock = threading.Lock()

# ROS2 Kamera Abonesi ve QR Kod Okuyucu
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/world/talon_runway/model/mini_talon_vtail/link/base_link/sensor/talon_camera/image',
            self.image_callback,
            10
        )
        self.msg_count = 0
        self.timer = self.create_timer(2.0, self.check_status)
        self.qr_detected = False
        self.qr_data = ""
        self.qr_history = []  # QR geçmişi
        print("📷 Kamera abonesi başlatıldı")
    
    def image_callback(self, msg):
        try:
            self.msg_count += 1
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # QR tarama aktifse tarama yap
            global qr_scan_active
            if qr_scan_active:
                cv_image = self.scan_qr_code(cv_image)
            
            # Görüntüyü göster
            cv2.imshow("Mini Talon Camera - QR Scanner", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')
    
# Değiştirilecek kısım: scan_qr_code fonksiyonu
    def scan_qr_code(self, image):
        """Görüntüde QR kodu tarar ve görselleştirir"""
        try:
            # Görüntü ön işleme - QR okunabilirliğini artır
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Histogram eşitleme (düşük ışık için)
            gray = cv2.equalizeHist(gray)
            # Kenar keskinleştirme
            kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            sharpened = cv2.filter2D(gray, -1, kernel)
            
            # QR kodları tara (hem renkli hem gri skala)
            decoded_objects = pyzbar.decode(image) + pyzbar.decode(sharpened)
            
            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                
                # QR boyutu kontrolü (çok küçükse yoksay)
                x, y, w, h = obj.rect
                if w < 50 or h < 50:  # 50px'den küçük QR'lar
                    self.get_logger().info(f"Ignored small QR: {w}x{h}px")
                    continue
                    
                # Yeni QR kodu ise kaydet
                if qr_data not in self.qr_history:
                    self.qr_history.append(qr_data)
                    self.qr_data = qr_data
                    self.qr_detected = True
                    print(f"✅ YENİ QR Kodu Okundu: {qr_data}")
                    print(f"📋 Toplam QR: {len(self.qr_history)}")
                    # ROS loguna da yaz
                    self.get_logger().info(f"Yeni QR: {qr_data} ({w}x{h}px)")
                
                # QR kodun etrafına çerçeve çiz
                points = obj.polygon
                if len(points) > 4:
                    hull = cv2.convexHull(
                        np.array([point for point in points], dtype=np.float32)
                    )
                    hull = list(map(tuple, np.squeeze(hull)))
                else:
                    hull = points
                
                n = len(hull)
                for j in range(0, n):
                    cv2.line(image, hull[j], hull[(j+1) % n], (0, 255, 0), 3)
                
                # Bilgi metni
                cv2.putText(image, qr_data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0), 2)
                cv2.putText(image, f"{w}x{h}px", (x, y+h+20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
                
            # QR kodu sayısını göster
            cv2.putText(image, f"QR KOD: {len(self.qr_history)}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # QR tarama aktif olduğunu göster
            status_text = "QR TARAMA AKTIF" if qr_scan_active else "QR TARAMA PASIF"
            color = (0, 255, 0) if qr_scan_active else (0, 0, 255)
            cv2.putText(image, status_text, (10, image.shape[0]-20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            return image
            
        except Exception as e:
            self.get_logger().error(f'QR tarama hatası: {str(e)}')
            return image
    def check_status(self):
        if self.msg_count == 0:
            self.get_logger().warn('Henüz kamera mesajı alınmadı')
        else:
            self.get_logger().info(f'Kamera mesajları alınıyor: {self.msg_count} mesaj')
            if len(self.qr_history) > 0:
                self.get_logger().info(f'QR Geçmişi: {self.qr_history}')

# ROS2'yi başlatan fonksiyon
def start_ros2_camera():
    """ROS2 kamera thread'ini başlatır"""
    def ros2_thread():
        try:
            print("🚀 ROS2 kamera thread'i başlatılıyor...")
            rclpy.init()
            camera_subscriber = CameraSubscriber()
            print("📷 Kamera subscriber oluşturuldu")
            rclpy.spin(camera_subscriber)
        except Exception as e:
            print(f"❌ ROS2 kamera hatası: {e}")
        finally:
            try:
                camera_subscriber.destroy_node()
                rclpy.shutdown()
                cv2.destroyAllWindows()
                print("🔄 ROS2 kamera kapatıldı")
            except:
                pass
    
    ros2_camera_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros2_camera_thread.start()
    print("✅ ROS2 kamera thread'i başlatıldı")
    return ros2_camera_thread

# Bağlantıyı kur
master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()
print("✅ MAVLink bağlantısı kuruldu.")

# ROS2 kamera sistemini başlat
print("🎥 ROS2 kamera sistemi başlatılıyor...")
camera_subscriber_instance = None
try:
    camera_thread = start_ros2_camera()
    time.sleep(3)  # ROS2'nin başlaması için bekle
    print("📹 Kamera sistemi hazır")
except Exception as e:
    print(f"⚠️ ROS2 kamera başlatılamadı: {e}")
    print("📝 Kamera olmadan devam ediliyor...")

# Yükseklik ve hız okuma thread'i
def telemetry_reader():
    global current_agl, current_airspeed, agl_updated, running
    while running:
        try:
            # GLOBAL_POSITION_INT mesajını al
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if msg:
                current_agl = msg.relative_alt / 1000.0  # mm to meters
                agl_updated = time.time()
            
            # VFR_HUD mesajını al (airspeed için)
            msg2 = master.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
            if msg2:
                current_airspeed = msg2.airspeed
                
        except Exception as e:
            print(f"⚠️ Telemetri okuma hatası: {e}")
        time.sleep(0.01)

# Telemetri thread'ini başlat
telemetry_thread = threading.Thread(target=telemetry_reader)
telemetry_thread.daemon = True
telemetry_thread.start()
print("📊 Telemetri thread'i başlatıldı")

# Mod değiştir: FBWA (manuel yerine)
mode = 'FBWA'  # FBWA otomatik dengeleme sağlar
master.set_mode_apm(mode)
print(f"🟢 {mode} moduna geçildi.")
time.sleep(2)

# ARM
print("🟡 ARM ediliyor...")
master.arducopter_arm()
master.motors_armed_wait()
print("✅ Uçak ARM oldu.")

# QR taramayı aktifleştir (baştan itibaren)
with qr_scan_lock:
    qr_scan_active = True
    print("🔍 QR tarama aktif edildi (uçuş boyunca)")

# Kademeli throttle artışı (tüm kontroller nötr)
print("🚀 Kademeli throttle artışı...")
for throttle in range(1100, 1801, 100):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, throttle, 1500, 0, 0, 0, 0  # RC1=1500 (aileron), RC4=1500 (rudder)
    )
    print(f"⚡ Throttle: {throttle}")
    time.sleep(1)

# Yeterli hız kazanana kadar bekle
print("🛫 Yeterli hız kazanılması bekleniyor...")
while current_airspeed < 15.0:  # 15 m/s minimum hız
    print(f"🌪️ Hız: {current_airspeed:.2f} m/s, AGL: {current_agl:.2f} m")
    time.sleep(0.5)

print("✅ Yeterli hız kazanıldı, tırmanışa geçiliyor...")

# Kademeli tırmanış
print("🛫 Kademeli tırmanış başlatılıyor...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1650, 1800, 1500, 0, 0, 0, 0  # Tüm kontroller düz
)
print("📈 Tırmanış: RC1=1500, RC2=1650, RC3=1800, RC4=1500")

# 20 metreye kadar tırmanış bekle
print("🔼 20 metreye kadar tırmanış...")
last_altitude = 0
stall_counter = 0

while True:
    print(f"📏 AGL: {current_agl:.2f} m, Hız: {current_airspeed:.2f} m/s")
    
    # Stall kontrolü
    if current_airspeed < 12.0:  # Stall riski
        stall_counter += 1
        if stall_counter > 10:  # 1 saniye stall
            print("⚠️ STALL RİSKİ! Pitch azaltılıyor...")
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                1500, 1550, 1800, 1500, 0, 0, 0, 0  # Pitch azalt
            )
            time.sleep(2)
            stall_counter = 0
    else:
        stall_counter = 0
    
    # Yükseklik kaybı kontrolü
    if current_agl < last_altitude - 1.0:  # 1 metreden fazla kayıp
        print("⚠️ Yükseklik kaybı! Throttle artırılıyor...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1600, 1900, 1500, 0, 0, 0, 0  # Daha fazla throttle
        )
    
    if current_agl >= 20:
        print("🎯 20 metreye ulaşıldı!")
        break
    
    last_altitude = current_agl
    time.sleep(0.1)

# Hız kazanma
print("🏁 Düz uçuş ile hız kazanma...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1500, 1800, 1500, 0, 0, 0, 0  # Nötr pitch
)
time.sleep(3)

# Kontrollü dalış
print("🛬 Kontrollü dalış başlatılıyor...")
print("🔍 QR tarama dalış sırasında devam ediyor...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1200, 1600, 1500, 0, 0, 0, 0  # Daha yumuşak dalış (1200), throttle azalt
)
print("📉 Kontrollü dalış: RC1=1500, RC2=1200, RC3=1600, RC4=1500")

# 5 metreye kadar dalış bekle
print("🔽 5 metreye kadar dalış...")
while True:
    print(f"📉 AGL: {current_agl:.2f} m, Hız: {current_airspeed:.2f} m/s")
    
    # Çok hızlı dalış kontrolü
    if current_airspeed > 35.0:  # Maksimum hız sınırı
        print("⚠️ Çok hızlı! Throttle artırılıyor...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1300, 1700, 1500, 0, 0, 0, 0
        )
    
    if current_agl <= 5.0:
        print("🟢 5 metreye ulaşıldı!")
        break
    time.sleep(0.1)

# Toparlanma
print("📈 Toparlanma...")
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500, 1700, 1800, 1500, 0, 0, 0, 0
)
print("🔄 Toparlanma: RC1=1500, RC2=1700, RC3=1800, RC4=1500")

# 10 saniye toparlanma
print("⏰ 10 saniye toparlanma süresi...")
time.sleep(10)

# RC override temizle
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("🔁 RC override temizlendi. Kumanda kontrolüne dönüldü.")

# QR taramayı deaktif et
with qr_scan_lock:
    qr_scan_active = False
    print("🔍 QR tarama deaktif edildi")

print("📊 QR Tarama Özeti:")
print("=" * 30)

# Thread'leri durdur
print("🛑 Thread'ler durduruluyor...")
running = False
telemetry_thread.join()

# OpenCV pencerelerini kapat
try:
    cv2.destroyAllWindows()
    print("🖼️ OpenCV pencereleri kapatıldı")
except:
    pass

print("🏁 Program tamamlandı.")
print("📊 Telemetri thread'i durduruldu")
print("📷 Kamera sistemi kapatıldı")
