from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import time

WAYPOINTS_FILE = "/home/mustafa/drone_guncel/rota.waypoints"
CONNECTION_STRING = 'udp:127.0.0.1:14550'

# 1. Araca bağlan
print("Drone'a bağlanılıyor...")
vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=57600)

# 2. Mevcut görevleri temizle
cmds = vehicle.commands
cmds.clear()

# 3. Waypoint dosyasını oku
def parse_waypoints(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    wp_list = []
    for line in lines[1:]:  # İlk satır başlıktır
        parts = line.strip().split('\t')
        if len(parts) < 12:
            continue
        
        # Command nesnesini doğru parametrelerle oluştur
        command = Command(
            0,  # target_system
            0,  # target_component
            0,  # seq
            int(parts[2]),      # frame
            int(parts[3]),      # command
            int(parts[4]),      # current
            int(parts[5]),      # autocontinue
            float(parts[6]),    # param1
            float(parts[7]),    # param2
            float(parts[8]),    # param3
            float(parts[9]),    # param4
            float(parts[10]),   # x (latitude)
            float(parts[11]),   # y (longitude)
            float(parts[12] if len(parts) > 12 else 0.0)  # z (altitude)
        )
        wp_list.append(command)
    
    return wp_list

print("Görev yükleniyor...")
waypoints = parse_waypoints(WAYPOINTS_FILE)

for cmd in waypoints:
    cmds.add(cmd)

cmds.upload()

# 4. GUIDED moda geç
print("GUIDED moda geçiliyor...")
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == "GUIDED":
    print("GUIDED moda geçiliyor...")
    time.sleep(1)

# 5. ARM işlemi
print("Arm ediliyor...")
vehicle.armed = True
while not vehicle.armed:
    print("Arm ediliyor...")
    time.sleep(1)

# 6. Kalkış (isteğe bağlı - AUTO mod bunu otomatik yapabilir)
print("Kalkış yapılıyor...")
vehicle.simple_takeoff(10)  # 10 metre yüksekliğe çık

# Kalkış tamamlanana kadar bekle
while True:
    print(f"Yükseklik: {vehicle.location.global_relative_frame.alt}")
    if vehicle.location.global_relative_frame.alt >= 10 * 0.95:
        print("Hedef yüksekliğe ulaşıldı")
        break
    time.sleep(1)

# 7. AUTO moda geç
print("AUTO moda geçiliyor ve görev başlatılıyor...")
vehicle.mode = VehicleMode("AUTO")
while not vehicle.mode.name == "AUTO":
    print("AUTO moda geçiliyor...")
    time.sleep(1)

print("Görev başladı. Drone uçuyor...")

# Opsiyonel: Görev tamamlanana kadar bekle
while vehicle.commands.next < len(waypoints):
    print(f"İlerleme: {vehicle.commands.next}/{len(waypoints)}")
    print(f"Mevcut konum: Lat:{vehicle.location.global_frame.lat}, Lon:{vehicle.location.global_frame.lon}, Alt:{vehicle.location.global_relative_frame.alt}")
    time.sleep(2)

print("Görev tamamlandı!")

# RTL (Return to Launch) modu
print("RTL moduna geçiliyor...")
vehicle.mode = VehicleMode("RTL")

# İstersen bağlantıyı sonlandırabilirsin
time.sleep(5)
vehicle.close()