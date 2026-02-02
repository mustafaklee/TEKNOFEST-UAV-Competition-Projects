from pymavlink import mavutil
import time


def angle_to_pwm(angle):
    # 0° -> 1000us, 180° -> 2000us
    return int(1000 + (angle / 180.0) * 1000)

def send_servo(vehicle,channel, angle):
    pwm = angle_to_pwm(angle)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,     # SERVOx numarası (ör: 10)
        pwm,         # mikro-saniye
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"[SERVO] CH{channel} -> {angle}° ({pwm}us)")

def box_open(vehicle,channel, wait=2.0,degree=180):
    # Kutu kapağını AÇ (180°)
    send_servo(vehicle,channel, degree)
    time.sleep(wait)

def box_close(vehicle,channel, wait=1.0,degree=0):
    # Kutu kapağını KAPAT (0°)
    send_servo(vehicle,channel, degree)
    time.sleep(wait)
