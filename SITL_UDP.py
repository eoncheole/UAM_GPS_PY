#!/usr/bin/env python3
from pymavlink import mavutil
import time
import math
import socket

gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

mav = mavutil.mavlink_connection('udp:127.0.0.1:14557')
mav.wait_heartbeat(timeout=10)

mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
time.sleep(0.5)

path = [
    (37.53000, 126.92000, 450),
    (37.51000, 127.08000, 500),
    (37.46000, 127.00000, 550),
    (37.40000, 126.80000, 600),
    (37.35000, 126.60000, 650),
    (37.30000, 126.50000, 700),
]

def send_fake_gps(lat, lon, alt):
    msg = mav.mav.gps_input_encode(
        0, 0, 1<<3, int(time.time()*1000) % (2**32 - 1),
        0, int(lat*1e7), int(lon*1e7), int(alt*1000),
        15, 15, 5.0, 0, 0, 0, 10, 0, 0, 0
    )
    buf = msg.pack(mav.mav)
    gps_sock.sendto(buf, ('127.0.0.1', 14560))

def recv_feedback():
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        print(f"Received lat: {msg.lat/1e7}, lon: {msg.lon/1e7}, alt: {msg.alt/1000}")

def offboard_setpoint(lat, lon, alt, yaw_deg):
    mav.mav.set_position_target_global_int_send(
        int(time.time()*1000), mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(lat*1e7), int(lon*1e7), int(alt),
        0,0,0,0,0,0,0, math.radians(yaw_deg)
    )

print("SITL UDP 스푸핑 시작")
for i in range(len(path)-1):
    lat1, lon1, alt1 = path[i]
    lat2, lon2, alt2 = path[i+1]
    steps = 400 if i == 0 else 300
    yaw = math.degrees(math.atan2(lon2-lon1, lat2-lat1))
    for s in range(steps + 1):
        t = s / steps
        lat = lat1 + (lat2-lat1)*t
        lon = lon1 + (lon2-lon1)*t
        alt = alt1 + (alt2-alt1)*t
        send_fake_gps(lat, lon, alt)
        offboard_setpoint(lat, lon, alt, yaw)
        recv_feedback()
        time.sleep(0.1)

gps_sock.close()
print("SITL UDP 스푸핑 완료")