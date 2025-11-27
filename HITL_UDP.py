#!/usr/bin/env python3
from pymavlink import mavutil
import time
import math

conn = mavutil.mavlink_connection('udp:127.0.0.1:14557')
conn.wait_heartbeat(timeout=10)
time.sleep(3)

conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
time.sleep(0.3)

path = [
    (37.53000, 126.92000, 450),
    (37.51000, 127.08000, 500),
    (37.46000, 127.00000, 550),
    (37.40000, 126.80000, 600),
    (37.35000, 126.60000, 650),
    (37.30000, 126.50000, 700),
]

def spoof(lat, lon, alt, hdg):
    ts = int(time.time() * 1e6)
    conn.mav.hil_gps_send(ts, 3, int(lat*1e7), int(lon*1e7), int(alt*1000), 30, 40, 1200, 0, 0, 0, 10, 0)
    conn.mav.hil_actuator_controls_send(ts, [0,0,0.60,0,0,0,0,0,0,0,0,0,0,0,0,0], 0, -1, 1)
    conn.mav.set_position_target_global_int_send(ts // 1000, conn.target_system, conn.target_component,
                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b110111111000,
                                                 int(lat*1e7), int(lon*1e7), int(alt), 0,0,0,0,0,0,0, math.radians(hdg))

def recv_feedback():
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        print(f"Received lat: {msg.lat/1e7}, lon: {msg.lon/1e7}, alt: {msg.alt/1000}")

for i in range(len(path)-1):
    lat1, lon1, alt1 = path[i]
    lat2, lon2, alt2 = path[i+1]
    steps = 400 if i == 0 else 300
    hdg = math.degrees(math.atan2(lon2-lon1, lat2-lat1))
    for s in range(steps + 1):
        t = s / steps
        lat = lat1 + (lat2-lat1)*t
        lon = lon1 + (lon2-lon1)*t
        alt = alt1 + (alt2-alt1)*t
        spoof(lat, lon, alt, hdg)
        recv_feedback()
        time.sleep(0.1)