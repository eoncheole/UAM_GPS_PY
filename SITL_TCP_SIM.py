#!/usr/bin/env python3
"""
SITL TCP GPS 스푸핑 - 시뮬레이터 대체 모드
TCP 4560 포트로 직접 HIL_SENSOR + HIL_GPS 송신
"""
import os
os.environ['MAVLINK20'] = '1'

import time
import math
import struct
from pymavlink import mavutil

class SITLTCPSpoofer:
    def __init__(self, connection_string='tcpout:127.0.0.1:4560'):
        """
        TCP 연결로 시뮬레이터 역할 수행
        주의: 기존 시뮬레이터(Gazebo 등)와 충돌 방지 필요
        """
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=1,
            source_component=1
        )
        self.boot_time = time.time()
        self.sim_time_us = 0
        
    def get_sim_time_us(self):
        """Lockstep용 시뮬레이션 시간"""
        self.sim_time_us += 4000  # 250Hz = 4000us 간격
        return self.sim_time_us
        
    def send_hil_sensor(self, accel=(0, 0, -9.81), gyro=(0, 0, 0),
                        mag=(0.2, 0, 0.4), baro_alt=0):
        """
        HIL_SENSOR 메시지 (lockstep 동기화에 필수)
        250Hz로 전송 필요
        """
        abs_pressure = 1013.25 * math.exp(-baro_alt / 8500)  # 기압 계산
        
        self.master.mav.hil_sensor_send(
            self.get_sim_time_us(),  # time_usec
            accel[0], accel[1], accel[2],  # xacc, yacc, zacc (m/s²)
            gyro[0], gyro[1], gyro[2],     # xgyro, ygyro, zgyro (rad/s)
            mag[0], mag[1], mag[2],        # xmag, ymag, zmag (gauss)
            abs_pressure,                   # abs_pressure (hPa)
            0.0,                           # diff_pressure
            baro_alt,                      # pressure_alt (m)
            25.0,                          # temperature (°C)
            0x1FFF,                        # fields_updated (모든 필드)
            0                              # id
        )
        
    def send_hil_gps_spoof(self, lat_deg, lon_deg, alt_m):
        """스푸핑된 GPS 데이터 송신"""
        self.master.mav.hil_gps_send(
            self.sim_time_us,       # lockstep 시간과 동기화
            3,                      # fix_type: 3D
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            int(alt_m * 1000),
            100, 100,               # eph, epv
            0,                      # vel
            0, 0, 0,                # vn, ve, vd
            0,                      # cog
            12                      # satellites
        )
        
    def simulation_loop(self, spoof_lat, spoof_lon, spoof_alt, duration_s=60):
        """
        시뮬레이션 메인 루프
        HIL_SENSOR: 250Hz, HIL_GPS: 10Hz
        """
        print(f"TCP 시뮬레이션 시작 - 스푸핑 위치: {spoof_lat}, {spoof_lon}")
        
        start_time = time.time()
        sensor_interval = 1.0 / 250  # 250Hz
        gps_interval = 1.0 / 10      # 10Hz
        last_gps_time = 0
        
        while (time.time() - start_time) < duration_s:
            loop_start = time.time()
            
            # HIL_SENSOR 송신 (매 루프)
            self.send_hil_sensor(baro_alt=spoof_alt)
            
            # HIL_GPS 송신 (10Hz)
            if (time.time() - last_gps_time) >= gps_interval:
                self.send_hil_gps_spoof(spoof_lat, spoof_lon, spoof_alt)
                last_gps_time = time.time()
                
            # 액추에이터 명령 수신 (비차단)
            msg = self.master.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)
            if msg:
                # 필요시 액추에이터 처리
                pass
                
            # 타이밍 유지
            elapsed = time.time() - loop_start
            if elapsed < sensor_interval:
                time.sleep(sensor_interval - elapsed)
                
        print("시뮬레이션 종료")

def main():
    print("=== SITL TCP 시뮬레이터 대체 모드 ===")
    print("주의: 기존 시뮬레이터를 종료한 후 실행하세요")
    print()
    
    spoofer = SITLTCPSpoofer('tcpout:127.0.0.1:4560')
    
    # 스푸핑 위치로 시뮬레이션
    spoofer.simulation_loop(
        spoof_lat=35.1796,   # 부산
        spoof_lon=129.0756,
        spoof_alt=100.0,
        duration_s=120
    )

if __name__ == "__main__":
    main()