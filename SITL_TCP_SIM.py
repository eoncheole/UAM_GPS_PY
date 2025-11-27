#!/usr/bin/env python3
"""
SITL TCP GPS 스푸핑 - Windows 호환 시뮬레이터 대체 모드
PX4 v1.14 + pymavlink 2.4.49

TCP 서버 모드: PX4 SITL이 이 스크립트에 연결됨
"""
import os
os.environ['MAVLINK20'] = '1'

import time
import math
import sys
import socket
import threading
from pymavlink import mavutil

class SITLTCPSimulator:
    def __init__(self, host='0.0.0.0', port=4560):
        """
        TCP 서버로 동작하여 PX4 SITL의 연결을 받음
        
        주의: 기존 시뮬레이터(X-Plane, Gazebo)를 종료해야 함
        """
        self.host = host
        self.port = port
        self.master = None
        self.boot_time = time.time()
        self.sim_time_us = 0
        self.running = False
        self.client_socket = None
        
    def start_server(self, timeout=60):
        """TCP 서버 시작 및 PX4 연결 대기"""
        print(f"TCP 서버 시작: {self.host}:{self.port}")
        print("PX4 SITL 연결 대기 중...")
        print("(먼저 기존 시뮬레이터를 종료하고 SITL을 재시작하세요)")
        
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            server_sock.bind((self.host, self.port))
            server_sock.listen(1)
            server_sock.settimeout(timeout)
            
            self.client_socket, addr = server_sock.accept()
            print(f"PX4 연결됨: {addr}")
            
            # pymavlink 연결 래퍼
            self.master = mavutil.mavlink_connection(
                f'tcp:{addr[0]}:{addr[1]}',
                source_system=1,
                source_component=1,
                input=False
            )
            
            # 실제로는 소켓을 직접 사용
            self.running = True
            return True
            
        except socket.timeout:
            print("타임아웃: PX4 연결 없음")
            return False
        except Exception as e:
            print(f"서버 오류: {e}")
            return False
            
    def get_sim_time_us(self):
        """Lockstep 시뮬레이션 시간 (4000us = 250Hz)"""
        self.sim_time_us += 4000
        return self.sim_time_us & 0xFFFFFFFFFFFFFFFF
        
    def pack_hil_sensor(self, accel, gyro, mag, baro_alt):
        """HIL_SENSOR 메시지 패킹"""
        abs_pressure = 1013.25 * math.exp(-baro_alt / 8500)
        
        # MAVLink 2.0 HIL_SENSOR 메시지 수동 구성
        msg = self.master.mav.hil_sensor_encode(
            self.get_sim_time_us(),
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            mag[0], mag[1], mag[2],
            abs_pressure,
            0.0,  # diff_pressure
            baro_alt,
            25.0,  # temperature
            0x1FFF,  # fields_updated
            0  # id
        )
        return msg.pack(self.master.mav)
        
    def pack_hil_gps(self, lat_deg, lon_deg, alt_m):
        """HIL_GPS 메시지 패킹"""
        msg = self.master.mav.hil_gps_encode(
            self.sim_time_us,
            3,  # fix_type
            int(lat_deg * 1e7),
            int(lon_deg * 1e7),
            int(alt_m * 1000),
            100, 100,  # eph, epv
            0,  # vel
            0, 0, 0,  # vn, ve, vd
            0,  # cog
            12  # satellites
        )
        return msg.pack(self.master.mav)
        
    def send_raw(self, data):
        """원시 바이트 전송"""
        if self.client_socket:
            try:
                self.client_socket.send(data)
                return True
            except:
                return False
        return False
        
    def recv_raw(self, size=1024):
        """원시 바이트 수신 (비차단)"""
        if self.client_socket:
            try:
                self.client_socket.setblocking(False)
                data = self.client_socket.recv(size)
                return data
            except BlockingIOError:
                return None
            except:
                return None
        return None
        
    def run_simulation(self, spoof_lat, spoof_lon, spoof_alt, duration_s=60):
        """
        시뮬레이션 메인 루프
        
        HIL_SENSOR: 250Hz (lockstep 동기화 필수)
        HIL_GPS: 10Hz
        """
        print(f"\n=== TCP 시뮬레이션 시작 ===")
        print(f"스푸핑 위치: {spoof_lat:.6f}, {spoof_lon:.6f}, {spoof_alt:.1f}m")
        print(f"지속시간: {duration_s}초")
        
        start_time = time.time()
        sensor_interval = 1.0 / 250  # 250Hz
        gps_counter = 0
        msg_count = 0
        
        # 초기 센서 값
        accel = (0.0, 0.0, -9.81)
        gyro = (0.0, 0.0, 0.0)
        mag = (0.2, 0.0, 0.4)
        
        while self.running and (time.time() - start_time) < duration_s:
            loop_start = time.time()
            
            # HIL_SENSOR 전송 (매 루프)
            sensor_data = self.pack_hil_sensor(accel, gyro, mag, spoof_alt)
            self.send_raw(sensor_data)
            msg_count += 1
            
            # HIL_GPS 전송 (25회마다 = 10Hz)
            gps_counter += 1
            if gps_counter >= 25:
                gps_data = self.pack_hil_gps(spoof_lat, spoof_lon, spoof_alt)
                self.send_raw(gps_data)
                gps_counter = 0
                
            # 액추에이터 명령 수신 (비차단)
            recv_data = self.recv_raw()
            if recv_data:
                # HIL_ACTUATOR_CONTROLS 파싱 가능
                pass
                
            # 진행 상황 출력 (5초마다)
            if msg_count % 1250 == 0:
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] 센서: {msg_count}, GPS: {msg_count // 25}")
                
            # 타이밍 유지
            elapsed = time.time() - loop_start
            if elapsed < sensor_interval:
                time.sleep(sensor_interval - elapsed)
                
        print(f"\n시뮬레이션 종료: 총 {msg_count} 메시지")
        
    def stop(self):
        """시뮬레이션 중지"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()


class SimpleTCPSpoofer:
    """
    간단한 TCP 클라이언트 모드 (기존 시뮬레이터와 병행)
    """
    def __init__(self, host='127.0.0.1', port=14540):
        """
        TCP 클라이언트로 mavlink-router에 연결
        """
        self.host = host
        self.port = port
        self.boot_time = time.time()
        
        # TCP 클라이언트 연결
        connection_string = f'tcp:{host}:{port}'
        print(f"TCP 연결: {connection_string}")
        
        self.master = mavutil.mavlink_connection(
            connection_string,
            source_system=255,
            source_component=190
        )
        
    def wait_heartbeat(self, timeout=30):
        """하트비트 대기"""
        print("하트비트 대기 중...")
        msg = self.master.wait_heartbeat(timeout=timeout)
        if msg:
            print(f"연결됨! System: {self.master.target_system}")
            return True
        print("타임아웃")
        return False
        
    def get_timestamp_us(self):
        """안전한 타임스탬프"""
        return int((time.time() - self.boot_time) * 1e6) & 0xFFFFFFFFFFFFFFFF
        
    def send_hil_gps(self, lat, lon, alt):
        """HIL_GPS 전송"""
        self.master.mav.hil_gps_send(
            self.get_timestamp_us(),
            3,  # fix_type
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000),
            100, 100,
            0, 0, 0, 0,
            0, 12
        )
        
    def spoof_loop(self, lat, lon, alt, duration_s=60, rate_hz=10):
        """스푸핑 루프"""
        print(f"GPS 스푸핑: {lat}, {lon}, {alt}m")
        
        interval = 1.0 / rate_hz
        start_time = time.time()
        
        while (time.time() - start_time) < duration_s:
            self.send_hil_gps(lat, lon, alt)
            time.sleep(interval)
            
        print("완료")


def main():
    print("=" * 60)
    print("SITL TCP GPS 스푸핑 - Windows 호환")
    print("=" * 60)
    print()
    print("모드 선택:")
    print("1. TCP 서버 모드 (시뮬레이터 대체 - X-Plane 종료 필요)")
    print("2. TCP 클라이언트 모드 (mavlink-router 연결)")
    
    mode = input("\n선택 (1-2): ").strip()
    
    BUSAN = (35.1796, 129.0756, 100.0)
    
    if mode == '1':
        print("\n=== TCP 서버 모드 ===")
        print("주의: X-Plane 또는 다른 시뮬레이터를 먼저 종료하세요!")
        print("그 후 PX4 SITL을 재시작하면 이 스크립트에 연결됩니다.")
        
        input("준비되면 Enter를 누르세요...")
        
        sim = SITLTCPSimulator(host='0.0.0.0', port=4560)
        
        if sim.start_server(timeout=120):
            sim.run_simulation(
                spoof_lat=BUSAN[0],
                spoof_lon=BUSAN[1],
                spoof_alt=BUSAN[2],
                duration_s=120
            )
            sim.stop()
        else:
            print("연결 실패")
            
    elif mode == '2':
        print("\n=== TCP 클라이언트 모드 ===")
        
        host = input("호스트 (기본: 127.0.0.1): ").strip() or '127.0.0.1'
        port = int(input("포트 (기본: 14540): ").strip() or '14540')
        
        spoofer = SimpleTCPSpoofer(host=host, port=port)
        
        if spoofer.wait_heartbeat():
            spoofer.spoof_loop(
                lat=BUSAN[0],
                lon=BUSAN[1],
                alt=BUSAN[2],
                duration_s=120,
                rate_hz=10
            )
        else:
            print("연결 실패")
    else:
        print("잘못된 선택")


if __name__ == "__main__":
    main()