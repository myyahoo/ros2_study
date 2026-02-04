"""
Serial Communication Diagnostic Tool
시리얼 통신 진단 및 테스트
"""

import serial
import time
import sys


def test_serial_connection(port='/dev/ttyACM1', baudrate=115200):
    """시리얼 포트 연결 테스트"""
    print(f"\n📡 시리얼 포트 연결 테스트")
    print(f"   포트: {port}")
    print(f"   속도: {baudrate} baud\n")
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=2)
        print(f"✓ 포트 연결 성공!")
        time.sleep(2)  # Arduino 재부팅 대기
        
        # 포트 정보 출력
        print(f"\n포트 정보:")
        print(f"  - 이름: {ser.name}")
        print(f"  - 속도: {ser.baudrate}")
        print(f"  - 타임아웃: {ser.timeout}")
        
        return ser
    
    except Exception as e:
        print(f"✗ 포트 연결 실패: {e}")
        sys.exit(1)


def test_send_command(ser, command, description=""):
    """명령 전송 테스트"""
    print(f"\n📤 명령 전송: {command}")
    if description:
        print(f"   설명: {description}")
    
    try:
        # 명령 전송
        cmd = command if command.endswith('\n') else command + '\n'
        ser.write(cmd.encode('utf-8'))
        print(f"   ✓ 전송 완료")
        
        # 응답 대기
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"📥 응답 수신: {response}")
            return response
        else:
            print(f"   응답 없음")
            return None
    
    except Exception as e:
        print(f"   ✗ 전송 실패: {e}")
        return None


def run_diagnostic():
    """전체 진단 실행"""
    print("\n" + "="*60)
    print("🤖 SO ARM 101 시리얼 통신 진단 도구")
    print("="*60)
    
    # 포트 연결
    ser = test_serial_connection()
    
    # 테스트 시퀀스
    print("\n" + "="*60)
    print("📋 테스트 시퀀스")
    print("="*60)
    
    # 1. Home 명령
    test_send_command(ser, "HOME", "모든 관절을 홈 위치(0도)로 이동")
    time.sleep(2)
    
    # 2. 단일 관절 제어 (J1만 45도)
    test_send_command(ser, "MOVE:45.0,0.0,0.0,0.0,0.0,0.0", "J1 = 45도")
    time.sleep(2)
    
    # 3. 모든 관절 제어
    test_send_command(ser, "MOVE:30.0,30.0,30.0,30.0,30.0,30.0", "모든 관절 = 30도")
    time.sleep(2)
    
    # 4. 속도 설정
    test_send_command(ser, "SPEED:50", "속도 = 50%")
    time.sleep(1)
    
    # 5. Home으로 복귀
    test_send_command(ser, "HOME", "Home으로 돌아가기")
    time.sleep(2)
    
    # 포트 종료
    ser.close()
    print("\n✓ 진단 완료\n")


def manual_test(port='/dev/ttyACM1', baudrate=115200):
    """수동 테스트 모드"""
    print("\n" + "="*60)
    print("⌨️  수동 테스트 모드")
    print("="*60)
    print("명령 예시:")
    print("  HOME - 홈 위치로 이동")
    print("  MOVE:45,0,0,0,0,0 - J1=45도")
    print("  STOP - 정지")
    print("  SPEED:75 - 속도 75%")
    print("  quit - 종료\n")
    
    ser = test_serial_connection(port, baudrate)
    
    try:
        while True:
            try:
                cmd = input("명령 입력: ").strip()
                
                if cmd.lower() == 'quit':
                    break
                
                if cmd:
                    test_send_command(ser, cmd)
                    time.sleep(0.5)
            
            except KeyboardInterrupt:
                break
    
    finally:
        ser.close()
        print("\n✓ 종료\n")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='SO ARM 101 시리얼 진단 도구')
    parser.add_argument('--port', default='/dev/ttyACM1', help='시리얼 포트 (기본값: /dev/ttyACM1)')
    parser.add_argument('--baud', type=int, default=115200, help='보드레이트 (기본값: 115200)')
    parser.add_argument('--manual', action='store_true', help='수동 테스트 모드')
    
    args = parser.parse_args()
    
    if args.manual:
        manual_test(args.port, args.baud)
    else:
        run_diagnostic()
