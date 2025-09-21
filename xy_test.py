# udp_test.py
import socket

# ▼▼▼ 확인하고 싶은 포트 번호를 여기에 입력하세요 ▼▼▼
LISTEN_PORT = 5005 
# ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    sock.bind(('0.0.0.0', LISTEN_PORT))
    print(f"UDP 포트 {LISTEN_PORT}에서 수신 대기 중...")

    while True:
        data, addr = sock.recvfrom(1024) # 데이터 수신
        message = data.decode(errors='ignore').strip()
        print(f"IP {addr} 로부터 메시지 수신: '{message}' (raw: {data})")

except KeyboardInterrupt:
    print("\n프로그램 종료.")
finally:
    sock.close()