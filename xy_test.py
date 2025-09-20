import socket

# --- 설정 ---
TARGET_IP = "192.168.0.137"  # 데이터를 보낼 IP 주소
TARGET_PORT = 5005           # 데이터를 보낼 포트 번호
MESSAGE = "1,1"              # 보낼 메시지

# --- UDP 소켓 생성 및 전송 ---
print(f"'{MESSAGE}' 메시지를 {TARGET_IP}:{TARGET_PORT}로 전송합니다.")

# 1. 소켓 생성 (AF_INET: IPv4, SOCK_DGRAM: UDP)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    # 2. 메시지를 바이트(byte)로 변환하여 지정된 주소로 전송
    sock.sendto(MESSAGE.encode('utf-8'), (TARGET_IP, TARGET_PORT))
    print("메시지를 성공적으로 전송했습니다.")

except Exception as e:
    print(f"메시지 전송 중 오류 발생: {e}")

finally:
    # 3. 소켓 리소스 정리
    sock.close()