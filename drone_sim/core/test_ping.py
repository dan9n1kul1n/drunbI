import socket

# Минимальный TCP-сервер
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", 5000))
s.listen(1)
print("Ожидание подключения на порту 5000...")

conn, addr = s.accept()
print(f"Кто-то подключился: {addr}")

# Отправим тестовое сообщение
conn.sendall(b"1.0 2.0 3.0 0 0 0\n")
print("Сообщение отправлено. Ждём 5 секунд...")
import time; time.sleep(5)
conn.close()
s.close()
