import socket

HOST = "10.114.214.229"  # ESP32 IP — change to match your setup
PORT = 8080

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello from Python")
    print(f"Sent to {HOST}:{PORT}")
