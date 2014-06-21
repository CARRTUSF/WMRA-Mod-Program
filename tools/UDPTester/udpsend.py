import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 7500
COMMAND = "COMMAND CAMERA_VIEW_CLOSE 400 -150 480 0 0 0\n"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.sendto(bytes(COMMAND, "utf-8"), (UDP_IP, UDP_PORT))
