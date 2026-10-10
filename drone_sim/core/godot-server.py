import socket
import threading
import time
import numpy as np
from pathlib import Path

from .drone_profile import DroneProfile
from .state import DroneState
from .physics import PhysicsEngine
from .mixer import MotorMixer
from .weather import Weather
from .autopilot import Autopilot
from .forces import GRAVITY


class GodotServer:
    """TCP-сервер: шлёт состояние дрона в Godot 30 раз в секунду."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5000):
        self.host = host
        self.port = port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((host, port))
        self.server_sock.listen(1)
        self.client = None
        self.running = False

    def start(self):
        """Блокируется до подключения Godot, затем запускает симуляцию."""
        print(f"Ожидаю подключения Godot на {self.host}:{self.port}...")
        self.client, addr = self.server_sock.accept()
        print(f"Godot подключился: {addr}")
        self.running = True

    def send_state(self, state: DroneState):
        if not self.client:
            return
        # Формат: x y z roll pitch yaw (в градусах)
        msg = (f"{state.position[0]:.3f} {state.position[1]:.3f} {state.position[2]:.3f} "
               f"{np.degrees(state.attitude[0]):.2f} "
               f"{np.degrees(state.attitude[1]):.2f} "
               f"{np.degrees(state.attitude[2]):.2f}\n")
        try:
            self.client.sendall(msg.encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            print("Godot отключился")
            self.running = False

    def stop(self):
        self.running = False
        if self.client:
            self.client.close()
        self.server_sock.close()
