#!/usr/bin/env python3
"""Control web para PAPA mediante el tópico /comando_arduino."""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HOST = os.environ.get("PAPA_WEB_HOST", "192.168.1.130")
PORT = int(os.environ.get("PAPA_WEB_PORT", "8080"))
COMMAND_TIMEOUT = 0.30


PAGE = """<!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Control PAPA</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { background: #101318; margin: 0; min-height: 100vh; display: grid;
           place-items: center; touch-action: none; }
    main { width: min(92vw, 430px); text-align: center; }
    h1 { font-size: 1.7rem; margin: 0 0 1.4rem; }
    .pad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
    button { aspect-ratio: 1.25; border: 0; border-radius: 18px; color: white;
             background: #2563eb; font-size: 2.8rem; font-weight: 700;
             box-shadow: 0 5px 0 #123b91; user-select: none;
             -webkit-tap-highlight-color: transparent; }
    button:active, button.active { background: #16a34a; box-shadow: 0 2px 0 #0b672d;
                                   transform: translateY(3px); }
    #stop { background: #dc2626; box-shadow: 0 5px 0 #8e1515; font-size: 1.3rem; }
    #stop:active { background: #ef4444; }
    .empty { visibility: hidden; }
    #status { min-height: 1.5em; margin-top: 1.5rem; color: #aab4c3; }
  </style>
</head>
<body>
  <main>
    <h1>Control PAPA</h1>
    <div class="pad">
      <span class="empty"></span>
      <button data-command="up" aria-label="Avanzar">▲</button>
      <span class="empty"></span>
      <button data-command="left" aria-label="Girar a la izquierda">◀</button>
      <button id="stop" aria-label="Detener">DETENER</button>
      <button data-command="right" aria-label="Girar a la derecha">▶</button>
      <span class="empty"></span>
      <button data-command="down" aria-label="Retroceder">▼</button>
      <span class="empty"></span>
    </div>
    <div id="status">Listo</div>
  </main>
  <script>
    const status = document.getElementById('status');
    let timer = null;
    let activeButton = null;

    async function send(command) {
      try {
        const response = await fetch('/command', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({command})
        });
        if (!response.ok) throw new Error('HTTP ' + response.status);
        status.textContent = command === 'stop' ? 'Detenido' : 'Moviendo';
      } catch (error) {
        status.textContent = 'Error de conexión';
        console.error(error);
      }
    }

    function stop() {
      if (timer !== null) {
        clearInterval(timer);
        timer = null;
      }
      if (activeButton) activeButton.classList.remove('active');
      activeButton = null;
      send('stop');
    }

    function start(button) {
      stop();
      activeButton = button;
      activeButton.classList.add('active');
      send(button.dataset.command);
      timer = setInterval(() => send(button.dataset.command), 100);
    }

    document.querySelectorAll('[data-command]').forEach(button => {
      button.addEventListener('pointerdown', event => {
        event.preventDefault();
        start(button);
      });
    });
    document.addEventListener('pointerup', stop);
    document.addEventListener('pointercancel', stop);
    document.getElementById('stop').addEventListener('pointerdown', event => {
      event.preventDefault();
      stop();
    });
    window.addEventListener('blur', stop);
  </script>
</body>
</html>
"""


class WebTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("control_web")
        self.publisher = self.create_publisher(String, "/comando_arduino", 10)
        self.max_speed = 100
        self.step = 1
        self.min_pwm = 60
        self.target_l = 0
        self.target_r = 0
        self.curr_l = 0
        self.curr_r = 0
        self.last_command = 0.0
        self._lock = threading.Lock()
        self._active = False
        self.timer = self.create_timer(0.05, self._update_motion)
        self.get_logger().info(f"Control web disponible en http://{HOST}:{PORT}")

    def set_command(self, command: str) -> None:
        targets = {
            "up": (self.max_speed, self.max_speed),
            "down": (-self.max_speed, -self.max_speed),
            "left": (-self.max_speed, self.max_speed),
            "right": (self.max_speed, -self.max_speed),
        }
        with self._lock:
            if command == "stop":
                self.target_l = self.target_r = 0
                self.curr_l = self.curr_r = 0
                self._active = False
                self._publish_locked()
                return
            if command not in targets:
                raise ValueError(f"Comando desconocido: {command}")
            self.target_l, self.target_r = targets[command]
            self.last_command = time.monotonic()
            self._active = True

    def _approach(self, current: int, target: int) -> int:
        if current == 0 and target:
            return self.min_pwm if target > 0 else -self.min_pwm
        if current < target:
            return min(current + self.step, target)
        if current > target:
            return max(current - self.step, target)
        return current

    def _update_motion(self) -> None:
        with self._lock:
            if self._active and time.monotonic() - self.last_command > COMMAND_TIMEOUT:
                self.target_l = self.target_r = 0
                self.curr_l = self.curr_r = 0
                self._active = False
                self._publish_locked()
                return
            if not self._active:
                return
            self.curr_l = self._approach(self.curr_l, self.target_l)
            self.curr_r = self._approach(self.curr_r, self.target_r)
            self._publish_locked()

    def _publish_locked(self) -> None:
        message = String()
        message.data = f"VEL,{self.curr_l},{self.curr_r}"
        self.publisher.publish(message)

    def destroy_node(self) -> None:
        with self._lock:
            self.curr_l = self.curr_r = 0
            self._publish_locked()
        super().destroy_node()


class ControlRequestHandler(BaseHTTPRequestHandler):
    node: WebTeleopNode

    def do_GET(self) -> None:
        if urlparse(self.path).path == "/":
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404, "Ruta no encontrada")

    def do_POST(self) -> None:
        if urlparse(self.path).path != "/command":
            self.send_error(404, "Ruta no encontrada")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            data = json.loads(self.rfile.read(length))
            self.node.set_command(data["command"])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError) as error:
            self.send_error(400, f"Comando inválido: {error}")
            return
        self.send_response(204)
        self.end_headers()

    def log_message(self, format: str, *args: object) -> None:
        self.node.get_logger().info(format % args)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebTeleopNode()
    try:
        server = ThreadingHTTPServer((HOST, PORT), ControlRequestHandler)
    except OSError as error:
        node.get_logger().error(f"No se pudo abrir http://{HOST}:{PORT}: {error}")
        node.destroy_node()
        rclpy.shutdown()
        raise

    ControlRequestHandler.node = node
    server_thread = threading.Thread(
        target=server.serve_forever,
        name="control-web-http",
        daemon=True,
    )
    server_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
