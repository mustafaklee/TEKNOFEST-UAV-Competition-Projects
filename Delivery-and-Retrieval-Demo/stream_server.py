# stream_server.py
from flask import Flask, Response
import threading
import time
import cv2

_app = Flask(__name__)
_latest_jpeg = None
_lock = threading.Lock()

def update_frame(bgr_image):
    """Ana koddaki her yeni BGR kareyi JPEG'e çevirip paylaş."""
    global _latest_jpeg
    ok, buf = cv2.imencode(".jpg", bgr_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if ok:
        with _lock:
            _latest_jpeg = buf.tobytes()

@_app.route("/video")
def video():
    def gen():
        boundary = b"--frame\r\n"
        while True:
            with _lock:
                frame = _latest_jpeg
            if frame is not None:
                yield boundary
                yield b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.03)  # ~30 FPS hedefi
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

def start_server(host="0.0.0.0", port=5001):
    """Flask'i arka planda başlat."""
    t = threading.Thread(target=_app.run, kwargs={"host": host, "port": port, "threaded": True, "use_reloader": False}, daemon=True)
    t.start()
