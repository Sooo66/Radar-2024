import time
from flask import Flask, render_template, request, jsonify, Response
import mmap
import posix_ipc
import struct
import cv2
import numpy as np

shm_name = "srm_viewer_web"
shm_size = 0x100000

path = "config.toml"
app = Flask(__name__)


def read_file(path):
    with open(path, "r") as f:
        data = f.read()
    return data


def write_file(path, data):
    with open(path, "w") as f:
        f.write(data)


def generate():
    shm = posix_ipc.SharedMemory(shm_name)
    shm_map = mmap.mmap(shm.fd, shm_size, mmap.MAP_SHARED, mmap.PROT_READ)
    while True:
        frame_size = struct.unpack("i", shm_map[:4])[0]
        frame_data = shm_map[4:frame_size]
        if frame_data is None:
            continue
        show_frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), -1)
        cv2.imshow("frame", show_frame)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_data + b"\r\n\r\n"
        )
        time.sleep(1 / 40)  # 视频显示在30帧左右即可


@app.route("/")
def index():
    config = read_file(path)
    return render_template("video.html", config=config)


@app.route("/VideoStream")
def VideoStream():
    return Response(
        generate(), content_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/save", methods=["POST"])
def save():
    data = request.form["content"]
    write_file(path, data)
    return jsonify({"status": "success"})


if __name__ == "__main__":
    app.run("0.0.0.0", "9003", True, threaded=True)
