import subprocess
import os
import threading
import time
import json
import pty
import select
import termios
import struct
import fcntl
from typing import List

from aiohttp import web
import socketio
import zipfile
import io
from pathlib import Path
import shutil


PROJECT_LOCATION = "/catkin_ws/src"

class BashProcess:
    def __init__(self, tid) -> None:
        self.tid = tid
        self.fd = None
        self.child_pid = None
        self.cmd = "bash"
        self.is_child = None

    def start(self):
        pid, fd = pty.fork()
        if pid == 0:
            self.is_child = True
            subprocess.run(self.cmd)
        else:
            self.is_child = False
            self.fd = fd
            self.child_pid = pid

class DockerThread(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd

    def run(self):
        subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)

    def call(self):
        subprocess.call(self.cmd, shell=True, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True) 


sio = socketio.AsyncServer(cors_allowed_origins = "*")
app = web.Application()
bash_processes: List[BashProcess] = []


def start_xserver(display):
        xserver_cmd = f"/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf {display}"
        xserver_thread = DockerThread(xserver_cmd)
        xserver_thread.start()

def start_vnc(display, internal_port, external_port):
        # Start VNC server without password, forever running in background
        x11vnc_cmd = f"x11vnc -display {display} -nopw -forever -xkb -bg -rfbport {internal_port}"
        x11vnc_thread = DockerThread(x11vnc_cmd)
        x11vnc_thread.start()

        # Start noVNC with default port 6080 listening to VNC server on 5900
        novnc_cmd = f"/noVNC/utils/launch.sh --listen {external_port} --vnc localhost:{internal_port}"
        novnc_thread = DockerThread(novnc_cmd)
        novnc_thread.start()


def set_winsize(fd, row, col, xpix=0, ypix=0):
    winsize = struct.pack("HHHH", row, col, xpix, ypix)
    fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)

async def read_and_forward_pty_output(sid, bash_process: BashProcess):
    max_read_bytes = 1024 * 20
    while True:
        await sio.sleep(0.01)
        if bash_process.fd is not None:
            timeout_sec = 0
            (data_ready, _, _) = select.select([bash_process.fd], [], [], timeout_sec)
            if data_ready:
                output = os.read(bash_process.fd, max_read_bytes).decode()
                print("Sending data", output)
                await sio.emit("pty-output", {"output": output, "tid": bash_process.tid})


@sio.event
async def start_gazebo(sid, data):
    world = data["world"]
    if world == "empty":
        gazebo_cmd = f"DISPLAY=:0 roslaunch gazebo_ros empty_world.launch"
    elif world == "mud":
        gazebo_cmd = f"DISPLAY=:0 roslaunch gazebo_ros mud_world.launch"
    else:
        gazebo_cmd = f"DISPLAY=:0 roslaunch gazebo_ros empty_world.launch"

    gazebo_thread = DockerThread(gazebo_cmd)
    gazebo_thread.start()


@sio.event
async def stop_gazebo(sid, data):
    cmd = ["pkill", "-9", "-f", "roslaunch gazebo_ros"]
    try:
        subprocess.call(cmd, stdout=subprocess.PIPE, bufsize=1024, universal_newlines=True)
    except:
        pass

@sio.event
async def start_bash(sid, data):

    global bash_processes
    tid = data["tid"]
    print("Starting bash:", tid)
    if len(bash_processes) != tid:
        return

    bash_process = BashProcess(tid)
    bash_process.start()
    print("Is child:", bash_process.is_child)
    if bash_process.child_pid is not None:
        bash_processes.append(bash_process)
        print("FD:", bash_process.fd)
        set_winsize(bash_process.fd, data["rows"], data["cols"])
        sio.start_background_task(read_and_forward_pty_output, sid, bash_process)

@sio.on("pty-input")
async def pty_input(sid, data):
    tid = data["tid"]
    if tid >= len(bash_processes):
        return
    bash_process = bash_processes[tid]
    if bash_process.fd is not None:
        os.write(bash_process.fd, data["input"].encode())

@sio.event
async def resize(sid, data):
    tid = data["tid"]
    bash_process = bash_processes[tid]
    if bash_process.fd is not None:
        set_winsize(bash_process.fd, data["rows"], data["cols"])

@sio.on("upload")
async def receive_file(sid, data):
    filename = data["filename"]
    file = data["file"]
    filepath = Path(PROJECT_LOCATION, filename)
    directory = Path(PROJECT_LOCATION, filepath.stem)

    if directory.exists():
        shutil.rmtree(str(directory.absolute()))

    with filepath.open("wb") as f:
        f.write(file)

    zf = zipfile.ZipFile(io.BytesIO(file), "r")
    zf.extractall(str(directory.absolute()))
    zf.close()


def main():
    start_xserver(':0')
    time.sleep(5)
    start_vnc(':0', 5900, 6080)
    time.sleep(5)
    sio.attach(app)

    web.run_app(app, host="0.0.0.0", port=8765)


if __name__ == '__main__':
    main()