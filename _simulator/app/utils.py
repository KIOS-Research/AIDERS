import random
import subprocess
import threading

# print to the console and log file
def myPrint(_msg):
    print(_msg)
    with open("/logs/app_logs.txt", "a") as file:
        file.write(f"{_msg}\n")


def randomDirection(_stepSize, _previousDirection):
    directions = ['n', 'e', 's', 'w', 'ne', 'se', 'sw', 'nw']
    directions.remove(oppositeDirection(_previousDirection)) # avoid backtracking
    direction = random.choice(directions)
    if direction == 'n':
        return 0, _stepSize, direction
    elif direction == 'e':
        return _stepSize, 0, direction
    elif direction == 's':
        return 0, -_stepSize, direction
    elif direction == 'w':
        return -_stepSize, 0, direction
    elif direction == 'ne':
        return _stepSize, _stepSize, direction
    elif direction == 'se':
        return _stepSize, -_stepSize, direction
    elif direction == 'sw':
        return -_stepSize, -_stepSize, direction
    elif direction == 'nw':
        return -_stepSize, _stepSize, direction


def oppositeDirection(_direction):
    opposites = {'n': 's', 'e': 'w', 's': 'n', 'w': 'e', 'ne': 'sw', 'se': 'nw', 'sw': 'ne', 'nw': 'se'}
    return opposites.get(_direction)


def randomHeadingIncrement(_heading, _incrementRange):
    increment = random.choice([-1, 1])  # Randomly select either -1 or 1
    increment_value = increment * random.randint(*_incrementRange)
    _heading += increment_value
    _heading %= 360  # Ensure the heading stays within 0 and 359 degrees
    return _heading


def streamToRtmp(_input_file, _rtmp_url):
    while True:
        cmd = [
            'ffmpeg',
            '-hwaccel', 'auto',     # Automatically select hardware acceleration
            '-stream_loop', '-1',   # Loop the video indefinitely
            '-re',                  # limits the reading speed of the input
            '-i', _input_file,
            '-c:v', 'h264_nvenc',   # NVIDIA NVENC hardware-accelerated encoder
            # '-c:v', 'libx264',    # software-based encoding - 900% CPU
            '-b:v', '2000k',        # Set the video bitrate
            # '-r', '30',           # Set the output frame rate to 30 fps
            '-c:a', 'aac',
            '-an',                  # Disable audio
            '-f', 'flv',
            _rtmp_url
        ]
        # ffmpeg -hwaccel auto -stream_loop -1 -i ./SIM_Alpha.mp4 -c:v h264_nvenc -b:v 2000k -q:v 20 -r 30 -c:a aac -an -f flv rtmp://192.168.0.10/live/SIM_Alpha
        # ffmpeg -hwaccel auto -stream_loop -1 -re -i ./SIM_Alpha.mp4 -c:v h264_nvenc -b:v 2000k -c:a aac -an -f flv rtmp://192.168.0.10/live/SIM_Alpha
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print("Error:", e)

# returns true if a thread with the same name is already running
def threadStarted(_threadName):
    return any(thread.name == _threadName for thread in threading.enumerate())

