#!/bin/bash

# Example script to demonstrate daemon mode usage

echo "Turret Daemon Test Script"
echo "========================="
echo ""
echo "Example commands to test the daemon mode:"
echo ""
echo "1. Start daemon on Unix socket:"
echo "   turret daemon --socket /tmp/turret.sock"
echo ""
echo "2. Start daemon on TCP port:"
echo "   turret daemon --tcp-port 8080"
echo ""
echo "3. Send commands to daemon via Unix socket:"
echo '   echo '"'"'{"cmd":"set_attitude","pitch":10,"roll":0,"yaw":-15}'"'"' | nc -U /tmp/turret.sock'
echo '   echo '"'"'{"cmd":"get_status"}'"'"' | nc -U /tmp/turret.sock'
echo '   echo '"'"'{"cmd":"center"}'"'"' | nc -U /tmp/turret.sock'
echo ""
echo "4. Send commands to daemon via TCP:"
echo '   echo '"'"'{"cmd":"set_attitude","pitch":10,"roll":0,"yaw":-15}'"'"' | nc localhost 8080'
echo ""
echo "5. Object tracking integration example (Python):"
cat << 'EOF'

import json
import socket

# Connect to turret daemon
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect('/tmp/turret.sock')

def send_command(cmd):
    sock.send((json.dumps(cmd) + '\n').encode())
    response = sock.recv(1024).decode()
    return json.loads(response)

# Example: Track object position
while tracking:
    # Your object detection code here
    object_x, object_y = detect_object()

    # Convert to gimbal angles
    pitch = calculate_pitch(object_y)
    yaw = calculate_yaw(object_x)

    # Send to gimbal
    result = send_command({
        "cmd": "set_attitude",
        "pitch": pitch,
        "roll": 0,
        "yaw": yaw
    })

EOF