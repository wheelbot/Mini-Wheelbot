# MIT License

# Copyright (c) 2024 Henrik Hose

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import socket
import json
import pygame
import time

wheelbot_beta_1 = '192.168.10.101'
wheelbot_beta_2 = '192.168.10.102'
wheelbot_beta_3 = '192.168.10.103'
wheelbot_beta_4 = '192.168.10.104'

sleep_time = 0.2
yaw_delta = 90*sleep_time*3.14/180
drive_delta = 2*180*sleep_time*3.14/180

data = {
    "yaw_delta": 0.0,
    "drive_delta": 0.0
}

def send_data(sock, data):
    json_data = json.dumps(data) + "\n"
    print(f"Sending string: {json_data}")
    sock.sendall(json_data.encode('utf-8'))

def receive_data(sock):
    try:
        received_data = sock.recv(1024).decode('utf-8')
        if received_data.strip():
            print(f"Received string: {received_data.strip()}")
            odometry = json.loads(received_data)
            print(f"Odometry data: {odometry}")
    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON: {e}")
    except socket.error as e:
        print(f"Socket error: {e}")


json_data = json.dumps(data) + "\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((wheelbot_beta_1, 8888))
try:
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption('TCP JSON Sender')

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            data['drive_delta'] = drive_delta
        elif keys[pygame.K_s]:
            data['drive_delta'] = -drive_delta
        else:
            data['drive_delta'] = 0.0

        if keys[pygame.K_a]:
            data['yaw_delta'] = yaw_delta
        elif keys[pygame.K_d]:
            data['yaw_delta'] = -yaw_delta
        else:
            data['yaw_delta'] = 0


        send_data(sock, data)
        receive_data(sock)

        time.sleep(sleep_time)

except KeyboardInterrupt:
    pass
finally:
    sock.close()
