import socket
import threading

import matplotlib.pyplot as plt
import numpy as np

PORT = 4210
BUFFER_SIZE = 256

matrixL = np.zeros((15, 7))
matrixR = np.zeros((15, 7))
lock = threading.Lock()

# Setup socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))
sock.settimeout(1.0)

# Setup plot
plt.ion()
fig, (axL, axR) = plt.subplots(1, 2, figsize=(10, 5))
imL = axL.imshow(matrixL, cmap="plasma", vmin=0, vmax=1024)
imR = axR.imshow(matrixR, cmap="plasma", vmin=0, vmax=1024)
axL.set_title("Linkervoet")
axR.set_title("Rechtervoet")
plt.tight_layout()


def parse_matrix(data):
    values = [(data[1 + i * 2] << 8) | data[1 + i * 2 + 1] for i in range(105)]
    return np.array(values).reshape(15, 7)


def udp_listener():
    global matrixL, matrixR
    while True:
        try:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            if len(data) == 212 and data[0] in [ord("L"), ord("R")]:
                foot = chr(data[0])
                matrix = parse_matrix(data)
                print(f"Ontvangen matrix voor voet: {foot}")
                with lock:
                    if foot == "L":
                        matrixL = matrix
                    else:
                        matrixR = matrix
        except socket.timeout:
            continue
        except Exception as e:
            print(f"❌ Fout bij ontvangen: {e}")


# Start netwerk thread
threading.Thread(target=udp_listener, daemon=True).start()

# Main loop for visual updates
while True:
    with lock:
        imL.set_data(matrixL)
        imR.set_data(matrixR)
    plt.pause(0.05)
