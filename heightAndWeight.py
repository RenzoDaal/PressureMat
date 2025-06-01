import asyncio
import struct
import time

import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

matrixL = np.zeros((15, 7))
matrixR = np.zeros((15, 7))
last_valid_length = None
last_valid_weight = None
last_valid_time = 0
status_message = "Wachten op meting..."
recent_lengths = []
client_ref = None

# Visualisatie setup
fig = plt.figure(figsize=(12, 6))
gs = fig.add_gridspec(2, 3, width_ratios=[1.5, 1.5, 1.2], height_ratios=[0.1, 0.9])

# Titelvelden
title_ax = fig.add_subplot(gs[0, :])
title_ax.axis("off")
weight_text = title_ax.text(0.17, 0.5, "", ha="center", va="center", fontsize=12)
length_text = title_ax.text(0.5, 0.5, "", ha="center", va="center", fontsize=12)
status_text = title_ax.text(0.83, 0.5, "", ha="center", va="center", fontsize=12)

# Drukverdeling
axL = fig.add_subplot(gs[1, 0])
imL = axL.imshow(matrixL, cmap="plasma", vmin=0, vmax=1024)
fig.colorbar(imL, ax=axL)
axL.set_title("Linkervoet")

axR = fig.add_subplot(gs[1, 1])
imR = axR.imshow(matrixR, cmap="plasma", vmin=0, vmax=1024)
fig.colorbar(imR, ax=axR)
axR.set_title("Rechtervoet")

# Lengtemeting log
log_ax = fig.add_subplot(gs[1, 2])
log_ax.axis("off")
log_title = log_ax.text(
    0.5, 1.03, "Laatste Lengtemetingen", ha="center", va="bottom", fontsize=11
)
log_texts = [
    log_ax.text(0.1, 0.9 - i * 0.09, "", fontsize=10, ha="left") for i in range(10)
]

fig.tight_layout()


def parse_matrix(value):
    foot = chr(value[0])
    values = [(value[1 + i * 2] << 8) | value[1 + i * 2 + 1] for i in range(105)]
    matrix = np.array(values).reshape(15, 7)
    return foot, matrix


def parse_result(value):
    if value[0] != 0x57 or len(value) != 10:
        return None
    gewicht = struct.unpack("<f", value[1:5])[0]
    lengte = struct.unpack("<f", value[5:9])[0]
    status = value[9]
    return gewicht, lengte, status


def update_visual():
    imL.set_data(matrixL)
    imR.set_data(matrixR)

    seconds_ago = int(time.time() - last_valid_time) if last_valid_time else "?"
    lengte_str = (
        f"{last_valid_length:.1f} cm" if last_valid_length not in [None, -1] else "?"
    )
    gewicht_str = (
        f"{last_valid_weight:.1f} kg" if last_valid_weight is not None else "?"
    )

    weight_text.set_text(f"Gewicht: {gewicht_str}")
    length_text.set_text(f"Lengte: {lengte_str} ({seconds_ago}s geleden)")
    status_text.set_text(status_message)

    for i, t in enumerate(recent_lengths):
        log_texts[i].set_text(f"{i+1:>2}. {t:.1f} cm")
    for i in range(len(recent_lengths), 10):
        log_texts[i].set_text("")

    fig.canvas.draw_idle()
    plt.pause(0.001)


def handle_notification_factory(client):
    async def handler(sender, value):
        global matrixL, matrixR, last_valid_length, last_valid_weight, last_valid_time, status_message, recent_lengths
        if value[0] == ord("L") or value[0] == ord("R"):
            foot, matrix = parse_matrix(value)
            if foot == "L":
                matrixL = matrix
            else:
                matrixR = matrix
            await client.write_gatt_char(CHARACTERISTIC_RX, b"APPROVED")
            status_message = "Bezig met meting..."
            update_visual()

        elif value[0] == 0x57:
            result = parse_result(value)
            if result:
                gewicht, lengte, status = result
                last_valid_weight = gewicht

                if lengte == -1:
                    status_message = "Geen geldige lengtemeting"
                elif status == 0:
                    if last_valid_length != lengte:
                        last_valid_length = lengte
                        last_valid_time = time.time()
                        recent_lengths.insert(0, lengte)
                        recent_lengths[:] = recent_lengths[:10]
                        status_message = "✅ Nieuwe meting goedgekeurd"
                    else:
                        status_message = "Lengte ongewijzigd"
                else:
                    status_message = "❌ Lengtemeting afgekeurd"

                update_visual()
                await asyncio.sleep(0.5)
                await client.write_gatt_char(CHARACTERISTIC_RX, b"START")

    return handler


async def main():
    global client_ref
    print("🔍 Zoeken naar MeasureMates...")
    devices = await BleakScanner.discover()
    target = next((d for d in devices if d.name and "MeasureMates" in d.name), None)
    if not target:
        print("❌ Geen ESP32 gevonden.")
        return

    print(f"✅ Verbinden met {target.name}")
    async with BleakClient(target) as client:
        client_ref = client
        await client.start_notify(
            CHARACTERISTIC_TX, handle_notification_factory(client)
        )

        for cmd in ["CONNECTED", "START"]:
            print(f"➡️  Stuur: {cmd}")
            await client.write_gatt_char(CHARACTERISTIC_RX, cmd.encode())
            await asyncio.sleep(0.5)

        print("📡 Ontvangen en visualiseren...")
        while True:
            await asyncio.sleep(1)


# Start
asyncio.run(main())
