import asyncio

import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def parse_matrix(value):
    foot = chr(value[0])
    values = []
    for i in range(105):
        high = value[1 + i * 2]
        low = value[1 + i * 2 + 1]
        val = (high << 8) | low
        values.append(val)
    matrix = np.array(values).reshape(15, 7)
    return foot, matrix


async def send_disapproved(client):
    try:
        await client.write_gatt_char(CHARACTERISTIC_RX, b"DISAPPROVED")
    except Exception as e:
        print(f"⚠️  Fout bij versturen DISAPPROVED: {e}")


# Visualisatie setup
fig, (axL, axR) = plt.subplots(1, 2, figsize=(10, 5))
plt.ion()

# Create initial matrix and plot once
init_matrix = np.zeros((15, 7))
imL = axL.imshow(init_matrix, cmap="plasma", vmin=0, vmax=1024)
cbL = fig.colorbar(imL, ax=axL)
axL.set_title("Drukverdeling (L)")

imR = axR.imshow(init_matrix, cmap="plasma", vmin=0, vmax=1024)
cbR = fig.colorbar(imR, ax=axR)
axR.set_title("Drukverdeling (R)")


def handle_notification_factory(client):
    def handle(sender, value):
        if len(value) != 212 or value[0] not in [ord("L"), ord("R")]:
            return

        foot, matrix = parse_matrix(value)
        print(f"\n📥 Matrix {foot} ontvangen:\n{matrix}\n")

        if foot == "L":
            imL.set_data(matrix)
        else:
            imR.set_data(matrix)

        fig.canvas.draw_idle()
        plt.pause(0.001)

        asyncio.create_task(send_disapproved(client))

    return handle


async def main():
    print("🔍 Zoeken naar MeasureMates...")
    devices = await BleakScanner.discover()
    target = next((d for d in devices if d.name and "MeasureMates" in d.name), None)
    if not target:
        print("❌ Device niet gevonden.")
        return

    print(f"✅ Verbinden met {target.name}")
    async with BleakClient(target) as client:
        await client.start_notify(
            CHARACTERISTIC_TX, handle_notification_factory(client)
        )

        for cmd in ["CONNECTED", "START"]:
            print(f"➡️   Stuur: {cmd}")
            await client.write_gatt_char(CHARACTERISTIC_RX, cmd.encode())
            await asyncio.sleep(0.5)

        print("📡 Matrix data ontvangen...")
        while True:
            await asyncio.sleep(1)


asyncio.run(main())
