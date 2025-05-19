import asyncio

import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakClient, BleakScanner

# BLE instellingen
SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CHAR_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

rows, cols = 15, 7
frame_lines = []
weight = None
height = None

# Plot setup
plt.ion()
fig, axes = plt.subplots(1, 2, figsize=(8, 8))
im_objects = [None, None]
colorbar = None


# Verwerk 33 regels data tot matrix + waarden
def parse_frame(lines):
    global weight, height
    data = lines.copy()
    if not data[0].startswith("---START---"):
        return None
    try:
        weight = float(data[1].split(":")[1])
        height = float(data[2].split(":")[1]) if "NaN" not in data[2] else None
        matrix = np.zeros((2, rows, cols), dtype=int)
        for i in range(rows):
            matrix[0, i] = [int(x) for x in data[3 + i].split(",")]
            matrix[1, i] = [int(x) for x in data[3 + rows + i].split(",")]
        return matrix
    except:
        return None


# Werk de visualisatie bij
def update_plot(matrix):
    global im_objects, colorbar
    fig.suptitle(
        f"Weight: {weight:.1f} kg   Height: {height:.1f} cm"
        if height
        else f"Weight: {weight:.1f} kg"
    )

    for foot in range(2):
        ax = axes[foot]
        ax.set_title("Left" if foot == 0 else "Right")

        # Spiegel rechtervoet horizontaal
        data = matrix[foot]
        if foot == 1:
            data = np.fliplr(data)

        if im_objects[foot] is None:
            im_objects[foot] = ax.imshow(
                data,
                vmin=0,
                vmax=1024,
                cmap="hot",
                interpolation="nearest",
                origin="upper",
            )
        else:
            im_objects[foot].set_data(data)

        ax.axis("off")

    if colorbar is None:
        colorbar = fig.colorbar(im_objects[1], ax=axes[1], fraction=0.046, pad=0.04)
        colorbar.set_label("Druk (0–1024)")

    plt.pause(0.01)


# BLE-verwerking
async def run():
    print("🔍 Scanning for ESP32...")
    devices = await BleakScanner.discover(timeout=10.0)
    target = None
    for d in devices:
        if d.name and "ESP32-BLE-Drukmat" in d.name:
            target = d
            break
        elif d.name:
            print(f"🧭 Gevonden: {d.name} ({d.address})")

    if not target:
        print("❌ ESP32-BLE-Drukmat niet gevonden.")
        return

    print(f"✅ Verbonden met: {target.name} ({target.address})")
    async with BleakClient(target.address) as client:

        def handle_notify(_, data):
            global frame_lines
            line = data.decode("utf-8").strip()
            if line == "---START---":
                frame_lines = [line]
            else:
                frame_lines.append(line)
                if len(frame_lines) == 33:
                    matrix = parse_frame(frame_lines)
                    if matrix is not None:
                        update_plot(matrix)

        await client.start_notify(CHAR_TX_UUID, handle_notify)
        print("📡 BLE Notify gestart. Wachten op data...")

        while True:
            await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(run())
