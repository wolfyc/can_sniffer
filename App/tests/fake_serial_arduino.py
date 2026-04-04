# fake_arduino_serial.py
import os
import pty
import time
import random

def make_frame(ts_ms, can_id, data):
    dlc = len(data)
    data_str = " ".join(f"{b:02X}" for b in data)
    return f"T:{ts_ms} ID:{can_id:X} DLC:{dlc} DATA:{data_str}\n"

def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    print(f"Fake Arduino serial available on: {slave_name}")
    print("Utilise ce port dans ton programme serial_to_csv.")
    print("Ctrl+C pour arrêter.\n")

    ts = 0

    try:
        while True:
            ts += 100

            frames = [
                make_frame(ts, 0x1A0, [0x01, 0x02, 0x03, 0x04]),
                make_frame(ts + 10, 0x200, [0xAA, 0xBB]),
                make_frame(ts + 20, 0x7FF, []),
            ]

            # De temps en temps, injecter une ligne invalide
            if random.random() < 0.2:
                frames.append("garbage data %%%\n")

            for frame in frames:
                os.write(master_fd, frame.encode("utf-8"))
                print("Sent:", frame.strip())
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nArrêt du simulateur.")

if __name__ == "__main__":
    main()