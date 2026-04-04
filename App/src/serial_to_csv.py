import csv
import serial
import time
from datetime import datetime

def parse_arduino_csv_line(line: str):
    line = line.strip()

    if not line:
        return None

    # Ignore comments / heartbeat lines
    if line.startswith("#"):
        return None

    # Ignore firmware header
    if line == "ts_ms,id,ext,rtr,dlc,data":
        return None

    parts = line.split(",", maxsplit=5)
    if len(parts) != 6:
        return None

    ts_ms, can_id, ext, rtr, dlc, data = parts

    try:
        ts_ms = int(ts_ms)
        ext = int(ext)
        rtr = int(rtr)
        dlc = int(dlc)
    except ValueError:
        return None

    data = data.strip()
    data_bytes = data.split() if data else []

    if len(data_bytes) != dlc:
        return None

    return {
        "received_at": datetime.utcnow().isoformat(),
        "ts_ms": ts_ms,
        "id": can_id.upper(),
        "ext": ext,
        "rtr": rtr,
        "dlc": dlc,
        "data": " ".join(data_bytes),
    }

def main():
    port = "/dev/ttyACM0"
    baudrate = 115200
    output_csv = "can_log.csv"

    with serial.Serial(port, baudrate, timeout=1) as ser, \
         open(output_csv, "a", newline="") as f:

        time.sleep(2)  # allow Arduino reset after port open

        writer = csv.DictWriter(
            f,
            fieldnames=["received_at", "ts_ms", "id", "ext", "rtr", "dlc", "data"]
        )

        if f.tell() == 0:
            writer.writeheader()

        print(f"Listening on {port}...")

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                text = raw.decode("utf-8", errors="replace").strip()
                print("RX:", repr(text))

                frame = parse_arduino_csv_line(text)
                if frame is None:
                    continue

                writer.writerow(frame)
                f.flush()
                print("Logged:", frame)

        except KeyboardInterrupt:
            print("\nStopping cleanly.")

if __name__ == "__main__":
    main()