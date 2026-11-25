#!/usr/bin/env python3
"""
Diagnostic Serial Telemetry Monitor for Mamba F405 Drone.
Reads incoming telemetry frames ($TLM,roll,pitch,yaw,m1,m2,m3,m4,vbat) from USB UART.
"""

import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial is not installed. Install it with: pip install pyserial")
    sys.exit(1)


def list_ports():
    ports = serial.tools.list_ports.comports()
    print("Detected COM Ports:")
    for p in ports:
        print(f"  {p.device}: {p.description}")
    return [p.device for p in ports]


def main():
    print("==================================================")
    print("    MAMBA F405 Telemetry Test Monitor (Python)    ")
    print("==================================================")

    ports = list_ports()
    if not ports:
        print("[WARNING] No serial COM ports detected. Connect Mamba F405 via USB.")
        port = "COM3"
    else:
        port = ports[0]
        print(f"[INFO] Using detected port: {port}")

    baud = 115200

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        ser.reset_input_buffer()
        print(f"[SUCCESS] Connected to {port} @ {baud} bps. Listening for telemetry...\n")

        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("$TLM"):
                parts = line.split(",")
                if len(parts) >= 8:
                    roll, pitch, yaw = parts[1], parts[2], parts[3]
                    m1, m2, m3, m4 = parts[4], parts[5], parts[6], parts[7]
                    vbat = parts[8] if len(parts) > 8 else "N/A"
                    print(
                        f"\rRoll: {roll:>6}° | Pitch: {pitch:>6}° | Yaw: {yaw:>6}° | "
                        f"Motors: [{m1}, {m2}, {m3}, {m4}] | VBat: {vbat}V",
                        end="",
                        flush=True,
                    )
            elif line:
                print(f"[RAW]: {line}")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print(f"\n[ERROR]: {e}")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
