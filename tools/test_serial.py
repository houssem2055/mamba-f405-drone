#!/usr/bin/env python3
"""
Pro Diagnostic Serial Telemetry Monitor & CSV Blackbox Logger for MAMBA F405 Drone.
Parses 13-field telemetry frames:
  $TLM,roll,pitch,yaw,gx,gy,gz,m1,m2,m3,m4,vbat,armed,mode
and logs all frames to flight_log.csv.
"""

import csv
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial is not installed. Install with: pip install pyserial")
    sys.exit(1)

MODES = {0: "ACRO", 1: "ANGLE", 2: "HORIZON"}
AI_FAULTS = {0: "NORMAL", 1: "PROP_VIB", 2: "MOTOR_LOSS", 3: "COLLISION"}


def list_ports():
    ports = serial.tools.list_ports.comports()
    print("Detected COM Ports:")
    for p in ports:
        print(f"  {p.device}: {p.description}")
    return [p.device for p in ports]


def main():
    print("==========================================================")
    print("  MAMBA F405 Pro Telemetry Monitor & CSV Flight Logger    ")
    print("==========================================================")

    ports = list_ports()
    if not ports:
        print("[WARNING] No serial COM ports detected. Connect Mamba F405 via USB.")
        port = "COM3"
    else:
        port = ports[0]
        print(f"[INFO] Using detected port: {port}")

    baud = 115200
    log_file = "flight_log.csv"

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
        ser.dtr = True
        ser.rts = True
        ser.reset_input_buffer()
        ser.write(b"$CMD,PING\n")
        ser.write(b"$CMD,GET_PID\n")
        print(f"[SUCCESS] Connected to {port} @ {baud} bps. Logging to {log_file}...\n")

        with open(log_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "roll_deg", "pitch_deg", "yaw_deg",
                "gx_rads", "gy_rads", "gz_rads",
                "m1", "m2", "m3", "m4", "vbat", "armed", "mode"
            ])

            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line.startswith("$TLM,"):
                    parts = line.split(",")
                    if len(parts) >= 14:
                        roll, pitch, yaw = parts[1], parts[2], parts[3]
                        gx, gy, gz = parts[4], parts[5], parts[6]
                        m1, m2, m3, m4 = parts[7], parts[8], parts[9], parts[10]
                        vbat = parts[11]
                        armed = "ARMED" if parts[12] == "1" else "DISARM"
                        mode = MODES.get(int(parts[13]), parts[13])
                        writer.writerow([time.time(), roll, pitch, yaw, gx, gy, gz, m1, m2, m3, m4, vbat, parts[12], parts[13]])
                        print(
                            f"\r[{armed:<6} | {mode:<7}] R:{roll:>6}° P:{pitch:>6}° Y:{yaw:>6}° | "
                            f"M:[{m1},{m2},{m3},{m4}] | {vbat}V",
                            end="",
                            flush=True,
                        )
                elif line.startswith("$AI,"):
                    p = line.split(",")
                    if len(p) >= 8:
                        cls_name = AI_FAULTS.get(int(p[2]), p[2])
                        print(f"\n[AI]: Health={p[1]} | Fault={cls_name} | NN_du=({p[3]},{p[4]}) | Jitter={p[5]} | Asym={p[6]}")
                elif line:
                    print(f"\n[FC]: {line}")

    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped. Flight log saved to {log_file}.")
    except Exception as e:
        print(f"\n[ERROR]: {e}")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
