"""
MAMBA F405 AI Drone - Ground Control Station (GCS) Dashboard
------------------------------------------------------------
Usage:
  python tools/gcs_dashboard.py            # Live GUI (connects to COM port or --sim)
  python tools/gcs_dashboard.py --sim      # Built-in flight & AI simulation mode
"""
import argparse
import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

AI_FAULT_NAMES = {
    0: ("NORMAL", "#2ecc71"),
    1: ("PROP VIBRATION", "#f39c12"),
    2: ("MOTOR LOSS", "#e74c3c"),
    3: ("COLLISION", "#c0392b"),
}


class GCSApp:
    def __init__(self, root: tk.Tk, sim_mode: bool = False):
        self.root = root
        self.root.title("MAMBA F405 AI - Ground Control Station")
        self.root.geometry("760x520")
        self.root.configure(bg="#1e1e2e")

        self.sim_mode = sim_mode
        self.ser = None
        self.running = True
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self._build_ui()
        self._refresh_ports()

        if self.sim_mode:
            self._start_sim()
        self.root.after(50, self._process_queue)

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")

        top = tk.Frame(self.root, bg="#181825", pady=8, padx=10)
        top.pack(fill=tk.X)

        tk.Label(top, text="Port:", fg="#cdd6f4", bg="#181825").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)

        tk.Button(top, text="Refresh", command=self._refresh_ports, bg="#313244", fg="#cdd6f4").pack(side=tk.LEFT, padx=2)
        self.conn_btn = tk.Button(top, text="Connect", command=self._toggle_connect, bg="#89b4fa", fg="#11111b", width=10)
        self.conn_btn.pack(side=tk.LEFT, padx=6)

        self.status_lbl = tk.Label(top, text="DISCONNECTED", fg="#f38ba8", bg="#181825", font=("Segoe UI", 10, "bold"))
        self.status_lbl.pack(side=tk.RIGHT, padx=6)

        # Telemetry & AI Grid
        grid = tk.Frame(self.root, bg="#1e1e2e", padx=14, pady=10)
        grid.pack(fill=tk.BOTH, expand=True)

        self.vars = {
            "roll": tk.StringVar(value="0.00 deg"),
            "pitch": tk.StringVar(value="0.00 deg"),
            "yaw": tk.StringVar(value="0.00 deg"),
            "vbat": tk.StringVar(value="0.00 V"),
            "armed": tk.StringVar(value="DISARMED"),
            "ai_health": tk.StringVar(value="100%"),
            "ai_fault": tk.StringVar(value="NORMAL"),
            "safety": tk.StringVar(value="OK"),
        }

        cards = [
            ("Roll", "roll", 0, 0),
            ("Pitch", "pitch", 0, 1),
            ("Yaw", "yaw", 0, 2),
            ("Battery (4S)", "vbat", 1, 0),
            ("Arm State", "armed", 1, 1),
            ("Safety Status", "safety", 1, 2),
            ("AI Health", "ai_health", 2, 0),
            ("AI Fault Class", "ai_fault", 2, 1),
        ]

        for title, key, r, c in cards:
            frame = tk.Frame(grid, bg="#313244", padx=12, pady=10, relief=tk.RIDGE, bd=1)
            frame.grid(row=r, column=c, padx=6, pady=6, sticky="nsew")
            grid.grid_columnconfigure(c, weight=1)
            tk.Label(frame, text=title, fg="#a6adc8", bg="#313244", font=("Segoe UI", 9)).pack(anchor="w")
            lbl = tk.Label(frame, textvariable=self.vars[key], fg="#cdd6f4", bg="#313244", font=("Consolas", 14, "bold"))
            lbl.pack(anchor="w", pady=(4, 0))

        # Command Buttons
        cmds = tk.Frame(self.root, bg="#181825", pady=10, padx=10)
        cmds.pack(fill=tk.X)

        btn_specs = [
            ("PING", "$CMD,PING", "#89b4fa"),
            ("AI ON", "$CMD,AI_ON", "#a6e3a1"),
            ("AI OFF", "$CMD,AI_OFF", "#fab387"),
            ("AI RESET", "$CMD,AI_RESET", "#f9e2af"),
            ("GET PID", "$CMD,GET_PID", "#cba6f7"),
            ("ENTER DFU", "$CMD,DFU", "#f38ba8"),
        ]
        for label, cmd, color in btn_specs:
            tk.Button(
                cmds, text=label, bg=color, fg="#11111b", font=("Segoe UI", 9, "bold"),
                command=lambda c=cmd: self._send_cmd(c), width=11
            ).pack(side=tk.LEFT, padx=4)

        # Log Box
        self.log_box = tk.Text(self.root, height=8, bg="#11111b", fg="#a6e3a1", font=("Consolas", 9))
        self.log_box.pack(fill=tk.BOTH, padx=14, pady=(0, 10))

    def _refresh_ports(self):
        if not HAS_SERIAL:
            self.port_combo["values"] = ["(pyserial not installed)"]
            self.port_combo.current(0)
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports if ports else ["No COM ports"]
        self.port_combo.current(0)

    def _toggle_connect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
            self.conn_btn.config(text="Connect")
            self.status_lbl.config(text="DISCONNECTED", fg="#f38ba8")
            return
        if not HAS_SERIAL:
            messagebox.showwarning("pyserial missing", "Install pyserial: pip install pyserial")
            return
        port = self.port_var.get()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.conn_btn.config(text="Disconnect")
            self.status_lbl.config(text=f"CONNECTED ({port})", fg="#a6e3a1")
            threading.Thread(target=self._serial_reader, daemon=True).start()
        except Exception as exc:
            messagebox.showerror("Connection Error", str(exc))

    def _serial_reader(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.rx_queue.put(line)
            except Exception:
                break

    def _send_cmd(self, cmd: str):
        self._log(f"TX: {cmd}")
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\r\n").encode("utf-8"))

    def _start_sim(self):
        self.status_lbl.config(text="SIMULATION MODE", fg="#f9e2af")
        def sim_loop():
            t = 0.0
            while self.running:
                roll = 4.5 * math.sin(t)
                pitch = 3.0 * math.cos(t * 0.8)
                yaw = (t * 15.0) % 360.0
                vbat = 15.6 - 0.02 * (t % 20.0)
                self.rx_queue.put(f"$TLM,{roll:.2f},{pitch:.2f},{yaw:.2f},0.01,-0.01,0.00,1450,1455,1448,1452,{vbat:.2f},1,1")
                self.rx_queue.put("$AI,0.960,0,0.0012,-0.0008,0.015,4.2,1")
                t += 0.1
                time.sleep(0.1)
        threading.Thread(target=sim_loop, daemon=True).start()

    def _process_queue(self):
        while not self.rx_queue.empty():
            line = self.rx_queue.get_nowait()
            self._parse_line(line)
        if self.running:
            self.root.after(50, self._process_queue)

    def _parse_line(self, line: str):
        parts = line.split(",")
        if parts[0] == "$TLM" and len(parts) >= 14:
            self.vars["roll"].set(f"{parts[1]} deg")
            self.vars["pitch"].set(f"{parts[2]} deg")
            self.vars["yaw"].set(f"{parts[3]} deg")
            vbat = float(parts[11])
            self.vars["vbat"].set(f"{vbat:.2f} V")
            self.vars["armed"].set("ARMED" if parts[12] == "1" else "DISARMED")
            if 0.5 < vbat < 13.2:
                self.vars["safety"].set("CRITICAL BATTERY")
            elif 0.5 < vbat < 14.0:
                self.vars["safety"].set("LOW BATTERY")
            else:
                self.vars["safety"].set("OK")
        elif parts[0] == "$AI" and len(parts) >= 8:
            health = float(parts[1]) * 100.0
            cls_id = int(parts[2])
            name, _ = AI_FAULT_NAMES.get(cls_id, ("UNKNOWN", "#cdd6f4"))
            self.vars["ai_health"].set(f"{health:.1f}%")
            self.vars["ai_fault"].set(name)
        else:
            self._log(f"RX: {line}")

    def _log(self, msg: str):
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)


def main():
    parser = argparse.ArgumentParser(description="MAMBA F405 AI GCS Dashboard")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode without hardware")
    args = parser.parse_args()

    root = tk.Tk()
    app = GCSApp(root, sim_mode=args.sim)
    root.protocol("WM_DELETE_WINDOW", lambda: (setattr(app, "running", False), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
