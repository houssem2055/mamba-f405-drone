"""
MAMBA F405 AI Drone - Pro Aerospace Web Ground Control Station (GCS)
--------------------------------------------------------------------
Zero-dependency local web server (built-in http.server).
Usage:
  python tools/web_gcs.py --sim            # Start on http://127.0.0.1:8080 (Simulation Mode)
  python tools/web_gcs.py --port COM3      # Connect to live drone on COM3 (115200 baud)
"""
import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

STATE = {
    "mode": "SIMULATION",
    "connected": False,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "vbat": 15.8,
    "armed": False,
    "safety": "OK",
    "ai_health": 98.0,
    "ai_fault": "NORMAL",
    "ai_enabled": True,
    "rth_state": "IDLE",
    "logs": ["[SYS] Pro Aerospace GCS initialized."],
}
STATE_LOCK = threading.Lock()
SER_PORT = None

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MAMBA F405 AI — Pro Aerospace GCS</title>
<style>
  :root {
    --bg: #0b0e14; --panel: rgba(20, 25, 36, 0.88); --border: rgba(99, 179, 237, 0.22);
    --accent: #38bdf8; --green: #22c55e; --amber: #f59e0b; --red: #ef4444; --text: #e2e8f0; --muted: #94a3b8;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 18px; font-family: 'Segoe UI', system-ui, sans-serif;
    background: radial-gradient(circle at top right, #172554 0%, var(--bg) 55%);
    color: var(--text); min-height: 100vh;
  }
  .topbar {
    display: flex; justify-content: space-between; align-items: center;
    background: var(--panel); border: 1px solid var(--border); border-radius: 14px;
    padding: 12px 20px; margin-bottom: 16px; backdrop-filter: blur(12px);
  }
  .brand { display: flex; align-items: center; gap: 12px; font-size: 18px; font-weight: 700; letter-spacing: 0.5px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--green); box-shadow: 0 0 10px var(--green); }
  .badge { padding: 6px 14px; border-radius: 999px; font-weight: 700; font-size: 12px; background: rgba(56,189,248,0.15); color: var(--accent); border: 1px solid var(--accent); }
  .layout { display: grid; grid-template-columns: 340px 1fr 320px; gap: 16px; }
  @media (max-width: 1100px) { .layout { grid-template-columns: 1fr; } }
  .panel {
    background: var(--panel); border: 1px solid var(--border); border-radius: 14px;
    padding: 16px; backdrop-filter: blur(12px);
  }
  .panel h3 { margin: 0 0 12px 0; font-size: 14px; color: var(--accent); text-transform: uppercase; letter-spacing: 1px; }
  .hud-wrap { display: flex; flex-direction: column; align-items: center; }
  svg#hud { width: 280px; height: 280px; border-radius: 50%; border: 2px solid var(--accent); background: #0f172a; box-shadow: 0 0 24px rgba(56,189,248,0.18); }
  .metrics { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 14px; width: 100%; }
  .metric { background: rgba(15,23,42,0.8); padding: 10px; border-radius: 10px; border: 1px solid rgba(148,163,184,0.15); text-align: center; }
  .metric .k { font-size: 11px; color: var(--muted); }
  .metric .v { font-size: 17px; font-weight: 700; font-family: Consolas, monospace; margin-top: 4px; }
  canvas#chart { width: 100%; height: 210px; background: #090d16; border-radius: 10px; border: 1px solid rgba(148,163,184,0.15); }
  .btn-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 12px; }
  button {
    padding: 10px; border: none; border-radius: 8px; font-weight: 700; cursor: pointer; font-size: 13px;
    transition: transform 0.1s, opacity 0.15s;
  }
  button:hover { opacity: 0.9; transform: translateY(-1px); }
  .b-blue { background: var(--accent); color: #082f49; }
  .b-green { background: var(--green); color: #052e16; }
  .b-amber { background: var(--amber); color: #451a03; }
  .b-red { background: var(--red); color: #fff; }
  .pid-row { display: flex; align-items: center; justify-content: space-between; gap: 8px; margin-bottom: 10px; font-size: 13px; }
  .pid-row input { width: 90px; background: #0f172a; border: 1px solid var(--border); color: var(--text); padding: 6px; border-radius: 6px; font-family: Consolas, monospace; }
  .log {
    background: #06090f; border: 1px solid rgba(148,163,184,0.15); border-radius: 10px;
    padding: 10px; height: 150px; overflow-y: auto; font-family: Consolas, monospace; font-size: 12px;
    color: #4ade80; direction: ltr; text-align: left; margin-top: 10px;
  }
</style>
</head>
<body>
  <div class="topbar">
    <div class="brand"><div class="dot"></div> MAMBA F405 AI — محطة التحكم الأرضية الاحترافية</div>
    <span id="statusBadge" class="badge">SIMULATION</span>
  </div>

  <div class="layout">
    <!-- Left: Artificial Horizon HUD -->
    <div class="panel hud-wrap">
      <h3>الأفق الاصطناعي (HUD)</h3>
      <svg id="hud" viewBox="-100 -100 200 200">
        <defs><clipPath id="clipCircle"><circle cx="0" cy="0" r="96"/></clipPath></defs>
        <g clip-path="url(#clipCircle)">
          <g id="horizonGroup">
            <rect x="-200" y="-200" width="400" height="200" fill="#0284c7"/>
            <rect x="-200" y="0" width="400" height="200" fill="#78350f"/>
            <line x1="-200" y1="0" x2="200" y2="0" stroke="#fff" stroke-width="2"/>
            <line x1="-30" y1="-25" x2="30" y2="-25" stroke="#e2e8f0" stroke-width="1"/>
            <line x1="-30" y1="25" x2="30" y2="25" stroke="#e2e8f0" stroke-width="1"/>
          </g>
        </g>
        <path d="M -28 0 L -8 0 L 0 8 L 8 0 L 28 0" fill="none" stroke="#facc15" stroke-width="3"/>
      </svg>
      <div class="metrics">
        <div class="metric"><div class="k">ROLL</div><div id="roll" class="v">0.0°</div></div>
        <div class="metric"><div class="k">PITCH</div><div id="pitch" class="v">0.0°</div></div>
        <div class="metric"><div class="k">YAW</div><div id="yaw" class="v">0.0°</div></div>
      </div>
    </div>

    <!-- Center: Live Chart + Controls -->
    <div class="panel">
      <h3>الرسم البياني الحي (Roll / Pitch)</h3>
      <canvas id="chart" width="600" height="210"></canvas>
      <div class="btn-grid">
        <button class="b-blue" onclick="sendCmd('$CMD,PING')">PING</button>
        <button class="b-green" onclick="sendCmd('$CMD,AI_ON')">تفعيل AI</button>
        <button class="b-amber" onclick="sendCmd('$CMD,AI_OFF')">إيقاف AI</button>
        <button class="b-blue" onclick="sendCmd('$CMD,AI_RESET')">إعادة ضبط AI</button>
        <button class="b-blue" onclick="sendCmd('$CMD,GET_PID')">قراءة PID</button>
        <button class="b-red" onclick="sendCmd('$CMD,DFU')">دخول DFU</button>
      </div>
      <div id="logBox" class="log"></div>
    </div>

    <!-- Right: AI, Safety & PID -->
    <div class="panel">
      <h3>الذكاء الاصطناعي والأمان</h3>
      <div class="metrics" style="grid-template-columns: 1fr 1fr; margin-bottom: 14px;">
        <div class="metric"><div class="k">البطارية (4S)</div><div id="vbat" class="v">15.80 V</div></div>
        <div class="metric"><div class="k">الأمان</div><div id="safety" class="v">OK</div></div>
        <div class="metric"><div class="k">صحة AI</div><div id="ai_health" class="v">98%</div></div>
        <div class="metric"><div class="k">تصنيف AI</div><div id="ai_fault" class="v">NORMAL</div></div>
      </div>
      <h3>ضبط PID السريع</h3>
      <div class="pid-row"><span>Kp Roll</span><input id="kp_p" type="number" step="0.005" value="0.130"><button class="b-blue" onclick="setPid('Kp_p','kp_p')">إرسال</button></div>
      <div class="pid-row"><span>Ki Roll</span><input id="ki_p" type="number" step="0.005" value="0.050"><button class="b-blue" onclick="setPid('Ki_p','ki_p')">إرسال</button></div>
      <div class="pid-row"><span>Kd Roll</span><input id="kd_p" type="number" step="0.0002" value="0.0014"><button class="b-blue" onclick="setPid('Kd_p','kd_p')">إرسال</button></div>
    </div>
  </div>

<script>
const rollHist = [], pitchHist = [];
function drawChart() {
  const c = document.getElementById('chart'), ctx = c.getContext('2d');
  ctx.clearRect(0, 0, c.width, c.height);
  ctx.strokeStyle = 'rgba(148,163,184,0.15)';
  ctx.beginPath(); ctx.moveTo(0, c.height/2); ctx.lineTo(c.width, c.height/2); ctx.stroke();
  const plot = (arr, color) => {
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    arr.forEach((v, i) => {
      const x = (i / 100) * c.width, y = c.height/2 - v * 4;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  };
  plot(rollHist, '#38bdf8');
  plot(pitchHist, '#f59e0b');
}
async function refresh() {
  const res = await fetch('/api/state');
  const d = await res.json();
  document.getElementById('statusBadge').textContent = d.mode;
  document.getElementById('roll').textContent = d.roll.toFixed(1) + '°';
  document.getElementById('pitch').textContent = d.pitch.toFixed(1) + '°';
  document.getElementById('yaw').textContent = d.yaw.toFixed(1) + '°';
  document.getElementById('vbat').textContent = d.vbat.toFixed(2) + ' V';
  document.getElementById('safety').textContent = d.safety;
  document.getElementById('ai_health').textContent = d.ai_health.toFixed(0) + '%';
  document.getElementById('ai_fault').textContent = d.ai_fault;
  document.getElementById('horizonGroup').setAttribute('transform', `rotate(${-d.roll}) translate(0, ${d.pitch * 2.2})`);
  rollHist.push(d.roll); pitchHist.push(d.pitch);
  if (rollHist.length > 100) { rollHist.shift(); pitchHist.shift(); }
  drawChart();
  document.getElementById('logBox').innerHTML = d.logs.slice(-12).join('<br>');
}
async function sendCmd(cmd) {
  await fetch('/api/cmd', { method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({cmd}) });
  refresh();
}
function setPid(param, inputId) {
  const val = document.getElementById(inputId).value;
  sendCmd(`$SET,${param},${val}`);
}
setInterval(refresh, 150);
refresh();
</script>
</body>
</html>
"""


class WebGCSHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        if self.path == "/api/state":
            with STATE_LOCK:
                payload = json.dumps(STATE).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(payload)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

    def do_POST(self):
        if self.path == "/api/cmd":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            cmd = body.get("cmd", "")
            with STATE_LOCK:
                STATE["logs"].append(f"TX: {cmd}")
                if cmd == "$CMD,AI_ON":
                    STATE["ai_enabled"] = True
                elif cmd == "$CMD,AI_OFF":
                    STATE["ai_enabled"] = False
            if SER_PORT and SER_PORT.is_open:
                SER_PORT.write((cmd + "\r\n").encode("utf-8"))
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')


def sim_worker():
    t = 0.0
    while True:
        with STATE_LOCK:
            STATE["roll"] = 8.5 * math.sin(t)
            STATE["pitch"] = 5.2 * math.cos(t * 0.8)
            STATE["yaw"] = (t * 18.0) % 360.0
            STATE["vbat"] = 15.8 - 0.01 * (t % 20.0)
            STATE["safety"] = "OK"
            STATE["ai_health"] = 98.0 if STATE["ai_enabled"] else 0.0
            STATE["ai_fault"] = "NORMAL"
        t += 0.12
        time.sleep(0.1)


def main():
    global SER_PORT
    parser = argparse.ArgumentParser(description="MAMBA F405 Pro Aerospace Web GCS")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode")
    parser.add_argument("--port", type=str, default="", help="Serial COM port (e.g. COM3)")
    parser.add_argument("--http-port", type=int, default=8080, help="Local web port (default 8080)")
    args = parser.parse_args()

    if args.port and HAS_SERIAL:
        SER_PORT = serial.Serial(args.port, 115200, timeout=0.1)
        STATE["mode"] = f"LIVE ({args.port})"
        STATE["connected"] = True
    else:
        threading.Thread(target=sim_worker, daemon=True).start()

    server = HTTPServer(("127.0.0.1", args.http_port), WebGCSHandler)
    print(f"MAMBA F405 Pro Aerospace Web GCS running at http://127.0.0.1:{args.http_port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
