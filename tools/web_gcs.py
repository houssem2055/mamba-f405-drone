"""
MAMBA F405 AI Drone - Betaflight-Style Pro Web Configurator & GCS
-----------------------------------------------------------------
Zero-dependency local web server (built-in http.server) with full drone control:
  - Setup Tab: Live HUD, Quad-X Attitude, Calibrate Gyro, DFU Bootloader
  - PID & AI Tab: Full 13-parameter Roll/Pitch/Yaw/Angle PID table + Read/Save + TinyML AI
  - Motors Tab: Betaflight-style Quad-X diagram, Safety Lock switch, Master + M1..M4 sliders (1000-2000 us)
  - CLI Tab: Full interactive command terminal

Usage:
  python tools/web_gcs.py --port COM9
  python tools/web_gcs.py --sim
"""
import argparse
import json
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

STATE = {
    "mode": "DISCONNECTED",
    "port": "",
    "connected": False,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "m1": 1000,
    "m2": 1000,
    "m3": 1000,
    "m4": 1000,
    "vbat": 0.0,
    "armed": False,
    "flight_mode": 1,
    "ai_health": 100.0,
    "ai_fault": "NORMAL",
    "ai_enabled": True,
    "pid": {
        "KP_ANGLE": 6.0,
        "KP_P": 0.130, "KI_P": 0.050, "KD_P": 0.0014, "KF_P": 0.0020,
        "KP_Q": 0.130, "KI_Q": 0.050, "KD_Q": 0.0014, "KF_Q": 0.0020,
        "KP_R": 0.500, "KI_R": 0.200, "KD_R": 0.0000, "KF_R": 0.0000,
    },
    "pid_seq": 0,
    "ports": [],
    "logs": ["[SYS] MAMBA F405 Betaflight-Style Web GCS Ready."],
}
STATE_LOCK = threading.Lock()
SER_PORT = None
SER_LOCK = threading.Lock()
TX_QUEUE = queue.Queue()
SIM_MODE = False

FAULT_NAMES = {0: "NORMAL", 1: "PROP_DMG", 2: "MOTOR_LOSS", 3: "COLLISION"}

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>MAMBA F405 Configurator — Web GCS</title>
<style>
  :root {
    --bg: #11141a; --sidebar: #181c24; --panel: #1e232d; --card: #252b37;
    --border: #323a4a; --accent: #ffbb00; --blue: #38bdf8; --green: #22c55e;
    --red: #ef4444; --text: #e2e8f0; --muted: #94a3b8;
  }
  * { box-sizing: border-box; }
  body { margin: 0; font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); color: var(--text); height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
  /* Top Header Bar (Betaflight style) */
  .header {
    height: 58px; background: #141820; border-bottom: 2px solid var(--accent);
    display: flex; align-items: center; justify-content: space-between; padding: 0 18px; flex-shrink: 0;
  }
  .logo { display: flex; align-items: center; gap: 10px; font-weight: 800; font-size: 17px; color: var(--accent); }
  .logo span { color: var(--text); font-weight: 600; font-size: 13px; }
  .conn-bar { display: flex; align-items: center; gap: 10px; }
  select, input[type=number], input[type=text] {
    background: #0f131a; border: 1px solid var(--border); color: var(--text);
    padding: 6px 10px; border-radius: 6px; font-family: Consolas, monospace; font-size: 13px;
  }
  button {
    padding: 8px 14px; border: none; border-radius: 6px; font-weight: 700; cursor: pointer;
    font-size: 12.5px; transition: opacity 0.15s, transform 0.1s;
  }
  button:hover { opacity: 0.9; }
  button:active { transform: scale(0.98); }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  .btn-accent { background: var(--accent); color: #111; }
  .btn-blue { background: var(--blue); color: #082f49; }
  .btn-green { background: var(--green); color: #052e16; }
  .btn-red { background: var(--red); color: #fff; }
  .btn-dark { background: #2d3544; color: var(--text); border: 1px solid var(--border); }
  .status-chips { display: flex; align-items: center; gap: 10px; font-size: 12px; font-family: Consolas, monospace; }
  .chip { background: var(--card); border: 1px solid var(--border); padding: 5px 10px; border-radius: 6px; }
  .chip.ok { border-color: var(--green); color: var(--green); }
  .chip.warn { border-color: var(--accent); color: var(--accent); }

  /* Main Workspace */
  .workspace { display: flex; flex: 1; overflow: hidden; }
  .sidebar {
    width: 210px; background: var(--sidebar); border-left: 1px solid var(--border);
    display: flex; flex-direction: column; padding: 12px 8px; gap: 6px; flex-shrink: 0;
  }
  .nav-btn {
    display: flex; align-items: center; gap: 10px; padding: 11px 14px; border-radius: 8px;
    background: transparent; color: var(--muted); text-align: right; font-size: 13.5px; font-weight: 600;
  }
  .nav-btn:hover { background: rgba(255,255,255,0.05); color: var(--text); }
  .nav-btn.active { background: rgba(255,187,0,0.14); color: var(--accent); border-right: 3px solid var(--accent); }

  .content { flex: 1; padding: 18px; overflow-y: auto; }
  .tab { display: none; }
  .tab.active { display: block; }

  .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 14px; }
  @media (max-width: 1000px) { .grid-2, .grid-3 { grid-template-columns: 1fr; } }

  .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 16px; margin-bottom: 16px; }
  .panel h3 { margin: 0 0 14px 0; font-size: 14px; color: var(--accent); border-bottom: 1px solid var(--border); padding-bottom: 8px; }

  /* HUD & Chart */
  .hud-box { display: flex; flex-direction: column; align-items: center; }
  svg#hud { width: 250px; height: 250px; border-radius: 50%; border: 3px solid var(--accent); background: #0b101b; }
  canvas#chart { width: 100%; height: 230px; background: #0b101b; border-radius: 8px; border: 1px solid var(--border); }

  /* Motors Tab */
  .safety-banner {
    background: rgba(239,68,68,0.12); border: 1px solid var(--red); border-radius: 8px;
    padding: 12px 16px; display: flex; align-items: center; justify-content: space-between; margin-bottom: 16px;
  }
  .switch-wrap { display: flex; align-items: center; gap: 10px; font-weight: 700; cursor: pointer; }
  .switch-wrap input { width: 18px; height: 18px; accent-color: var(--red); cursor: pointer; }
  .quad-diagram {
    position: relative; width: 260px; height: 240px; margin: 0 auto; background: #141820;
    border-radius: 12px; border: 1px solid var(--border); display: flex; align-items: center; justify-content: center;
  }
  .arm-x { position: absolute; width: 190px; height: 8px; background: #3b4559; border-radius: 4px; }
  .arm-1 { transform: rotate(45deg); }
  .arm-2 { transform: rotate(-45deg); }
  .motor-node {
    position: absolute; width: 66px; height: 66px; border-radius: 50%; background: var(--card);
    border: 2px solid var(--blue); display: flex; flex-direction: column; align-items: center; justify-content: center;
    font-size: 11px; font-weight: 700; z-index: 2;
  }
  .m-pos-4 { top: 18px; left: 22px; }   /* Front-Left CCW */
  .m-pos-2 { top: 18px; right: 22px; }  /* Front-Right CW */
  .m-pos-3 { bottom: 18px; left: 22px; }/* Rear-Left CW */
  .m-pos-1 { bottom: 18px; right: 22px; }/* Rear-Right CCW */
  .slider-row { display: grid; grid-template-columns: 75px 1fr 65px; align-items: center; gap: 12px; margin-bottom: 12px; }
  .slider-row input[type=range] { width: 100%; accent-color: var(--accent); cursor: pointer; }
  .bar-wrap { height: 10px; background: #0f131a; border-radius: 5px; overflow: hidden; margin-top: 4px; }
  .bar-fill { height: 100%; background: var(--green); width: 0%; transition: width 0.08s; }

  /* PID Table */
  table.pid-tbl { width: 100%; border-collapse: collapse; font-size: 13px; }
  table.pid-tbl th, table.pid-tbl td { padding: 8px 10px; border-bottom: 1px solid var(--border); text-align: center; }
  table.pid-tbl th { color: var(--muted); font-size: 12px; }
  table.pid-tbl input { width: 85px; text-align: center; }

  /* CLI Terminal */
  .cli-out {
    background: #070a0f; border: 1px solid var(--border); border-radius: 8px; padding: 12px;
    height: 360px; overflow-y: auto; font-family: Consolas, monospace; font-size: 12.5px;
    color: #4ade80; direction: ltr; text-align: left; margin-bottom: 10px;
  }
  .cli-in { display: flex; gap: 8px; direction: ltr; }
  .cli-in input { flex: 1; }
</style>
</head>
<body>
  <!-- TOP HEADER -->
  <div class="header">
    <div class="logo">⚡ MAMBA F405 MK2 <span>Betaflight-Style Web Configurator</span></div>
    <div class="status-chips">
      <div id="chipMode" class="chip ok">DISCONNECTED</div>
      <div id="chipVbat" class="chip">0.00 V</div>
      <div id="chipArm" class="chip">DISARMED</div>
      <div id="chipAI" class="chip">AI: 100%</div>
    </div>
    <div class="conn-bar">
      <select id="portSelect"></select>
      <button id="btnConnect" class="btn-accent" onclick="toggleConnect()">اتصال (Connect)</button>
      <button class="btn-red" onclick="emergencyStop()">🛑 STOP (S)</button>
    </div>
  </div>

  <!-- WORKSPACE -->
  <div class="workspace">
    <!-- SIDEBAR -->
    <div class="sidebar">
      <button class="nav-btn active" onclick="showTab('setup', this)">🧭 الإعدادات والأفق (Setup)</button>
      <button class="nav-btn" onclick="showTab('motors', this)">🚁 المحركات (Motors)</button>
      <button class="nav-btn" onclick="showTab('pid', this)">🎛️ ضبط PID والذكاء</button>
      <button class="nav-btn" onclick="showTab('cli', this)">💻 سطر الأوامر (CLI)</button>
      <div style="margin-top:auto; padding:10px; font-size:11px; color:var(--muted); border-top:1px solid var(--border);">
        <button class="btn-dark" style="width:100%; margin-bottom:6px;" onclick="sendCmd('$CMD,PING')">فحص الاتصال (PING)</button>
        <button class="btn-red" style="width:100%;" onclick="sendCmd('$CMD,BOOTLOADER')">دخول DFU</button>
      </div>
    </div>

    <!-- CONTENT -->
    <div class="content">
      <!-- TAB 1: SETUP -->
      <div id="tab-setup" class="tab active">
        <div class="grid-2" style="grid-template-columns: 1.35fr 1fr; gap:14px;">
          <div class="panel hud-box" style="padding:12px;">
            <div style="display:flex; justify-content:space-between; align-items:center; width:100%; margin-bottom:8px;">
              <h3 style="margin:0; border:none; padding:0;">محاكاة 3D مباشرة (Solid FPV 3D)</h3>
              <div style="display:flex; gap:6px;">
                <span id="linkBadge" class="chip ok" style="padding:2px 8px; font-size:11px;">LINKED</span>
                <span id="fpsBadge" class="chip" style="padding:2px 8px; font-size:11px;">60 FPS</span>
              </div>
            </div>
            <canvas id="drone3d" width="660" height="330" style="width:100%; height:315px; background:linear-gradient(180deg, #0d1526 0%, #070a10 100%); border-radius:10px; border:1px solid var(--accent);"></canvas>
            <div style="display:flex; justify-content:space-between; width:100%; margin-top:8px; font-size:11.5px; color:var(--muted);">
              <label><input id="invRoll" type="checkbox"> عكس Roll</label>
              <label><input id="invPitch" type="checkbox"> عكس Pitch</label>
              <label><input id="invYaw" type="checkbox"> عكس Yaw</label>
            </div>
            <div class="grid-3" style="width:100%; margin-top:8px; gap:8px;">
              <div class="chip" style="text-align:center; padding:6px;">ROLL <strong id="valRoll" style="color:var(--blue);">0.0°</strong></div>
              <div class="chip" style="text-align:center; padding:6px;">PITCH <strong id="valPitch" style="color:var(--accent);">0.0°</strong></div>
              <div class="chip" style="text-align:center; padding:6px;">YAW <strong id="valYaw" style="color:var(--green);">0.0°</strong></div>
            </div>
            <div style="display:flex; gap:8px; margin-top:8px; width:100%;">
              <button class="btn-blue" style="flex:1; padding:7px;" onclick="sendCmd('$CMD,CAL_GYRO')">معايرة الجيروسكوب</button>
              <button class="btn-dark" style="flex:1; padding:7px;" onclick="reset3dYaw()">تصفير اتجاه 3D</button>
              <button class="btn-dark" style="flex:1; padding:7px;" onclick="sendCmd('$CMD,GET_PID')">قراءة PID</button>
            </div>
          </div>

          <div class="panel" style="padding:12px; display:flex; flex-direction:column; justify-content:space-between;">
            <h3>الأفق الاصطناعي والرسم البياني</h3>
            <div style="display:flex; align-items:center; gap:12px; margin-bottom:10px;">
              <svg id="hud" viewBox="-100 -100 200 200" style="width:120px; height:120px; flex-shrink:0;">
                <defs><clipPath id="clipC"><circle cx="0" cy="0" r="96"/></clipPath></defs>
                <g clip-path="url(#clipC)">
                  <g id="horizonGroup">
                    <rect x="-200" y="-200" width="400" height="200" fill="#0284c7"/>
                    <rect x="-200" y="0" width="400" height="200" fill="#78350f"/>
                    <line x1="-200" y1="0" x2="200" y2="0" stroke="#fff" stroke-width="2"/>
                  </g>
                </g>
                <path d="M -30 0 L -10 0 L 0 9 L 10 0 L 30 0" fill="none" stroke="#ffbb00" stroke-width="3"/>
              </svg>
              <div style="flex:1; display:flex; flex-direction:column; gap:6px;">
                <div class="chip">صحة TinyML AI: <strong id="aiHealthVal">100%</strong></div>
                <div class="chip">تصنيف الأعطال: <strong id="aiFaultVal">NORMAL</strong></div>
                <div class="chip">المستشعر: <strong style="color:var(--green);">ICM-42688-P (1 kHz)</strong></div>
              </div>
            </div>
            <canvas id="chart" width="600" height="200" style="height:200px;"></canvas>
          </div>
        </div>
      </div>

      <!-- TAB 2: MOTORS (Betaflight Style) -->
      <div id="tab-motors" class="tab">
        <div class="safety-banner">
          <span>⚠️ تحذير السلامة: تأكد من <strong>نزع جميع المراوح الـ 4</strong> قبل تفعيل منزلقات المحركات!</span>
          <label class="switch-wrap">
            <input id="motorSafetyLock" type="checkbox" onchange="toggleMotorLock()">
            أفهم المخاطر (تفعيل التحكم)
          </label>
        </div>

        <div class="grid-2">
          <div class="panel">
            <h3>مخطط المحركات (Quad-X) والقيم الحية</h3>
            <div class="quad-diagram">
              <div class="arm-x arm-1"></div>
              <div class="arm-x arm-2"></div>
              <div class="motor-node m-pos-4">M4 (CCW)<span id="diag_m4" style="color:var(--accent);">1000</span></div>
              <div class="motor-node m-pos-2">M2 (CW)<span id="diag_m2" style="color:var(--accent);">1000</span></div>
              <div class="motor-node m-pos-3">M3 (CW)<span id="diag_m3" style="color:var(--accent);">1000</span></div>
              <div class="motor-node m-pos-1">M1 (CCW)<span id="diag_m1" style="color:var(--accent);">1000</span></div>
            </div>
            <div style="display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:16px;">
              <button class="btn-green" onclick="sendCmd('A')">تشغيل الكل + PID (A)</button>
              <button class="btn-blue" onclick="sendCmd('T')">اختبار بالتتابع (T)</button>
              <button class="btn-accent" onclick="sendCmd('C')">معايرة ESC (C)</button>
              <button class="btn-red" onclick="emergencyStop()">إيقاف فوري (S)</button>
            </div>
          </div>

          <div class="panel">
            <h3>التحكم بالسرعة (1000 - 2000 µs)</h3>
            <div class="slider-row">
              <strong>الكل (Master)</strong>
              <input id="sl_all" type="range" min="1000" max="2000" step="5" value="1000" disabled oninput="setAllMotors(this.value)">
              <span id="txt_all" style="font-family:Consolas; font-weight:700; color:var(--accent);">1000</span>
            </div>
            <hr style="border-color:var(--border); margin:12px 0;">
            <div class="slider-row">
              <span>M1 (Rear-R)</span>
              <div><input id="sl_m1" type="range" min="1000" max="2000" step="5" value="1000" disabled oninput="setIndivMotor()"><div class="bar-wrap"><div id="bar_m1" class="bar-fill"></div></div></div>
              <span id="txt_m1" style="font-family:Consolas;">1000</span>
            </div>
            <div class="slider-row">
              <span>M2 (Front-R)</span>
              <div><input id="sl_m2" type="range" min="1000" max="2000" step="5" value="1000" disabled oninput="setIndivMotor()"><div class="bar-wrap"><div id="bar_m2" class="bar-fill"></div></div></div>
              <span id="txt_m2" style="font-family:Consolas;">1000</span>
            </div>
            <div class="slider-row">
              <span>M3 (Rear-L)</span>
              <div><input id="sl_m3" type="range" min="1000" max="2000" step="5" value="1000" disabled oninput="setIndivMotor()"><div class="bar-wrap"><div id="bar_m3" class="bar-fill"></div></div></div>
              <span id="txt_m3" style="font-family:Consolas;">1000</span>
            </div>
            <div class="slider-row">
              <span>M4 (Front-L)</span>
              <div><input id="sl_m4" type="range" min="1000" max="2000" step="5" value="1000" disabled oninput="setIndivMotor()"><div class="bar-wrap"><div id="bar_m4" class="bar-fill"></div></div></div>
              <span id="txt_m4" style="font-family:Consolas;">1000</span>
            </div>
          </div>
        </div>
      </div>

      <!-- TAB 3: PID & AI -->
      <div id="tab-pid" class="tab">
        <div class="panel">
          <h3>جدول معاملات الـ PID الكامل (Roll / Pitch / Yaw / Angle)</h3>
          <table class="pid-tbl">
            <thead>
              <tr><th>المحور</th><th>Proportional (P)</th><th>Integral (I)</th><th>Derivative (D)</th><th>Feedforward (F)</th></tr>
            </thead>
            <tbody>
              <tr>
                <td><strong>Roll</strong></td>
                <td><input id="KP_P" type="number" step="0.005" value="0.130"></td>
                <td><input id="KI_P" type="number" step="0.005" value="0.050"></td>
                <td><input id="KD_P" type="number" step="0.0002" value="0.0014"></td>
                <td><input id="KF_P" type="number" step="0.0005" value="0.0020"></td>
              </tr>
              <tr>
                <td><strong>Pitch</strong></td>
                <td><input id="KP_Q" type="number" step="0.005" value="0.130"></td>
                <td><input id="KI_Q" type="number" step="0.005" value="0.050"></td>
                <td><input id="KD_Q" type="number" step="0.0002" value="0.0014"></td>
                <td><input id="KF_Q" type="number" step="0.0005" value="0.0020"></td>
              </tr>
              <tr>
                <td><strong>Yaw</strong></td>
                <td><input id="KP_R" type="number" step="0.01" value="0.500"></td>
                <td><input id="KI_R" type="number" step="0.01" value="0.200"></td>
                <td><input id="KD_R" type="number" step="0.0005" value="0.0000"></td>
                <td><input id="KF_R" type="number" step="0.0005" value="0.0000"></td>
              </tr>
              <tr>
                <td><strong>Angle Mode</strong></td>
                <td><input id="KP_ANGLE" type="number" step="0.2" value="6.000"></td>
                <td colspan="3" style="color:var(--muted);">Outer Attitude Loop Gain</td>
              </tr>
            </tbody>
          </table>
          <div style="display:flex; gap:10px; margin-top:14px;">
            <button class="btn-blue" onclick="sendCmd('$CMD,GET_PID')">قراءة من الدرون (Read PID)</button>
            <button class="btn-green" onclick="saveAllPids()">حفظ وإرسال كل القيم (Save PID)</button>
            <button class="btn-green" style="margin-right:auto;" onclick="sendCmd('$CMD,AI_ON')">تفعيل TinyML AI</button>
            <button class="btn-dark" onclick="sendCmd('$CMD,AI_OFF')">إيقاف AI</button>
            <button class="btn-dark" onclick="sendCmd('$CMD,AI_RESET')">تصفير AI</button>
          </div>
        </div>
      </div>

      <!-- TAB 4: CLI -->
      <div id="tab-cli" class="tab">
        <div class="panel">
          <h3>سطر الأوامر المباشر (CLI Console)</h3>
          <div id="cliOut" class="cli-out"></div>
          <div class="cli-in">
            <input id="cliInput" type="text" placeholder="Type command e.g. $CMD,PING or $CMD,GET_PID or $PWM,1150,1150,1150,1150" onkeydown="if(event.key==='Enter')sendCli()">
            <button class="btn-accent" onclick="sendCli()">Send</button>
          </div>
        </div>
      </div>
    </div>
  </div>

<script>
let lastPidSeq = -1;
const rollHist = [], pitchHist = [];
let pwmTimer = null;
let liveAtt = { roll: 0, pitch: 0, yaw: 0, m: [1000, 1000, 1000, 1000] };
let smoothAtt = { roll: 0, pitch: 0, yaw: 0 };
let yawOffset3d = 0;
let propAngle = 0;
let fpsFrames = 0, fpsLast = performance.now();

function reset3dYaw() {
  yawOffset3d = liveAtt.yaw;
}

function project3D(p, rollDeg, pitchDeg, yawDeg, w, h) {
  const r = rollDeg * Math.PI / 180;
  const pt = pitchDeg * Math.PI / 180;
  const yw = (yawDeg - yawOffset3d) * Math.PI / 180;
  let x = p[0], y = p[1], z = p[2];
  let x1 = x * Math.cos(r) - z * Math.sin(r), y1 = y, z1 = x * Math.sin(r) + z * Math.cos(r);
  let x2 = x1, y2 = y1 * Math.cos(pt) - z1 * Math.sin(pt), z2 = y1 * Math.sin(pt) + z1 * Math.cos(pt);
  let x3 = x2 * Math.cos(yw) - y2 * Math.sin(yw), y3 = x2 * Math.sin(yw) + y2 * Math.cos(yw), z3 = z2;
  const camTilt = 26 * Math.PI / 180;
  let yc = y3 * Math.cos(camTilt) - z3 * Math.sin(camTilt);
  let zc = y3 * Math.sin(camTilt) + z3 * Math.cos(camTilt);
  const fov = 380;
  const depth = fov / (fov + yc + 280);
  return [w / 2 + x3 * depth, h / 2 - zc * depth, depth, yc];
}

function draw3DBox(ctx, pr, cx, cy, cz, sx, sy, sz, colTop, colSide, colFront) {
  const hx = sx/2, hy = sy/2, hz = sz/2;
  const v = [
    [cx-hx, cy-hy, cz-hz], [cx+hx, cy-hy, cz-hz], [cx+hx, cy+hy, cz-hz], [cx-hx, cy+hy, cz-hz],
    [cx-hx, cy-hy, cz+hz], [cx+hx, cy-hy, cz+hz], [cx+hx, cy+hy, cz+hz], [cx-hx, cy+hy, cz+hz]
  ].map(pr);
  const faces = [
    { idx:[4,5,6,7], col: colTop },
    { idx:[0,1,2,3], col: '#090d16' },
    { idx:[0,1,5,4], col: colSide },
    { idx:[2,3,7,6], col: colFront },
    { idx:[1,2,6,5], col: colSide },
    { idx:[3,0,4,7], col: colSide }
  ];
  return faces.map(f => {
    const pts = f.idx.map(i => v[i]);
    const avgDepth = pts.reduce((s, p) => s + p[3], 0) / 4;
    return { pts, col: f.col, depth: avgDepth };
  });
}

function renderDrone3D(now) {
  const rSign = document.getElementById('invRoll')?.checked ? -1 : 1;
  const pSign = document.getElementById('invPitch')?.checked ? -1 : 1;
  const ySign = document.getElementById('invYaw')?.checked ? -1 : 1;

  const tR = liveAtt.roll * rSign;
  const tP = liveAtt.pitch * pSign;
  const tY = liveAtt.yaw * ySign;

  smoothAtt.roll  += (tR - smoothAtt.roll)  * 0.30;
  smoothAtt.pitch += (tP - smoothAtt.pitch) * 0.30;
  let dy = tY - smoothAtt.yaw;
  while (dy > 180) dy -= 360;
  while (dy < -180) dy += 360;
  smoothAtt.yaw += dy * 0.30;

  fpsFrames++;
  if (now - fpsLast >= 1000) {
    const b = document.getElementById('fpsBadge');
    if (b) b.textContent = fpsFrames + ' FPS';
    fpsFrames = 0;
    fpsLast = now;
  }

  const c = document.getElementById('drone3d');
  if (!c) { requestAnimationFrame(renderDrone3D); return; }
  const ctx = c.getContext('2d');
  const w = c.width, h = c.height;
  ctx.clearRect(0, 0, w, h);

  // Perspective Horizon Grid
  ctx.strokeStyle = 'rgba(56,189,248,0.12)';
  ctx.lineWidth = 1;
  for (let g = -200; g <= 200; g += 50) {
    const a1 = project3D([g, -200, -62], 0, 0, 0, w, h);
    const a2 = project3D([g,  200, -62], 0, 0, 0, w, h);
    ctx.beginPath(); ctx.moveTo(a1[0], a1[1]); ctx.lineTo(a2[0], a2[1]); ctx.stroke();
    const b1 = project3D([-200, g, -62], 0, 0, 0, w, h);
    const b2 = project3D([ 200, g, -62], 0, 0, 0, w, h);
    ctx.beginPath(); ctx.moveTo(b1[0], b1[1]); ctx.lineTo(b2[0], b2[1]); ctx.stroke();
  }

  const R = smoothAtt.roll, P = smoothAtt.pitch, Y = smoothAtt.yaw;
  const pr = p => project3D(p, R, P, Y, w, h);

  const armLen = 108;
  const motors = [
    { pos: [ armLen, -armLen, 4], name: 'M1', led: '#ef4444', dir: -1, pwm: liveAtt.m[0] },
    { pos: [ armLen,  armLen, 4], name: 'M2', led: '#22c55e', dir:  1, pwm: liveAtt.m[1] },
    { pos: [-armLen, -armLen, 4], name: 'M3', led: '#ef4444', dir:  1, pwm: liveAtt.m[2] },
    { pos: [-armLen,  armLen, 4], name: 'M4', led: '#22c55e', dir: -1, pwm: liveAtt.m[3] },
  ];

  // Soft ground shadow
  const shPts = motors.map(mt => project3D([mt.pos[0], mt.pos[1], -60], R, P, Y, w, h));
  ctx.fillStyle = 'rgba(0,0,0,0.45)';
  ctx.beginPath();
  ctx.moveTo(shPts[0][0], shPts[0][1]); ctx.lineTo(shPts[1][0], shPts[1][1]);
  ctx.lineTo(shPts[3][0], shPts[3][1]); ctx.lineTo(shPts[2][0], shPts[2][1]);
  ctx.closePath(); ctx.fill();

  // Solid 3D Arms
  const c0 = pr([0, 0, -2]);
  motors.forEach(mt => {
    const mp = pr([mt.pos[0], mt.pos[1], -2]);
    ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 12; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(c0[0], c0[1]); ctx.lineTo(mp[0], mp[1]); ctx.stroke();
    ctx.strokeStyle = '#334155'; ctx.lineWidth = 5;
    ctx.beginPath(); ctx.moveTo(c0[0], c0[1]); ctx.lineTo(mp[0], mp[1]); ctx.stroke();
  });

  // Collect 3D solid faces (Bottom Carbon Plate, FC Stack, Top Plate, LiPo Battery, FPV Camera Pod)
  let allFaces = [];
  allFaces.push(...draw3DBox(ctx, pr, 0, 0, -4, 46, 86, 6, '#1e293b', '#0f172a', '#334155'));    // Bottom Frame
  allFaces.push(...draw3DBox(ctx, pr, 0, 0, 4,  38, 44, 10, '#0284c7', '#0369a1', '#38bdf8'));   // Mamba F405 Stack
  allFaces.push(...draw3DBox(ctx, pr, 0, 0, 12, 44, 82, 5, '#334155', '#1e293b', '#475569'));    // Top Carbon Plate
  allFaces.push(...draw3DBox(ctx, pr, 0, -8, 22, 30, 56, 16, '#eab308', '#ca8a04', '#facc15'));  // 4S LiPo Battery
  allFaces.push(...draw3DBox(ctx, pr, 0, 36, 18, 22, 18, 16, '#ef4444', '#991b1b', '#f87171'));  // Front FPV Camera

  allFaces.sort((a, b) => b.depth - a.depth).forEach(f => {
    ctx.fillStyle = f.col;
    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    f.pts.forEach((pt, i) => i === 0 ? ctx.moveTo(pt[0], pt[1]) : ctx.lineTo(pt[0], pt[1]));
    ctx.closePath(); ctx.fill(); ctx.stroke();
  });

  // Front Direction Arrow
  const noseTip = pr([0, 76, 16]), noseL = pr([-10, 52, 16]), noseR = pr([10, 52, 16]);
  ctx.fillStyle = '#22c55e';
  ctx.beginPath(); ctx.moveTo(noseTip[0], noseTip[1]); ctx.lineTo(noseL[0], noseL[1]); ctx.lineTo(noseR[0], noseR[1]); ctx.closePath(); ctx.fill();

  // Solid 3D Motor Bells & Propellers
  propAngle += 0.24;
  motors.map(mt => ({ ...mt, proj: pr(mt.pos) }))
        .sort((a, b) => b.proj[3] - a.proj[3])
        .forEach(mt => {
          const [px, py, sc] = mt.proj;
          const propR = 54;
          const spin = propAngle * mt.dir * (1 + Math.max(0, (mt.pwm - 1000) / 110));
          // Motor Bell Cylinder
          ctx.fillStyle = '#475569';
          ctx.beginPath(); ctx.ellipse(px, py + 4*sc, 11*sc, 6*sc, 0, 0, Math.PI*2); ctx.fill();
          ctx.fillStyle = '#94a3b8';
          ctx.beginPath(); ctx.ellipse(px, py - 2*sc, 11*sc, 6*sc, 0, 0, Math.PI*2); ctx.fill();
          // LED Ring
          ctx.fillStyle = mt.led;
          ctx.beginPath(); ctx.arc(px, py + 5*sc, 3.5*sc, 0, Math.PI*2); ctx.fill();
          // Spinning Prop Blur Disc
          ctx.fillStyle = mt.pwm > 1030 ? 'rgba(34,197,94,0.16)' : 'rgba(148,163,184,0.08)';
          ctx.strokeStyle = mt.pwm > 1030 ? 'rgba(34,197,94,0.85)' : 'rgba(148,163,184,0.35)';
          ctx.lineWidth = 2;
          ctx.beginPath(); ctx.ellipse(px, py - 6*sc, propR*sc, propR*sc*0.40, 0, 0, Math.PI*2);
          ctx.fill(); ctx.stroke();
          // Spinning Blade
          const bx = Math.cos(spin) * propR, by = Math.sin(spin) * propR;
          const b1 = pr([mt.pos[0] - bx, mt.pos[1] - by, 10]);
          const b2 = pr([mt.pos[0] + bx, mt.pos[1] + by, 10]);
          ctx.strokeStyle = mt.led; ctx.lineWidth = 3;
          ctx.beginPath(); ctx.moveTo(b1[0], b1[1]); ctx.lineTo(b2[0], b2[1]); ctx.stroke();
          ctx.fillStyle = '#fff'; ctx.font = 'bold 11px Consolas';
          ctx.fillText(mt.name, px - 7, py + 22*sc);
        });

  requestAnimationFrame(renderDrone3D);
}
requestAnimationFrame(renderDrone3D);

function showTab(name, btn) {
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('tab-' + name).classList.add('active');
  if (btn) btn.classList.add('active');
}

function toggleMotorLock() {
  const en = document.getElementById('motorSafetyLock').checked;
  ['sl_all','sl_m1','sl_m2','sl_m3','sl_m4'].forEach(id => {
    document.getElementById(id).disabled = !en;
  });
  if (!en) emergencyStop();
}

function setAllMotors(val) {
  document.getElementById('txt_all').textContent = val;
  ['m1','m2','m3','m4'].forEach(m => {
    document.getElementById('sl_' + m).value = val;
    document.getElementById('txt_' + m).textContent = val;
  });
  if (pwmTimer) clearTimeout(pwmTimer);
  pwmTimer = setTimeout(() => sendCmd(`$PWM,${val},${val},${val},${val}`), 35);
}

function setIndivMotor() {
  const m1 = document.getElementById('sl_m1').value;
  const m2 = document.getElementById('sl_m2').value;
  const m3 = document.getElementById('sl_m3').value;
  const m4 = document.getElementById('sl_m4').value;
  document.getElementById('txt_m1').textContent = m1;
  document.getElementById('txt_m2').textContent = m2;
  document.getElementById('txt_m3').textContent = m3;
  document.getElementById('txt_m4').textContent = m4;
  if (pwmTimer) clearTimeout(pwmTimer);
  pwmTimer = setTimeout(() => sendCmd(`$PWM,${m1},${m2},${m3},${m4}`), 35);
}

function emergencyStop() {
  document.getElementById('sl_all').value = 1000;
  document.getElementById('txt_all').textContent = 1000;
  ['m1','m2','m3','m4'].forEach(m => {
    document.getElementById('sl_' + m).value = 1000;
    document.getElementById('txt_' + m).textContent = 1000;
  });
  sendCmd('S');
}

async function saveAllPids() {
  const keys = ['KP_ANGLE','KP_P','KI_P','KD_P','KF_P','KP_Q','KI_Q','KD_Q','KF_Q','KP_R','KI_R','KD_R','KF_R'];
  for (const k of keys) {
    const el = document.getElementById(k);
    if (el) await sendCmd(`$SET,${k},${el.value}`);
  }
}

function sendCli() {
  const inp = document.getElementById('cliInput');
  if (inp.value.trim()) {
    sendCmd(inp.value.trim());
    inp.value = '';
  }
}

async function toggleConnect() {
  const port = document.getElementById('portSelect').value;
  await fetch('/api/connect', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({port})
  });
  refresh();
}

async function sendCmd(cmd) {
  await fetch('/api/cmd', {
    method: 'POST', headers: {'Content-Type':'application/json'},
    body: JSON.stringify({cmd})
  });
}

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
  plot(pitchHist, '#ffbb00');
}

async function refresh() {
  try {
    const res = await fetch('/api/state');
    const d = await res.json();
    document.getElementById('chipMode').textContent = d.mode;
    const lb = document.getElementById('linkBadge');
    if (lb) {
      lb.textContent = d.connected ? ('LINKED: ' + d.port) : 'SIMULATION';
      lb.className = 'chip ' + (d.connected ? 'ok' : 'warn');
    }
    document.getElementById('btnConnect').textContent = d.connected ? 'قطع الاتصال' : 'اتصال (Connect)';
    document.getElementById('chipVbat').textContent = d.vbat.toFixed(2) + ' V';
    document.getElementById('chipArm').textContent = d.armed ? 'ARMED' : 'DISARMED';
    document.getElementById('chipArm').className = 'chip ' + (d.armed ? 'warn' : '');
    document.getElementById('chipAI').textContent = 'AI: ' + d.ai_health.toFixed(0) + '%';
    document.getElementById('aiHealthVal').textContent = d.ai_health.toFixed(0) + '%';
    document.getElementById('aiFaultVal').textContent = d.ai_fault;

    document.getElementById('valRoll').textContent = d.roll.toFixed(1) + '°';
    document.getElementById('valPitch').textContent = d.pitch.toFixed(1) + '°';
    document.getElementById('valYaw').textContent = d.yaw.toFixed(1) + '°';
    liveAtt.roll = d.roll;
    liveAtt.pitch = d.pitch;
    liveAtt.yaw = d.yaw;
    liveAtt.m = [d.m1, d.m2, d.m3, d.m4];
    document.getElementById('horizonGroup').setAttribute('transform', `rotate(${-d.roll}) translate(0, ${d.pitch * 2.2})`);

    [1,2,3,4].forEach(i => {
      const v = Math.round(d['m' + i]);
      document.getElementById('diag_m' + i).textContent = v;
      const pct = Math.max(0, Math.min(100, ((v - 1000) / 1000) * 100));
      document.getElementById('bar_m' + i).style.width = pct + '%';
    });

    if (d.pid_seq !== lastPidSeq) {
      lastPidSeq = d.pid_seq;
      for (const [k, v] of Object.entries(d.pid)) {
        const el = document.getElementById(k);
        if (el) el.value = v;
      }
    }

    const sel = document.getElementById('portSelect');
    const curPorts = Array.from(sel.options).map(o => o.value).join(',');
    const newPorts = d.ports.join(',');
    if (curPorts !== newPorts && d.ports.length > 0) {
      sel.innerHTML = '';
      d.ports.forEach(p => {
        const o = document.createElement('option');
        o.value = p; o.textContent = p;
        if (p === d.port) o.selected = true;
        sel.appendChild(o);
      });
    }

    rollHist.push(d.roll); pitchHist.push(d.pitch);
    if (rollHist.length > 100) { rollHist.shift(); pitchHist.shift(); }
    drawChart();

    const cli = document.getElementById('cliOut');
    cli.innerHTML = d.logs.slice(-30).join('<br>');
    cli.scrollTop = cli.scrollHeight;
  } catch (e) {}
}

setInterval(refresh, 45);
refresh();
</script>
</body>
</html>
"""


def list_available_ports():
    if not HAS_SERIAL:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


def open_serial_port(port_name):
    global SER_PORT
    with SER_LOCK:
        if SER_PORT and SER_PORT.is_open:
            SER_PORT.close()
        SER_PORT = serial.Serial(port_name, 115200, timeout=0.04, write_timeout=0.5)
        SER_PORT.dtr = True
        SER_PORT.rts = True
    TX_QUEUE.put("$CMD,PING")
    TX_QUEUE.put("$CMD,INFO")
    TX_QUEUE.put("$CMD,GET_PID")
    with STATE_LOCK:
        STATE["port"] = port_name
        STATE["connected"] = True
        STATE["mode"] = f"LIVE ({port_name})"
        STATE["logs"].append(f"[SYS] Connected to {port_name} @ 115200")


def close_serial_port():
    global SER_PORT
    with SER_LOCK:
        if SER_PORT and SER_PORT.is_open:
            SER_PORT.close()
        SER_PORT = None
    with STATE_LOCK:
        STATE["connected"] = False
        STATE["mode"] = "DISCONNECTED"
        STATE["logs"].append("[SYS] Disconnected.")


class WebGCSHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        if self.path == "/api/state":
            with STATE_LOCK:
                STATE["ports"] = list_available_ports()
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
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length).decode("utf-8")) if length > 0 else {}

        if self.path == "/api/connect":
            if STATE["connected"]:
                close_serial_port()
            else:
                port = body.get("port", "")
                if port and HAS_SERIAL:
                    try:
                        open_serial_port(port)
                    except Exception as e:
                        with STATE_LOCK:
                            STATE["logs"].append(f"[ERR] {e}")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
            return

        if self.path == "/api/cmd":
            cmd = body.get("cmd", "")
            with STATE_LOCK:
                STATE["logs"].append(f"TX: {cmd}")
                if len(STATE["logs"]) > 80:
                    STATE["logs"] = STATE["logs"][-50:]
                if cmd == "$CMD,AI_ON":
                    STATE["ai_enabled"] = True
                elif cmd == "$CMD,AI_OFF":
                    STATE["ai_enabled"] = False
                elif not STATE["connected"]:
                    if cmd.startswith("$PWM,"):
                        parts = [int(x) for x in cmd[5:].split(",")]
                        if len(parts) == 4:
                            STATE["m1"], STATE["m2"], STATE["m3"], STATE["m4"] = parts
                            STATE["armed"] = any(p > 1000 for p in parts)
                    elif cmd in ("S", "s"):
                        STATE["m1"] = STATE["m2"] = STATE["m3"] = STATE["m4"] = 1000
                        STATE["armed"] = False
            if cmd:
                TX_QUEUE.put(cmd)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')


def serial_reader_worker():
    global SER_PORT
    while True:
        try:
            line = ""
            with SER_LOCK:
                port = SER_PORT
                if port and port.is_open:
                    while not TX_QUEUE.empty():
                        tx = TX_QUEUE.get_nowait()
                        port.write((tx + "\n").encode("utf-8"))
                    line = port.readline().decode("utf-8", errors="replace").strip()
            if not port or not port.is_open:
                while not TX_QUEUE.empty():
                    TX_QUEUE.get_nowait()
                time.sleep(0.1)
                continue
            if not line:
                continue
            if line.startswith("$TLM,"):
                parts = line.split(",")
                if len(parts) >= 14:
                    with STATE_LOCK:
                        STATE["roll"] = float(parts[1])
                        STATE["pitch"] = float(parts[2])
                        STATE["yaw"] = float(parts[3])
                        STATE["m1"] = float(parts[7])
                        STATE["m2"] = float(parts[8])
                        STATE["m3"] = float(parts[9])
                        STATE["m4"] = float(parts[10])
                        STATE["vbat"] = float(parts[11])
                        STATE["armed"] = (parts[12] == "1")
                        STATE["flight_mode"] = int(parts[13])
                elif len(parts) >= 11:
                    with STATE_LOCK:
                        STATE["roll"] = float(parts[1])
                        STATE["pitch"] = float(parts[2])
                        STATE["yaw"] = float(parts[3])
                        STATE["m1"] = float(parts[4])
                        STATE["m2"] = float(parts[5])
                        STATE["m3"] = float(parts[6])
                        STATE["m4"] = float(parts[7])
                        STATE["vbat"] = float(parts[8])
                        STATE["armed"] = (parts[9] == "1")
            elif line.startswith("$AI,"):
                parts = line.split(",")
                if len(parts) >= 3:
                    with STATE_LOCK:
                        if parts[1]:
                            STATE["ai_health"] = float(parts[1]) * 100.0
                        if parts[2]:
                            STATE["ai_fault"] = FAULT_NAMES.get(int(parts[2]), parts[2])
                        if len(parts) >= 8 and parts[7]:
                            STATE["ai_enabled"] = (parts[7] == "1")
            elif line.startswith("$PID,"):
                parts = line.split(",")
                keys = ["KP_ANGLE","KP_P","KI_P","KD_P","KF_P","KP_Q","KI_Q","KD_Q","KF_Q","KP_R","KI_R","KD_R","KF_R"]
                if len(parts) >= 14:
                    with STATE_LOCK:
                        for idx, k in enumerate(keys, start=1):
                            if parts[idx]:
                                STATE["pid"][k] = float(parts[idx])
                        STATE["pid_seq"] += 1
                        STATE["logs"].append(f"RX: {line}")
            else:
                with STATE_LOCK:
                    STATE["logs"].append(f"RX: {line}")
                    if len(STATE["logs"]) > 80:
                        STATE["logs"] = STATE["logs"][-50:]
        except Exception as e:
            with SER_LOCK:
                if SER_PORT and SER_PORT.is_open:
                    try:
                        SER_PORT.close()
                    except Exception:
                        pass
                SER_PORT = None
            with STATE_LOCK:
                STATE["connected"] = False
                STATE["mode"] = "DISCONNECTED"
                STATE["logs"].append(f"[ERR] {e}")
            time.sleep(0.4)


def sim_worker():
    t = 0.0
    while True:
        with STATE_LOCK:
            if not STATE["connected"] and SIM_MODE:
                STATE["mode"] = "SIMULATION"
                STATE["roll"] = 8.5 * math.sin(t)
                STATE["pitch"] = 5.2 * math.cos(t * 0.8)
                STATE["yaw"] = (t * 18.0) % 360.0
                STATE["vbat"] = 15.8 - 0.01 * (t % 20.0)
                STATE["ai_health"] = 98.0 if STATE["ai_enabled"] else 0.0
                STATE["ai_fault"] = "NORMAL"
        t += 0.12
        time.sleep(0.1)


def main():
    global SIM_MODE
    parser = argparse.ArgumentParser(description="MAMBA F405 Betaflight-Style Web Configurator")
    parser.add_argument("--sim", action="store_true", help="Run in simulation mode")
    parser.add_argument("--port", type=str, default="", help="Serial COM port (e.g. COM9)")
    parser.add_argument("--http-port", type=int, default=8080, help="Local web port (default 8080)")
    args = parser.parse_args()

    SIM_MODE = args.sim or (not args.port)
    threading.Thread(target=serial_reader_worker, daemon=True).start()
    threading.Thread(target=sim_worker, daemon=True).start()

    if args.port and HAS_SERIAL:
        try:
            open_serial_port(args.port)
        except Exception as e:
            with STATE_LOCK:
                STATE["logs"].append(f"[ERR] Could not open {args.port}: {e}")

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), WebGCSHandler)
    print(f"MAMBA F405 Betaflight-Style Web GCS running at http://127.0.0.1:{args.http_port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
