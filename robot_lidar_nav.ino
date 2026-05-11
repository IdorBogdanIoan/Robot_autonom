#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ─── WiFi ────────────────────────────────────────────
const char *ssid     = "acasa2";
const char *password = "Acasa2026";

// ─── Giroscop ────────────────────────────────────────
MPU6050 mpu(Wire);
float currentAngle = 0.0;

// --- Motor Pins ---
const int STBY = 15;

const int PWMA = 23;
const int AIN1 = 18;  
const int AIN2 = 19;  

const int PWMB = 2;  
const int BIN1 = 5;   
const int BIN2 = 4;   

const int freq = 5000;       
const int resolution = 8;    

// Setări independente de viteză pentru stabilitate
const int SPEED_FWD = 127;
const int SPEED_TURN = 70;

// ─── Navigație Autonomă ──────────────────────────────
enum NavState { IDLE, TURNING, DRIVING, AVOIDING_TURN_OUT, AVOIDING_DRIVE };
NavState navState = IDLE;
bool isNavigating = false;

float targetAngle = 0.0;

// Variabile monitorizare LIDAR
volatile float minFront = 8000, minSide1 = 8000, minSide2 = 8000, minSideAll = 8000;
volatile float tempMinFront = 8000, tempMinSide1 = 8000, tempMinSide2 = 8000, tempMinSideAll = 8000;

volatile bool obstacleInFront = false; // Zona de pericol 165 - 195
volatile bool obstacleInSide = false;  // Zona de degajare 90 - 270
volatile bool avoidLeft = true;

// ─── Funcții Motoare ─────────────────────────────────
void forward() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, SPEED_FWD); ledcWrite(PWMB, SPEED_FWD);
}

void back() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, SPEED_FWD); ledcWrite(PWMB, SPEED_FWD);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); 
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  
  ledcWrite(PWMA, SPEED_TURN); ledcWrite(PWMB, SPEED_TURN);
}

void turnRight() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, SPEED_TURN); ledcWrite(PWMB, SPEED_TURN);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, 0); ledcWrite(PWMB, 0);
}

// ─── Logica de Control Navigație ─────────────────────
float getAngleDiff(float target, float current) {
  float diff = target - current;
  while (diff <= -180.0) diff += 360.0;
  while (diff > 180.0)  diff -= 360.0;
  return diff;
}
void updateNavigation() {
  if (!isNavigating) return;

  float diff;
  switch (navState) {
    case TURNING:
      if (obstacleInFront) {
        stopMotors();
        avoidLeft = (minSide1 > minSide2); 
        navState = AVOIDING_TURN_OUT;
        break;
      }
      diff = getAngleDiff(targetAngle, currentAngle);
      if (abs(diff) <= 5.0) { 
        stopMotors();
        navState = DRIVING;
      } else if (diff > 0) {
        turnLeft();
      } else {
        turnRight();
      }
      break;

    case DRIVING:
      if (obstacleInFront) {
        stopMotors();
        avoidLeft = (minSide1 > minSide2); 
        navState = AVOIDING_TURN_OUT;
      } else {
        diff = getAngleDiff(targetAngle, currentAngle);
        if (abs(diff) > 15.0) { 
          navState = TURNING;
        } else {
          forward();
        }
      }
      break;

    case AVOIDING_TURN_OUT:
      if (!obstacleInFront) { 
        stopMotors();
        navState = AVOIDING_DRIVE;
      } else {
        if (avoidLeft) turnLeft();
        else turnRight();
      }
      break;

    case AVOIDING_DRIVE:
      if (obstacleInFront) { 
        stopMotors();
        avoidLeft = (minSide1 > minSide2);
        navState = AVOIDING_TURN_OUT;
      } else if (!obstacleInSide) { 
        stopMotors();
        navState = TURNING; 
      } else {
        forward();
      }
      break;
      
    case IDLE:
    default:
      stopMotors();
      break;
  }
}

// ─── LIDAR Structs & Servere ─────────────────────────
typedef struct __attribute__((__packed__)) {
  uint8_t  sync_byte1;
  uint8_t  sync_byte2;
  uint8_t  type;
  uint8_t  sample_count;
  uint16_t angle_start;
  uint16_t angle_end;
  uint16_t checksum;
} lidar_scan_header_t;

#define RX_BUF_SIZE 512
typedef struct { uint8_t data[RX_BUF_SIZE]; uint16_t index; } lidar_rx_buffer_t;

lidar_rx_buffer_t lidar_rx_buffer[2];
uint8_t ping_pong = 0;
uint8_t  binBuffer[1600];
uint16_t binIndex = 0;

WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ─── WebSocket Router ────────────────────────────────
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String cmd = (char*)payload;
    if (cmd.startsWith("GOTO:")) {
      targetAngle = cmd.substring(5).toFloat();
      isNavigating = true;
      navState = TURNING;
      Serial.print("NAVIGARE SPRE: "); Serial.println(targetAngle);
    } else if (cmd == "STOP_NAV" || cmd == "STP") {
      isNavigating = false;
      navState = IDLE;
      stopMotors();
      Serial.println("NAVIGARE OPRITA.");
} else if (cmd == "FWD" && !isNavigating) { forward();
    } else if (cmd == "BCK" && !isNavigating) { back();
    } else if (cmd == "LFT" && !isNavigating) { turnLeft();
    } else if (cmd == "RGT" && !isNavigating) { turnRight();
    }
  }
}

// ─── HTML (în flash) ──────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=0">
<title>LIDAR NavMap</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #111; display: flex; flex-direction: column; align-items: center; font-family: monospace; color: #aaa; padding: 12px; touch-action: manipulation; }
  h2 { color: #00ff88; margin-bottom: 6px; letter-spacing: 2px; }
  #info { font-size: 12px; margin-bottom: 8px; display: flex; gap: 15px; }
  #info span { color: #fff; }
  .highlight { color: #00bbff !important; font-weight: bold; }
  canvas { background: #000; border: 1px solid #222; border-radius: 8px; max-width: 100%; cursor: crosshair; }
  #controls { margin-top: 10px; display: flex; gap: 12px; align-items: center; font-size: 12px; }
  
  #drive-controls { margin-top: 15px; display: flex; gap: 15px; flex-wrap: wrap; justify-content: center; }
  .btn { padding: 12px 20px; font-size: 14px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; user-select: none; }
  .btn-fwd { background: #00ff88; color: #000; }
  .btn-bck { background: #555; color: #fff; }
  .btn-stop { background: #ff3333; color: #fff; border: 2px solid #fff; }
  .btn-side { background: #0088ff; color: #fff; }
  .btn:active { opacity: 0.7; }
</style>
</head>
<body>
<h2>LIDAR AUTOPILOT</h2>
<div id="info">
  St: <span id="status">Conectare...</span> | Unghi: <span id="gyro-angle" class="highlight">0.0°</span>
</div>
<canvas id="c" width="600" height="600"></canvas>

<div id="drive-controls">
  <button id="btn-lft" class="btn btn-side">LEFT</button>
  <button id="btn-fwd" class="btn btn-fwd">FWD</button>
  <button id="btn-stop" class="btn btn-stop">🛑 STOP</button>
  <button id="btn-bck" class="btn btn-bck">BCK</button>
  <button id="btn-rgt" class="btn btn-side">RIGHT</button>
</div>

<div id="controls">
  <label>Zoom: <input type="range" id="zoom" min="1" max="10" value="4" step="0.5"> <span id="zoomVal">4×</span></label>
  <label><input type="checkbox" id="trail" checked> Trail</label>
  <label><input type="checkbox" id="grid" checked> Grid</label>
</div>

<script>
const canvas  = document.getElementById('c');
const ctx     = canvas.getContext('2d');
const CX = 300, CY = 300;
let SCALE     = 0.04;
let trailMode = true;
let showGrid  = true;
let robotAngle = 0.0;
let targetDrawAngle = null;

document.getElementById('zoom').addEventListener('input', function() { SCALE = this.value / 100; document.getElementById('zoomVal').textContent = this.value + '×'; });
document.getElementById('trail').addEventListener('change', function() { trailMode = this.checked; });
document.getElementById('grid').addEventListener('change', function() { showGrid = this.checked; });

// Click pentru navigare
canvas.addEventListener('click', function(e) {
  const rect = canvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  const dx = px - CX;
  const dy = CY - py; // Y matematic merge in sus
  
  let angle = Math.atan2(dy, dx) * 180 / Math.PI +180;
  if(angle < 0) angle += 360;
  
  targetDrawAngle = angle;
  sendCmd("GOTO:" + angle.toFixed(1));
});

function drawGrid() {
  if (!showGrid) return;
  ctx.save(); ctx.strokeStyle = '#1a1a1a'; ctx.lineWidth = 0.5;
  [1000, 2000, 3000, 4000].forEach(r => {
    ctx.beginPath(); ctx.arc(CX, CY, r * SCALE, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = '#333'; ctx.font = '10px monospace'; ctx.fillText(r/1000 + 'm', CX + r*SCALE + 2, CY);
  });
  ctx.strokeStyle = '#181818';
  for (let a = 0; a < 360; a += 45) {
    const rad = a * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(CX, CY); ctx.lineTo(CX + Math.cos(rad) * 4000 * SCALE, CY - Math.sin(rad) * 4000 * SCALE); ctx.stroke();
  }
  ctx.restore();
}

function drawOrigin() {
  ctx.save();
  ctx.translate(CX, CY);
  
  if (targetDrawAngle !== null) {
    ctx.save();
    ctx.rotate(-targetDrawAngle * Math.PI / 180);
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-1000, 0);
    ctx.strokeStyle = '#00bbff'; ctx.setLineDash([5, 5]); ctx.lineWidth = 2; ctx.stroke();
    ctx.restore();
  }

  ctx.rotate(-robotAngle * Math.PI / 180);
  ctx.beginPath(); ctx.arc(0, 0, 8, 0, 2 * Math.PI); ctx.fillStyle = '#00ff88'; ctx.fill();
  
  // SĂGEATĂ REORIENTATĂ EXACT INVERS
  ctx.beginPath(); 
  ctx.moveTo(0, -6); 
  ctx.lineTo(-16, 0); // Schimbat la pozitiv
  ctx.lineTo(0, 6); 
  ctx.fillStyle = '#ff3333'; ctx.fill();
  
  ctx.restore();
}

let ws, pointCount = 0;
function sendCmd(cmd) { if(ws && ws.readyState === WebSocket.OPEN) ws.send(cmd); }

const btnFwd = document.getElementById('btn-fwd');
const btnBck = document.getElementById('btn-bck');
const btnLft = document.getElementById('btn-lft'); // <-- NOU
const btnRgt = document.getElementById('btn-rgt'); // <-- NOU
const btnStop = document.getElementById('btn-stop');

btnFwd.addEventListener('mousedown', () => sendCmd('FWD')); btnFwd.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd('FWD'); });
btnBck.addEventListener('mousedown', () => sendCmd('BCK')); btnBck.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd('BCK'); });
btnLft.addEventListener('mousedown', () => sendCmd('LFT')); btnLft.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd('LFT'); }); // <-- NOU
btnRgt.addEventListener('mousedown', () => sendCmd('RGT')); btnRgt.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd('RGT'); }); // <-- NOU

['mouseup', 'mouseleave', 'touchend'].forEach(evt => {
  btnFwd.addEventListener(evt, () => sendCmd('STP'));
  btnBck.addEventListener(evt, () => sendCmd('STP'));
  btnLft.addEventListener(evt, () => sendCmd('STP')); // <-- NOU
  btnRgt.addEventListener(evt, () => sendCmd('STP')); // <-- NOU
});
btnStop.addEventListener('click', () => { targetDrawAngle = null; sendCmd('STOP_NAV'); });

function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { document.getElementById('status').textContent = '🟢 OK'; document.getElementById('status').style.color = '#00ff88'; };
  ws.onclose = () => { document.getElementById('status').textContent = '🔴 OFF'; document.getElementById('status').style.color = '#ff4444'; setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();

  ws.onmessage = e => {
    if (typeof e.data === 'string') {
      if (e.data.startsWith("ANG:")) {
        robotAngle = parseFloat(e.data.substring(4)); // Fără +180 ca să păstrăm giroscopul corect matematic
        document.getElementById('gyro-angle').textContent = robotAngle.toFixed(1) + '°';
        return;
      }
      if (e.data === "SCAN") {
        if (trailMode) { ctx.fillStyle = 'rgba(0,0,0,0.35)'; ctx.fillRect(0, 0, 600, 600); } 
        else { ctx.clearRect(0, 0, 600, 600); }
        drawGrid(); drawOrigin(); return;
      }
    }

    const view = new DataView(e.data);
    const nPts = Math.floor(e.data.byteLength / 4);

    for (let i = 0; i < nPts; i++) {
      const angle = view.getUint16(i * 4, true) / 10.0;
      const dist  = view.getUint16(i * 4 + 2, true);
      const rad = (angle - robotAngle) * Math.PI / 180;
      const px  = CX + Math.cos(rad) * dist * SCALE;
      const py  = CY - Math.sin(rad) * dist * SCALE;

      const extDist = dist * SCALE + 40;
      const ex = CX + Math.cos(rad) * extDist;
      const ey = CY - Math.sin(rad) * extDist;

      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey);
      ctx.strokeStyle = 'rgba(255, 0, 0, 0.2)'; ctx.lineWidth = 0.5; ctx.stroke();
      ctx.fillStyle = '#00ff88'; ctx.fillRect(px - 1, py - 1, 3, 3);
    }
  };
}
connect();
</script>
</body>
</html>
)rawliteral";

// ─── Prototipuri ──────────────────────────────────────
uint8_t ydlidar_receive(uint8_t ch);
void    processPacket(uint8_t *p_data);

// ─── Setup ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  Wire.begin(21, 22);
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true, true);

  pinMode(STBY, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, freq, resolution); ledcAttach(PWMB, freq, resolution);
  digitalWrite(STBY, HIGH); stopMotors();             

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  httpServer.on("/", []() { httpServer.send_P(200, "text/html", INDEX_HTML); });
  httpServer.begin();
  wsServer.onEvent(webSocketEvent); wsServer.begin();

  delay(500);
  Serial1.write(0xa5); Serial1.write(0x60);
}

// ─── Loop ────────────────────────────────────────────
void loop() {
  httpServer.handleClient();
  wsServer.loop();
  mpu.update();

  updateNavigation();

  while (Serial1.available()) {
    if (ydlidar_receive(Serial1.read())) {
      lidar_rx_buffer_t *buf = &lidar_rx_buffer[ping_pong ^ 1];
      processPacket(buf->data);
    }
  }
}

// ─── Procesare Pachet (Aici verificăm obstacolele!) ───
void processPacket(uint8_t *p_data) {
  lidar_scan_header_t *hdr = (lidar_scan_header_t *)p_data;

  if (hdr->type == 1) {
    if (binIndex > 0) { wsServer.broadcastBIN(binBuffer, binIndex); binIndex = 0; }
    currentAngle = mpu.getAngleZ();
    wsServer.broadcastTXT("ANG:" + String(currentAngle, 1));
    wsServer.broadcastTXT("SCAN");

    minFront = tempMinFront; 
    minSideAll = tempMinSideAll;
    minSide1 = tempMinSide1; 
    minSide2 = tempMinSide2;

    // Crescut la 25cm (250mm) pentru siguranță la frânare!
    obstacleInFront = (minFront < 300.0);   
    obstacleInSide = (minSideAll < 300.0);  // Spațiu lateral la ocolire: 30cm

    tempMinFront = 8000; tempMinSideAll = 8000; tempMinSide1 = 8000; tempMinSide2 = 8000;
    return;
  }

  int   lsn       = hdr->sample_count;
  float fsa       = hdr->angle_start >> 7;
  float lsa       = hdr->angle_end   >> 7;
  float angle_dif = (lsa - fsa < 0) ? (lsa - fsa + 360.0f) : (lsa - fsa);
  float da        = (lsn > 1) ? angle_dif / (lsn - 1) : 0.0f;

  uint8_t *p = &p_data[sizeof(lidar_scan_header_t)];

  for (int i = 0; i < lsn; i++) {
    int dist = (p[0] + p[1] * 256) / 4;
    p += 2;

    if (dist < 50 || dist > 8000) continue;

    float angle = fsa + da * i;
    if (angle >= 360.0f) angle -= 360.0f;

    // Laturi (90° -> 270°)
    if (angle >= 90.0 && angle <= 270.0) {
      if (dist < tempMinSideAll) tempMinSideAll = dist;
    }
    
    // PERICOL FRONTAL mărit (150° -> 210°)
    if (angle >= 150.0 && angle <= 210.0) {
      if (dist < tempMinFront) tempMinFront = dist;
    }
    
    // Decizie Stânga vs Dreapta
    if (angle >= 90.0 && angle < 150.0) {
      if (dist < tempMinSide1) tempMinSide1 = dist;
    } else if (angle > 210.0 && angle <= 270.0) {
      if (dist < tempMinSide2) tempMinSide2 = dist;
    }

    if (binIndex + 4 > sizeof(binBuffer)) break;
    uint16_t a = (uint16_t)(angle * 10.0f);
    uint16_t d = (uint16_t)dist;            
    binBuffer[binIndex++] = a & 0xFF; binBuffer[binIndex++] = (a >> 8) & 0xFF;
    binBuffer[binIndex++] = d & 0xFF; binBuffer[binIndex++] = (d >> 8) & 0xFF;
  }
}

// ─── Recepție LIDAR ───────────────────────────────────
uint8_t ydlidar_receive(uint8_t ch) {
  static uint8_t  ch_prev = 0, length_header = 0; static uint16_t length_data = 0;
  lidar_rx_buffer_t *buf = &lidar_rx_buffer[ping_pong];

  if (ch == 0x55 && ch_prev == 0xaa) {
    buf->data[0] = ch_prev; buf->index = 1; length_header = sizeof(lidar_scan_header_t); length_data = 0;
  }
  if (buf->index < RX_BUF_SIZE) buf->data[buf->index++] = ch;
  ch_prev = ch;

  if (length_header > 0) {
    if (--length_header == 0) {
      if (buf->data[0] == 0xaa) length_data = 2 * ((lidar_scan_header_t *)buf->data)->sample_count + 1;
    }
  }
  if (length_data > 0) {
    if (--length_data == 0) { ping_pong ^= 0x01; return 1; }
  }
  return 0;
}