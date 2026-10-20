/*
  Sensor de Aproximação VL53L0X — com Factory Reset por botão em GPIO3 (RX)
  - Long press >= 5000 ms no pino GPIO3 inicia Factory Reset
  - Factory defaults:
      thresholdCm = 50
      presenceSeconds = 3
      holdSeconds = 10
      maxOnMinutes = 3
      blockMinutes = 2
  - AP defaults after reset: SSID = "SENSOR_<ultimos4hex do MAC>" (apSSID_config = ""), password = "12345678" (apPass_config = "")
  - Sensor params persistidos em EEPROM; AP config persistido em LittleFS
  - Read interval: 40 ms, REQUIRED_VALID_READS = 2
  - Relay active-low (LOW = ON)
  - Use GPIO3 (RX) como botão (INPUT_PULLUP). Tenha atenção se usar adaptador serial: pressionar o botão pode interferir no RX.
  - NÃO inicializar Serial se usar SDA = GPIO1
  - Acesso via DNS: além de 192.168.4.1, acessar via http://sensor.net (redirecionado via DNS Server)
  - "Tempo Acionado após Ausência" contabilizado quando presença acaba, contagem regressiva a partir daí; botão manual liga/desliga sem depender desse tempo
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>  // Biblioteca para servidor DNS
#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ---------- CONFIG ----------
const char* DEFAULT_AP_PREFIX = "SENSOR_";
const char* DEFAULT_AP_PASS = "12345678";
const char* AP_CONFIG_FILE = "/config_ap.json";

const int I2C_SDA_PIN = 1; // GPIO1 (TX) - NÃO inicialize Serial se usar este pino
const int I2C_SCL_PIN = 2; // GPIO2
const int GPIO_ACT = 0;    // GPIO0 -> saída do relé
const int GPIO_FACTORY = 3; // GPIO3 (RX) -> botão de factory reset
const int EEPROM_ADDR = 0; // espaço para 10 bytes

const uint16_t VL53_MAX_MM = 1250;
const float VL53_MAX_CM = VL53_MAX_MM / 10.0f;

// defaults sensor params (factory defaults requested)
const float FACTORY_THRESHOLD_CM = 50.0f;
const uint16_t FACTORY_PRESENCE_SEC = 3;
const uint16_t FACTORY_HOLD_SEC = 10;
const uint16_t FACTORY_MAXON_MIN = 3;
const uint16_t FACTORY_BLOCK_MIN = 2;

// debounce / leitura
const int REQUIRED_VALID_READS = 2; // responsivo
const int MAX_INVALID_READS = 5;
const unsigned long SENSOR_READ_INTERVAL_MS = 40; // ~25 Hz

// button factory reset timings
const unsigned long FACTORY_LONGPRESS_MS = 5000; // 5s
const unsigned long BUTTON_DEBOUNCE_MS = 40;

// Relay logic: true => LOW = ON
const bool RELAY_ACTIVE_LOW = true;

// ---------- GLOBALS ----------
ESP8266WebServer server(80);
DNSServer dnsServer;  // Instância do servidor DNS
VL53L0X lox;
bool loxReady = false;

float lastDistanceCm = -1.0f;     // valor filtrado (apenas atualizado após streak)
float lastRawDistanceCm = -1.0f;  // último raw válido lido

// persisted sensor params (EEPROM)
float thresholdCm = FACTORY_THRESHOLD_CM;
uint16_t presenceSeconds = FACTORY_PRESENCE_SEC;
uint16_t holdSeconds = FACTORY_HOLD_SEC;
uint16_t maxOnMinutes = FACTORY_MAXON_MIN;
uint16_t blockMinutes = FACTORY_BLOCK_MIN;

// AP config (stored & effective)
String apSSID_config = ""; // empty => use generated
String apPass_config = ""; // empty or <8 => use default
String apSSID_in_use = "";
String apPass_in_use = "";

// state machine
enum State { STATE_IDLE, STATE_PRESENCE_COUNTING, STATE_TRIGGERED, STATE_MANUAL, STATE_BLOCKED };
State currentState = STATE_IDLE;

// timestamps
unsigned long stateStartMs = 0;
unsigned long activatedStartMs = 0;
unsigned long holdEndMs = 0;
unsigned long blockedUntilMs = 0;

// continuous presence tracker
unsigned long presenceFirstMs = 0;

// debounce counters
int validStreak = 0;
int invalidStreak = 0;

// manual
bool manualActive = false;

// AP restart scheduling (to avoid breaking HTTP response)
bool pendingAPRestart = false;
unsigned long pendingAPRestartAt = 0;

// Factory button state tracking
bool factoryPressed = false;
unsigned long factoryPressStart = 0;
unsigned long lastButtonReadMs = 0;
bool factoryActionInProgress = false;
bool factoryResetDone = false; // evita repetir o reset enquanto o botão segue pressionado

// ---------- util actuator ----------
void actuatorWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(GPIO_ACT, on ? LOW : HIGH);
  else                digitalWrite(GPIO_ACT, on ? HIGH : LOW);
}

// ---------- AP helpers ----------
String macNoColonUpper() {
  String mac = WiFi.macAddress(); // e.g. "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  mac.toUpperCase();
  return mac;
}

String generateDefaultSSID() {
  String mac = macNoColonUpper();
  if (mac.length() >= 4) return String(DEFAULT_AP_PREFIX) + mac.substring(mac.length() - 4);
  return String(DEFAULT_AP_PREFIX) + "0000";
}

void loadAPConfigFromFS() {
  if (!LittleFS.begin()) { apSSID_config = ""; apPass_config = ""; return; }
  if (!LittleFS.exists(AP_CONFIG_FILE)) { apSSID_config = ""; apPass_config = ""; return; }
  File f = LittleFS.open(AP_CONFIG_FILE, "r");
  if (!f) { apSSID_config = ""; apPass_config = ""; return; }
  size_t sz = f.size();
  std::unique_ptr<char[]> buf(new char[sz + 1]);
  f.readBytes(buf.get(), sz);
  buf[sz] = '\0';
  f.close();
  JsonDocument doc;
  auto err = deserializeJson(doc, buf.get());
  if (err) { apSSID_config = ""; apPass_config = ""; return; }
  const char* s = doc["ssid"] | "";
  const char* p_val = doc["pass"] | "";
  apSSID_config = String(s);
  apPass_config = String(p_val);
}

void saveAPConfigToFS() {
  if (!LittleFS.begin()) return;
  JsonDocument doc;
  doc["ssid"] = apSSID_config;
  doc["pass"] = apPass_config;
  File f = LittleFS.open(AP_CONFIG_FILE, "w");
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}

void applyAPConfig() {
  String gen = generateDefaultSSID();
  apSSID_in_use = (apSSID_config.length() == 0) ? gen : apSSID_config;
  apPass_in_use = (apPass_config.length() < 8) ? String(DEFAULT_AP_PASS) : apPass_config;
  WiFi.softAPdisconnect(true);
  delay(60);
  WiFi.softAP(apSSID_in_use.c_str(), apPass_in_use.c_str());
  // Iniciar DNS Server para redirecionar "sensor.net" para o IP do AP
  dnsServer.stop();
  dnsServer.start(53, "sensor.net", WiFi.softAPIP());
}

// ---------- EEPROM helpers (sensor params) ----------
void saveSensorParamsToEEPROM() {
  int16_t thr10 = (int16_t)round(thresholdCm * 10.0f);
  uint16_t h = holdSeconds, ps = presenceSeconds, m = maxOnMinutes, b = blockMinutes;
  EEPROM.begin(512);
  EEPROM.write(EEPROM_ADDR + 0, thr10 & 0xFF);
  EEPROM.write(EEPROM_ADDR + 1, (thr10 >> 8) & 0xFF);
  EEPROM.write(EEPROM_ADDR + 2, h & 0xFF);
  EEPROM.write(EEPROM_ADDR + 3, (h >> 8) & 0xFF);
  EEPROM.write(EEPROM_ADDR + 4, ps & 0xFF);
  EEPROM.write(EEPROM_ADDR + 5, (ps >> 8) & 0xFF);
  EEPROM.write(EEPROM_ADDR + 6, m & 0xFF);
  EEPROM.write(EEPROM_ADDR + 7, (m >> 8) & 0xFF);
  EEPROM.write(EEPROM_ADDR + 8, b & 0xFF);
  EEPROM.write(EEPROM_ADDR + 9, (b >> 8) & 0xFF);
  EEPROM.commit();
  EEPROM.end();
}

void loadSensorParamsFromEEPROM() {
  EEPROM.begin(512);
  uint8_t b0 = EEPROM.read(EEPROM_ADDR + 0);
  uint8_t b1 = EEPROM.read(EEPROM_ADDR + 1);
  uint8_t b2 = EEPROM.read(EEPROM_ADDR + 2);
  uint8_t b3 = EEPROM.read(EEPROM_ADDR + 3);
  uint8_t b4 = EEPROM.read(EEPROM_ADDR + 4);
  uint8_t b5 = EEPROM.read(EEPROM_ADDR + 5);
  uint8_t b6 = EEPROM.read(EEPROM_ADDR + 6);
  uint8_t b7 = EEPROM.read(EEPROM_ADDR + 7);
  uint8_t b8 = EEPROM.read(EEPROM_ADDR + 8);
  uint8_t b9 = EEPROM.read(EEPROM_ADDR + 9);
  EEPROM.end();

  int16_t thr10 = (int16_t)((b1 << 8) | b0);
  uint16_t h = (uint16_t)((b3 << 8) | b2);
  uint16_t ps = (uint16_t)((b5 << 8) | b4);
  uint16_t m = (uint16_t)((b7 << 8) | b6);
  uint16_t bl = (uint16_t)((b9 << 8) | b8);

  thresholdCm = (thr10 <= 0 || thr10 > 3000) ? FACTORY_THRESHOLD_CM : ((float)thr10)/10.0f;
  holdSeconds = (h > 3600) ? FACTORY_HOLD_SEC : h;
  presenceSeconds = (ps > 3600) ? FACTORY_PRESENCE_SEC : ps;
  maxOnMinutes = (m > 24*60) ? FACTORY_MAXON_MIN : m;
  blockMinutes = (bl > 24*60) ? FACTORY_BLOCK_MIN : bl;
}

// ---------- Factory Reset ----------
void doFactoryReset() {
  if (factoryActionInProgress) return;
  factoryActionInProgress = true;

  // 1) Restore sensor parameters to requested factory values
  thresholdCm = FACTORY_THRESHOLD_CM;
  presenceSeconds = FACTORY_PRESENCE_SEC;
  holdSeconds = FACTORY_HOLD_SEC;
  maxOnMinutes = FACTORY_MAXON_MIN;
  blockMinutes = FACTORY_BLOCK_MIN;
  saveSensorParamsToEEPROM();

  // 2) Reset AP config to defaults: keep SSID generated by MAC and default pass
  apSSID_config = "";   // empty -> generated SENSOR_<last4mac>
  apPass_config = "";   // empty -> default "12345678"
  saveAPConfigToFS();
  applyAPConfig(); // apply immediately

  // 3) Reset runtime state
  manualActive = false;
  currentState = STATE_IDLE;
  activatedStartMs = 0;
  holdEndMs = 0;
  blockedUntilMs = 0;
  presenceFirstMs = 0;
  validStreak = 0;
  invalidStreak = 0;
  actuatorWrite(false);

  // Slight delay to stabilize
  delay(200);

  factoryActionInProgress = false;
}

// ---------- HTTP handlers and UI ----------
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Sensor de Aproximação</title>
<style>
:root{--bg:#071021;--card:#0b1220;--accent:#1fb6ff;--muted:#98a0ab;}
html,body{height:100%;margin:0;background:linear-gradient(180deg,#071021 0%, #0b1220 60%);color:#e6eef6;font-family:Arial,Helvetica,sans-serif;}
.container{max-width:980px;margin:18px auto;padding:0 14px}
h1{margin:0 0 10px 0;font-size:22px;color:var(--accent)}
.card{background:linear-gradient(180deg, rgba(255,255,255,0.02), rgba(255,255,255,0.01));padding:14px;border-radius:10px;margin-bottom:12px;}
.row{margin-top:8px}
.label{color:var(--muted);font-size:13px;display:block;margin-bottom:6px}
.input-field{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid rgba(255,255,255,0.06);background:#051123;color:#eaf6ff;height:34px}
.actions{margin-top:10px;text-align:center}
.btn{padding:8px 12px;border-radius:8px;background:var(--accent);color:#021018;border:0;cursor:pointer;margin-right:8px}
.btn-manual{display:inline-block;min-width:160px}
.btn-manual.pressed{background:#f04b4b;color:#fff;box-shadow:inset 0 2px 4px rgba(0,0,0,0.2)}
.small{font-size:12px;color:var(--muted);margin-top:8px}
.status-grid{display:block}
.status-line{display:flex;justify-content:space-between;padding:4px 0;color:#e6eef6}
.input-row{display:flex;align-items:center;gap:8px}
.eye-btn{background:transparent;border:0;color:var(--muted);cursor:pointer;padding:6px;border-radius:6px;height:34px;width:38px}
</style>
</head><body>
<div class="container">
  <h1>Sensor de Aproximação</h1>

  <div class="card">
    <div class="status-grid">
      <div class="status-line"><div>Distância:</div><div id="statusDistance">--</div></div>
      <div class="status-line"><div>Estado:</div><div id="statusState">--</div></div>
      <div class="status-line"><div>Tempo Acionado após Ausência:</div><div id="statusHold">--</div></div>
      <div class="status-line"><div>Permanencia para Acionar:</div><div id="statusPresence">-- / -- s</div></div>
      <div class="status-line"><div>Bloqueio restante:</div><div id="statusCooldown">--</div></div>
    </div>

    <div class="actions">
      <button id="manualBtn" class="btn btn-manual">Acionamento Manual</button>
    </div>
  </div>

  <div class="card">
    <div class="label">Configurações</div>

    <div class="row">
      <label class="label">Distância p/ Acionar (cm)</label>
      <input id="th" class="input-field" type="number" step="0.1" />
    </div>

    <div class="row">
      <label class="label">Permanência p/ Acionar (seg)</label>
      <input id="pres" class="input-field" type="number" step="0.1" min="0" />
    </div>

    <div class="row">
      <label class="label">Tempo Acionado após Ausência (Seg)</label>
      <input id="hold" class="input-field" type="number" step="1" min="0" />
    </div>

    <div class="row">
      <label class="label">Tempo Max. de Acionamento (Min.)</label>
      <input id="maxon" class="input-field" type="number" step="1" min="0" />
    </div>

    <div class="row">
      <label class="label">Tempo de Bloqueio (Min.)</label>
      <input id="block" class="input-field" type="number" step="1" min="0" />
    </div>

    <hr style="margin:12px 0; border-color: rgba(255,255,255,0.04)">

    <div class="row">
      <label class="label">SSID do AP (deixe vazio p/ gerado)</label>
      <input id="apssid" class="input-field" type="text" />
    </div>

    <div class="row">
      <label class="label">Senha do AP (mínimo 8 chars)</label>
      <div class="input-row">
        <input id="appass" class="input-field" type="password" />
        <button id="togglePass" class="eye-btn" type="button">👁️</button>
      </div>
    </div>

    <div class="row actions">
      <button id="saveBtn" class="btn">Salvar</button>
    </div>

    <div class="small">AP reinicia somente se SSID ou senha efetiva mudarem.</div>
  </div>
</div>

<script>
async function fetchStatus() {
  try {
    const r = await fetch('/status');
    if (!r.ok) return;
    const s = await r.json();

    // Distância filtrada
    const distEl = document.getElementById('statusDistance');
    if (s.distance !== null && typeof s.distance === 'number' && s.distance >= 0) distEl.innerText = s.distance.toFixed(2) + ' cm';
    else distEl.innerText = '--';

    // Estado
    const stateEl = document.getElementById('statusState');
    let state = 'Desligado';
    if (typeof s.blockRemaining === 'number' && s.blockRemaining > 0) state = 'Bloqueado';
    else if (s.triggered) state = 'Acionado';
    stateEl.innerText = state;

    // Tempo acionado após ausência
    document.getElementById('statusHold').innerText = (s.holdRemaining !== null && typeof s.holdRemaining === 'number') ? s.holdRemaining.toFixed(1) + ' s' : '--';

    // Permanencia para acionar
    const pres = (s.presenceElapsed !== null && typeof s.presenceElapsed === 'number') ? s.presenceElapsed.toFixed(1) : '--';
    const presReq = (s.presenceRequired !== null) ? s.presenceRequired : '--';
    document.getElementById('statusPresence').innerText = pres + ' / ' + presReq + ' s';

    // Bloqueio restante
    const br = document.getElementById('statusCooldown');
    if (typeof s.blockRemaining === 'number' && s.blockRemaining > 0) br.innerText = s.blockRemaining.toFixed(1) + ' s';
    else br.innerText = '--';

    // Manual button reflect firmware manualActive
    const mb = document.getElementById('manualBtn');
    if (s.manualActive) {
      mb.classList.add('pressed');
      mb.innerText = 'Parar Acionamento';
    } else {
      mb.classList.remove('pressed');
      mb.innerText = 'Acionamento Manual';
    }

  } catch (e) {
    // silencioso
  }
}

async function loadParams() {
  try {
    const r = await fetch('/getParams');
    if (!r.ok) return;
    const j = await r.json();
    document.getElementById('th').value = j.threshold;
    document.getElementById('pres').value = j.presenceSeconds;
    document.getElementById('hold').value = j.holdSeconds;
    document.getElementById('maxon').value = j.maxOnMinutes;
    document.getElementById('block').value = j.blockMinutes;
    document.getElementById('apssid').value = j.apSSID_in_use || '';
    document.getElementById('appass').value = j.apPass_in_use || '';
  } catch (e) {}
}

document.getElementById('saveBtn').addEventListener('click', async () => {
  const v = parseFloat(document.getElementById('th').value||'0');
  const p1 = parseFloat(document.getElementById('pres').value||'0');
  const h = parseInt(document.getElementById('hold').value||'0');
  const m = parseInt(document.getElementById('maxon').value||'0');
  const b = parseInt(document.getElementById('block').value||'0');
  const ssid = document.getElementById('apssid').value || '';
  const pass = document.getElementById('appass').value || '';
  if (isNaN(v)||isNaN(p1)||isNaN(h)||isNaN(m)||isNaN(b)) { alert('Valores inválidos'); return; }
  if (pass.length>0 && pass.length < 8) { alert('A senha do AP precisa ter pelo menos 8 caracteres ou ficar vazia para usar a padrão.'); return; }
  try {
    const payload = { threshold: v, presenceSeconds: p1, holdSeconds: h, maxOnMinutes: m, blockMinutes: b, apSSID: ssid, apPass: pass };
    const r = await fetch('/setParams', { method:'POST', body: JSON.stringify(payload), headers: {'Content-Type':'application/json'} });
    if (r.ok) {
      alert('Salvo. Se alterou o SSID/senha, o AP será reiniciado em instantes (apenas se necessário).');
      setTimeout(()=>{ try{ location.reload(); } catch(e){} }, 900);
    } else {
      const txt = await r.text();
      alert('Erro ao salvar: ' + txt);
    }
  } catch(e) { alert('Erro: ' + e); }
});

// Manual button logic: toggle start/stop and visual pressed while manualActive - não depende de holdSeconds para desligar
document.getElementById('manualBtn').addEventListener('click', async () => {
  const mb = document.getElementById('manualBtn');
  const pressed = mb.classList.contains('pressed');
  try {
    if (!pressed) {
      const r = await fetch('/manualStart', { method: 'POST' });
      const j = await r.json();
      if (r.ok && j.ok) {
        mb.classList.add('pressed');
        mb.innerText = 'Parar Acionamento';
      } else {
        alert('Falha ao iniciar (bloqueio ou erro).');
      }
    } else {
      const r = await fetch('/manualStop', { method: 'POST' });
      if (r.ok) {
        mb.classList.remove('pressed');
        mb.innerText = 'Acionamento Manual';
      } else {
        alert('Falha ao parar (bloqueio ou erro).');
      }
    }
  } catch(e) {
    alert('Erro manual: ' + e);
  }
});

// toggle password visibility
document.getElementById('togglePass').addEventListener('click', function(){
  const inp = document.getElementById('appass');
  if (inp.type === 'password') { inp.type = 'text'; this.innerText = '🙈'; }
  else { inp.type = 'password'; this.innerText = '👁️'; }
});

// polling a cada 350 ms
setInterval(fetchStatus, 350);
window.onload = () => { fetchStatus(); loadParams(); };
</script>
</body></html>
)rawliteral";

// ---------- HTTP handlers ----------
void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE_HTML);
}

void handleGetParams() {
  JsonDocument doc;
  doc["threshold"] = thresholdCm;
  doc["holdSeconds"] = holdSeconds;
  doc["presenceSeconds"] = presenceSeconds;
  doc["maxOnMinutes"] = maxOnMinutes;
  doc["blockMinutes"] = blockMinutes;
  // Return effective AP values for UI prefill
  doc["apSSID_in_use"] = apSSID_in_use;
  doc["apPass_in_use"] = apPass_in_use;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSetParams() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) { server.send(400, "application/json", "{\"ok\":false}"); return; }

  // sensor params (apply immediately and persist)
  float t = doc["threshold"] | thresholdCm;
  float ps = doc["presenceSeconds"] | presenceSeconds;  // Renomeado para ps para evitar conflito
  int h = doc["holdSeconds"] | holdSeconds;
  int m = doc["maxOnMinutes"] | maxOnMinutes;
  int b = doc["blockMinutes"] | blockMinutes;

  if (t < 0) t = 0;
  if (ps < 0) ps = 0;
  if (h < 0) h = 0;
  if (m < 0) m = 0;
  if (b < 0) b = 0;

  thresholdCm = t;
  presenceSeconds = (uint16_t)round(ps);
  holdSeconds = (uint16_t)h;
  maxOnMinutes = (uint16_t)m;
  blockMinutes = (uint16_t)b;
  saveSensorParamsToEEPROM();

  // AP params: check if effective AP values change
  const char* ssid = doc["apSSID"] | "";
  const char* p_val = doc["apPass"] | "";  // Renomeado para p_val para evitar conflito
  String new_apSSID_config = String(ssid);
  String new_apPass_config = String(p_val);

  // compute current and new effective AP
  String curSSID = apSSID_in_use;
  String curPass = apPass_in_use;
  String newEffSSID = (new_apSSID_config.length() == 0) ? generateDefaultSSID() : new_apSSID_config;
  String newEffPass = (new_apPass_config.length() < 8) ? String(DEFAULT_AP_PASS) : new_apPass_config;

  // update stored config and save
  apSSID_config = new_apSSID_config;
  apPass_config = new_apPass_config;
  saveAPConfigToFS();

  if (newEffSSID != curSSID || newEffPass != curPass) {
    JsonDocument resp; resp["ok"] = true;
    String out; serializeJson(resp, out);
    server.send(200, "application/json", out);
    // schedule AP restart outside handler
    pendingAPRestart = true;
    pendingAPRestartAt = millis() + 400;
  } else {
    apSSID_in_use = newEffSSID;
    apPass_in_use = newEffPass;
    JsonDocument resp; resp["ok"] = true;
    String out; serializeJson(resp, out);
    server.send(200, "application/json", out);
  }
}

void handleStatus() {
  JsonDocument doc;
  if (lastDistanceCm < 0.0f) { doc["distance"] = -1; doc["outOfRange"] = true; }
  else { doc["distance"] = lastDistanceCm; doc["outOfRange"] = false; }
  doc["lastRawDistance"] = lastRawDistanceCm;
  doc["triggered"] = (currentState == STATE_TRIGGERED || currentState == STATE_MANUAL);
  doc["manualActive"] = manualActive;
  if (!manualActive && ((currentState == STATE_TRIGGERED) && holdSeconds > 0 && holdEndMs > millis())) doc["holdRemaining"] = (holdEndMs - millis())/1000.0f;
  else doc["holdRemaining"] = nullptr;  // Para manual, não mostrar holdRemaining
  if (presenceFirstMs != 0) doc["presenceElapsed"] = (millis() - presenceFirstMs)/1000.0f;
  else doc["presenceElapsed"] = nullptr;
  doc["presenceRequired"] = presenceSeconds;
  if (blockedUntilMs > millis()) doc["blockRemaining"] = (blockedUntilMs - millis())/1000.0f;
  else doc["blockRemaining"] = nullptr;
  doc["apSSID_in_use"] = apSSID_in_use;
  doc["apPass_in_use"] = apPass_in_use;
  String out; serializeJson(doc, out);
  server.send(200,"application/json",out);
}

// Manual endpoints
void handleManualStart() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  unsigned long now = millis();
  if (blockedUntilMs > now) {
    JsonDocument resp; resp["ok"] = false; resp["error"] = "blocked";
    String out; serializeJson(resp, out);
    server.send(200, "application/json", out);
    return;
  }
  manualActive = true;
  currentState = STATE_MANUAL;
  stateStartMs = now;
  if (activatedStartMs == 0) activatedStartMs = now;
  actuatorWrite(true);
  holdEndMs = 0;  // Manual não usa holdEndMs
  JsonDocument resp; resp["ok"] = true; String out; serializeJson(resp, out);
  server.send(200, "application/json", out);
}

// Distance handler (unchanged)
void handleDistance() {
  JsonDocument doc;
  if (lastDistanceCm < 0.0f) { doc["distance"] = -1; doc["outOfRange"] = true; }
  else { doc["distance"] = lastDistanceCm; doc["outOfRange"] = false; }
  doc["lastRawDistance"] = lastRawDistanceCm;
  doc["activated"] = (currentState == STATE_TRIGGERED || currentState == STATE_MANUAL);
  if ((currentState == STATE_TRIGGERED || currentState == STATE_MANUAL) && holdSeconds > 0 && holdEndMs > millis()) doc["holdRemaining"] = (holdEndMs - millis())/1000.0f;
  if (blockedUntilMs > millis()) doc["blockRemaining"] = (blockedUntilMs - millis())/1000.0f;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleManualStop() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  manualActive = false;
  currentState = STATE_IDLE;
  stateStartMs = millis();
  activatedStartMs = 0;
  holdEndMs = 0;
  actuatorWrite(false);
  JsonDocument resp; resp["ok"] = true; String out; serializeJson(resp, out);
  server.send(200, "application/json", out);
}

// ---------- VL53 init / read / state machine ----------
void setupVL53() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(10);
  lox.setTimeout(500);
  if (lox.init()) { lox.startContinuous(); loxReady = true; } else loxReady = false;
}

void stateMachineStep() {
  unsigned long now = millis();

  if (blockedUntilMs > now) {
    if (currentState != STATE_BLOCKED) { currentState = STATE_BLOCKED; actuatorWrite(false); }
    return;
  } else if (currentState == STATE_BLOCKED) {
    currentState = STATE_IDLE; stateStartMs = now;
  }

  if (currentState == STATE_MANUAL) {
    if (!manualActive) { actuatorWrite(false); currentState = STATE_IDLE; stateStartMs = now; return; }
    // Não verifica hold para manual
    if (activatedStartMs > 0 && maxOnMinutes > 0 && (now - activatedStartMs >= (unsigned long)maxOnMinutes * 60000UL)) { manualActive = false; actuatorWrite(false); currentState = STATE_BLOCKED; blockedUntilMs = now + ((unsigned long)blockMinutes * 60000UL); return; }
    return;
  }

  bool presenceNow = (lastDistanceCm >= 0.0f && lastDistanceCm <= thresholdCm);

  if (presenceNow) {
    if (presenceFirstMs == 0) presenceFirstMs = now;
  } else {
    // Iniciar hold quando presença terminar (holdSeconds = 0 -> desliga imediatamente)
    if (presenceFirstMs != 0 && currentState == STATE_TRIGGERED) {
      holdEndMs = now + ((unsigned long)holdSeconds * 1000UL);
    }
    presenceFirstMs = 0;
  }

  if (currentState == STATE_IDLE) { if (presenceNow) { currentState = STATE_PRESENCE_COUNTING; stateStartMs = now; } return; }

  if (currentState == STATE_PRESENCE_COUNTING) {
    if (!presenceNow) { currentState = STATE_IDLE; stateStartMs = now; return; }
    unsigned long presenceElapsed = now - presenceFirstMs;
    if (presenceElapsed >= (unsigned long)presenceSeconds * 1000UL) {
      actuatorWrite(true);
      if (activatedStartMs == 0) activatedStartMs = presenceFirstMs;
      currentState = STATE_TRIGGERED; stateStartMs = now;
      return;
    }
    return;
  }

  if (currentState == STATE_TRIGGERED) {
    if (!presenceNow) {
      if (holdEndMs > 0 && now >= holdEndMs) {
        actuatorWrite(false);
        currentState = STATE_IDLE; stateStartMs = now; holdEndMs = 0; activatedStartMs = 0; return;
      }
    } else {
      // Presença ainda detectada, reset hold se aplicável
      holdEndMs = 0;
    }
    if (activatedStartMs > 0 && maxOnMinutes > 0) {
      if ((now - activatedStartMs) >= (unsigned long)maxOnMinutes * 60000UL) {
        actuatorWrite(false);
        currentState = STATE_BLOCKED; blockedUntilMs = now + ((unsigned long)blockMinutes * 60000UL);
        presenceFirstMs = 0; activatedStartMs = 0; holdEndMs = 0; return;
      }
    }
    return;
  }
}

// ---------- read & filter ----------
unsigned long lastReadMs = 0;  // Declarado globalmente para evitar bugs

void readAndFilter() {
  if (!loxReady) return;
  unsigned long now = millis();
  if (now - lastReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastReadMs = now;

  uint16_t mm = lox.readRangeContinuousMillimeters();
  bool to = lox.timeoutOccurred();
  bool valid = false;
  float thisCm = -1.0f;
  if (!to && mm != 0 && mm <= VL53_MAX_MM) { thisCm = mm / 10.0f; valid = true; }

  if (valid) {
    validStreak++; invalidStreak = 0; lastRawDistanceCm = thisCm;
    if (validStreak >= REQUIRED_VALID_READS) lastDistanceCm = lastRawDistanceCm;
  } else {
    invalidStreak++; validStreak = 0;
    if (invalidStreak >= MAX_INVALID_READS) { lastDistanceCm = -1.0f; lastRawDistanceCm = -1.0f; }
  }
}

// ---------- button factory handling ----------
void handleFactoryButton() {
  unsigned long now = millis();
  // read with simple debounce interval
  if (now - lastButtonReadMs < BUTTON_DEBOUNCE_MS) return;
  lastButtonReadMs = now;

  int v = digitalRead(GPIO_FACTORY); // HIGH when released (pull-up), LOW when pressed
  if (v == LOW) {
    if (!factoryPressed) {
      factoryPressed = true;
      factoryPressStart = now;
    } else {
      // already pressed - check duration
      if (!factoryResetDone && !factoryActionInProgress && (now - factoryPressStart >= FACTORY_LONGPRESS_MS)) {
        // perform factory reset once; wait until release to allow another
        doFactoryReset();
        factoryResetDone = true;
      }
    }
  } else {
    // released
    factoryPressed = false;
    factoryPressStart = 0;
    factoryResetDone = false;
  }
}

// ---------- setup / loop ----------
void setup() {
  pinMode(GPIO_ACT, OUTPUT);
  actuatorWrite(false);

  // button pin
  pinMode(GPIO_FACTORY, INPUT_PULLUP);

  // load params
  loadSensorParamsFromEEPROM();
  loadAPConfigFromFS();
  WiFi.persistent(false); // não grava config Wi-Fi na flash a cada softAP()
  WiFi.mode(WIFI_AP);
  applyAPConfig();

  // init sensor
  setupVL53();

  // endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/getParams", HTTP_GET, handleGetParams);
  server.on("/setParams", HTTP_POST, handleSetParams);
  server.on("/distance", HTTP_GET, handleDistance);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/manualStart", HTTP_POST, handleManualStart);
  server.on("/manualStop", HTTP_POST, handleManualStop);
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();  // Processar consultas DNS
  server.handleClient();
  readAndFilter();
  stateMachineStep();  // roda mesmo sem sensor, para manual/bloqueio funcionarem
  handleFactoryButton();

  // Apply pending AP restart outside handler
  if (pendingAPRestart && millis() >= pendingAPRestartAt) {
    pendingAPRestart = false;
    applyAPConfig();
  }
}