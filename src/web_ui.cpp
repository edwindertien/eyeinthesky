#include "web_ui.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "vision.h"
#include "blob_tracker.h"
#include "behaviour.h"
#include "servo_eye.h"
#include "calibration.h"
#include "states.h"

// STRINGIFY must be defined before use
#define STRINGIFY2(x) #x
#define STRINGIFY(x)  STRINGIFY2(x)

#ifndef UNIT_ID
#define UNIT_ID 1
#endif

#define AP_SSID     "EyeWatcher-" STRINGIFY(UNIT_ID)
#define AP_PASS     "eyewatch"
#define AP_IP       IPAddress(192, 168, 4, 1)
#define WS_STATUS_INTERVAL  1000  // ms between status-only frames (tiny JSON)
#define WS_CANVAS_INTERVAL  3000  // ms between canvas frames (larger payload)

static AsyncWebServer  server(80);
static AsyncWebSocket  ws("/ws");
static DNSServer       dns;
static uint32_t        lastStatusMs = 0;
static uint32_t        lastCanvasMs = 0;
static bool            canvasRequested = false;  // set true when client sends "canvas"
static String          lastLogLine = "";

// ── Base64 encoder (simple, no line breaks) ───────────────────────────────────
static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static String b64Encode(const uint8_t* data, size_t len) {
    String out;
    out.reserve((len * 4 / 3) + 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t b = (uint32_t)data[i] << 16;
        if (i+1 < len) b |= (uint32_t)data[i+1] << 8;
        if (i+2 < len) b |= data[i+2];
        out += B64[(b >> 18) & 0x3F];
        out += B64[(b >> 12) & 0x3F];
        out += (i+1 < len) ? B64[(b >>  6) & 0x3F] : '=';
        out += (i+2 < len) ? B64[(b      ) & 0x3F] : '=';
    }
    return out;
}

// ── WebSocket command handler ─────────────────────────────────────────────────
// Reuse the same command processing as the serial handler.
// We declare this extern -- main.cpp implements it.
extern void processCommand(const String& cmd);

static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WebUI] Client %u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WebUI] Client %u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len
            && info->opcode == WS_TEXT) {
            String cmd = String((char*)data, len);
            cmd.trim();
            if (cmd == "canvas") {
                canvasRequested = true;  // send canvas on next loop
            } else {
                Serial.printf("[WebUI] cmd: %s\n", cmd.c_str());
                processCommand(cmd);
            }
        }
    }
}

// ── Status frame builder ──────────────────────────────────────────────────────
// Small status frame -- state, pan, tilt, arousal, blobs, log
static void pushStatusOnly() {
    if (ws.count() == 0) return;
    ws.cleanupClients();

    BlobResult blobs = {};
    if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);

    VisionDebug dbg;
    bool hasDbg = visionGetDebug(dbg);

    JsonDocument doc;
    doc["state"]     = stateName(behaviour_sm.getState());
    doc["fps"]       = hasDbg ? (int)dbg.fps : 0;
    doc["pan"]       = (int)round(eye.getPanDeg());
    doc["tilt"]      = (int)round(eye.getTiltDeg());
    doc["arousal"]   = round(eye.getArousal() * 100) / 100.0f;
    doc["blobs"]     = blobs.count;
    doc["bgSettled"] = (bool)visionBgSettled;
    doc["mA"]        = servoBoard.readCurrentMA();
    doc["log"]       = lastLogLine;
    if (hasDbg)
        doc["mode"]  = (dbg.mode == VisionMode::BLOBS) ? "blob" : "sal";

    // Blob list (small)
    JsonArray blobArr = doc["blobList"].to<JsonArray>();
    for (int i = 0; i < blobs.count; i++) {
        JsonObject b = blobArr.add<JsonObject>();
        b["id"]    = blobs.blobs[i].id;
        b["x"]     = round(blobs.blobs[i].normX * 100) / 100.0f;
        b["y"]     = round(blobs.blobs[i].normY * 100) / 100.0f;
        b["score"] = round(blobs.blobs[i].score * 100) / 100.0f;
    }

    lastLogLine = "";  // clear after sending

    String json;
    json.reserve(256);
    serializeJson(doc, json);
    ws.textAll(json);
}

// Canvas frame -- includes motion mask and blob map (larger payload)
static void pushCanvas() {
    if (ws.count() == 0) return;

    VisionDebug dbg;
    if (!visionGetDebug(dbg)) return;

    BlobResult blobs = {};
    if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);

    JsonDocument doc;
    doc["type"] = "canvas";
    doc["mode"] = (dbg.mode == VisionMode::BLOBS) ? "blob" : "sal";
    doc["mw"] = 20; doc["mh"] = 15;

    // Downsample 40x30 -> 20x15
    static uint8_t sm[20*15], sb[20*15];
    for (int y=0; y<15; y++)
        for (int x=0; x<20; x++) {
            int s=0;
            for (int dy=0;dy<2;dy++)
                for (int dx=0;dx<2;dx++)
                    s += dbg.motion[(y*2+dy)*DBG_W+(x*2+dx)];
            sm[y*20+x] = s/4;
            s=0;
            for (int dy=0;dy<2;dy++)
                for (int dx=0;dx<2;dx++)
                    s += dbg.blobs[(y*2+dy)*DBG_W+(x*2+dx)];
            sb[y*20+x] = s/4;
        }
    doc["motion"]  = b64Encode(sm, 20*15);
    doc["blobMap"] = b64Encode(sb, 20*15);

    // Blob list with size info for canvas drawing
    JsonArray blobArr = doc["blobList"].to<JsonArray>();
    for (int i = 0; i < blobs.count; i++) {
        JsonObject b = blobArr.add<JsonObject>();
        b["id"]    = blobs.blobs[i].id;
        b["x"]     = round(blobs.blobs[i].normX * 100) / 100.0f;
        b["y"]     = round(blobs.blobs[i].normY * 100) / 100.0f;
        b["w"]     = round(blobs.blobs[i].normW * 100) / 100.0f;
        b["h"]     = round(blobs.blobs[i].normH * 100) / 100.0f;
        b["score"] = round(blobs.blobs[i].score * 100) / 100.0f;
    }

    String json;
    json.reserve(1024);
    serializeJson(doc, json);
    ws.textAll(json);
}

// ── Embedded HTML page ────────────────────────────────────────────────────────
static const char PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>EyeWatcher</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#111;color:#ddd;font-family:monospace;font-size:13px}
#header{background:#1a1a2e;padding:8px 12px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #333}
#header h1{font-size:15px;color:#7eb8f7}
#status{font-size:11px;color:#aaa;text-align:right}
#canvases{display:flex;gap:4px;padding:6px;background:#0a0a0a}
.cvwrap{flex:1;min-width:0}
.cvlabel{font-size:10px;color:#666;text-align:center;padding:2px}
canvas{width:100%;display:block;image-rendering:pixelated}
#blobinfo{padding:4px 8px;min-height:22px;font-size:11px;color:#fa0}
#quickbtns{display:flex;flex-wrap:wrap;gap:4px;padding:6px 8px;background:#141414;border-top:1px solid #222}
.qbtn{background:#1e3a5f;color:#7eb8f7;border:1px solid #2a5a8f;padding:5px 10px;border-radius:4px;cursor:pointer;font-size:12px;font-family:monospace}
.qbtn:active{background:#2a5a8f}
.qbtn.red{background:#5f1e1e;color:#f78;border-color:#8f2a2a}
#cmdinput{display:flex;gap:4px;padding:6px 8px;border-top:1px solid #222}
#cmd{flex:1;background:#0a0a0a;color:#ddd;border:1px solid #333;padding:6px 8px;font-family:monospace;font-size:13px;border-radius:4px}
#sendbtn{background:#1e3a1e;color:#7ef77e;border:1px solid #2a8f2a;padding:6px 12px;border-radius:4px;cursor:pointer;font-family:monospace}
#log{height:120px;overflow-y:auto;padding:6px 8px;background:#0a0a0a;border-top:1px solid #222;font-size:11px;color:#888}
#log .line{padding:1px 0;border-bottom:1px solid #1a1a1a}
#log .line.state{color:#7eb8f7}
#log .line.warn{color:#fa8}
#connbar{background:#333;text-align:center;font-size:11px;padding:3px;color:#aaa}
#connbar.ok{background:#1a3a1a;color:#7f7}
#connbar.err{background:#3a1a1a;color:#f77}
</style>
</head><body>
<div id="connbar" class="err">Connecting...</div>
<div id="header">
  <h1>EyeWatcher</h1>
  <div id="status">--</div>
</div>
<div id="canvases">
  <div class="cvwrap">
    <div class="cvlabel" id="mlabel">MOTION</div>
    <canvas id="mc" width="40" height="30"></canvas>
  </div>
  <div class="cvwrap">
    <div class="cvlabel">BLOBS</div>
    <canvas id="bc" width="40" height="30"></canvas>
  </div>
</div>
<div id="blobinfo">--</div>
<div id="quickbtns">
  <button class="qbtn" onclick="send('bgreset')">bgreset</button>
  <button class="qbtn" onclick="send('blob')">blob mode</button>
  <button class="qbtn" onclick="send('sal')">sal mode</button>
  <button class="qbtn" onclick="send('info')">info</button>
  <button class="qbtn" onclick="send('blobs')">blobs</button>
  <button class="qbtn" onclick="send('caminfo')">caminfo</button>
  <button class="qbtn" onclick="send('home')">home</button>
  <button class="qbtn" onclick="send('blink')">blink</button>
  <button class="qbtn red" onclick="send('reboot')">reboot</button>
</div>
<div id="cmdinput">
  <input id="cmd" type="text" placeholder="command..." autocomplete="off" autocorrect="off" autocapitalize="off" spellcheck="false">
  <button id="sendbtn" onclick="sendCmd()">Send</button>
</div>
<div id="log"></div>

<script>
let W=20,H=15;  // updated from server on first frame
let ws,reconnTimer;
let mImg,bImg;

// Canvas contexts
const mcEl=document.getElementById('mc');
const bcEl=document.getElementById('bc');
const mc=mcEl.getContext('2d');
const bc=bcEl.getContext('2d');

function initCanvases(w,h){
  if(W===w&&H===h&&mImg)return;
  W=w;H=h;
  mcEl.width=w;bcEl.width=w;
  mcEl.height=h;bcEl.height=h;
  mImg=mc.createImageData(w,h);
  bImg=bc.createImageData(w,h);
}
initCanvases(20,15);

// Base64 decode
function b64decode(s){
  const b=atob(s),u=new Uint8Array(b.length);
  for(let i=0;i<b.length;i++)u[i]=b.charCodeAt(i);
  return u;
}

// Render motion mask: grey levels with heat colouring
function renderMotion(data,mode){
  if(!mImg)return;
  const d=mImg.data;
  for(let i=0;i<W*H;i++){
    const v=data[i];
    const p=i*4;
    if(mode==='sal'){
      // saliency: green tint
      d[p]=0;d[p+1]=v;d[p+2]=v>>2;d[p+3]=255;
    } else {
      // motion: white->yellow->red
      if(v>180){d[p]=255;d[p+1]=60;d[p+2]=60;}
      else if(v>80){d[p]=255;d[p+1]=200;d[p+2]=0;}
      else{d[p]=v;d[p+1]=v;d[p+2]=v;}
      d[p+3]=255;
    }
  }
  mc.putImageData(mImg,0,0);
}

// Render blob map
function renderBlobs(data,blobList){
  if(!bImg)return;
  const d=bImg.data;
  const colors=[[255,80,80],[80,255,80],[80,180,255],[255,200,0],[200,80,255]];
  for(let i=0;i<W*H;i++){
    const v=data[i];const p=i*4;
    if(v===255){d[p]=255;d[p+1]=80;d[p+2]=80;d[p+3]=255;}
    else if(v>0){
      const ci=(Math.round(v/40)-1)%colors.length;
      const c=colors[ci];
      d[p]=c[0];d[p+1]=c[1];d[p+2]=c[2];d[p+3]=180;
    } else {
      d[p]=20;d[p+1]=20;d[p+2]=20;d[p+3]=255;
    }
  }
  bc.putImageData(bImg,0,0);
  // Draw blob centroids as crosses
  if(blobList){
    bc.strokeStyle='#fff';bc.lineWidth=1;
    blobList.forEach((b,i)=>{
      const x=b.x*W,y=b.y*H;
      const c=colors[i%colors.length];
      bc.strokeStyle=`rgb(${c[0]},${c[1]},${c[2]})`;
      bc.beginPath();bc.moveTo(x-2,y);bc.lineTo(x+2,y);
      bc.moveTo(x,y-2);bc.lineTo(x,y+2);bc.stroke();
    });
  }
}

function addLog(line,cls){
  const log=document.getElementById('log');
  const div=document.createElement('div');
  div.className='line'+(cls?' '+cls:'');
  div.textContent=line;
  log.appendChild(div);
  // Keep last 100 lines
  while(log.children.length>100)log.removeChild(log.firstChild);
  log.scrollTop=log.scrollHeight;
}

let lastState='';
function onMessage(evt){
  let d;
  try{d=JSON.parse(evt.data);}catch(e){addLog(evt.data,'warn');return;}

  if(d.type==='canvas'){
    // Canvas-only frame
    if(d.mw&&d.mh)initCanvases(d.mw,d.mh);
    if(d.motion)renderMotion(b64decode(d.motion),d.mode||'blob');
    if(d.blobMap)renderBlobs(b64decode(d.blobMap),d.blobList);
    document.getElementById('mlabel').textContent=
      d.mode==='sal'?'SALIENCY':'MOTION';
    return;
  }

  // Status frame
  const stateColors={SLEEPING:'#777',WAKING:'#fa0',SCANNING:'#7ef',
    TRACKING:'#7f7',FOCUS:'#f7f',DOZING:'#fa0',SCARED:'#f77'};
  const sc=stateColors[d.state]||'#ddd';
  document.getElementById('status').innerHTML=
    `<span style="color:${sc}">${d.state}</span> `+
    `fps:${d.fps} pan:${d.pan} tilt:${d.tilt} `+
    `ar:${d.arousal} ${d.mA}mA`;

  // Blob info from status
  let bi='';
  if(d.blobList&&d.blobList.length){
    bi=d.blobList.map(b=>`#${b.id} (${b.x},${b.y}) s=${b.score}`).join('  ');
  } else {
    bi=d.bgSettled?'no blobs':'bg settling...';
  }
  document.getElementById('blobinfo').textContent=bi;

  // Log state transitions
  if(d.state!==lastState){
    addLog(d.state,'state');
    lastState=d.state;
  }
  if(d.log&&d.log.length){
    const cls=d.log.includes('State')?'state':
              d.log.includes('WARN')||d.log.includes('FAIL')?'warn':'';
    addLog(d.log,cls);
  }
}

function send(cmd){
  if(ws&&ws.readyState===WebSocket.OPEN){
    ws.send(cmd);
    addLog('> '+cmd);
  }
}
function sendCmd(){
  const el=document.getElementById('cmd');
  const v=el.value.trim();
  if(v){send(v);el.value='';}
}
document.getElementById('cmd').addEventListener('keydown',e=>{
  if(e.key==='Enter')sendCmd();
});

function connect(){
  const bar=document.getElementById('connbar');
  bar.className='err';bar.textContent='Connecting...';
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{
    bar.className='ok';bar.textContent='Connected to '+location.hostname;
    addLog('Connected','state');
    // Request canvas immediately on connect, then every 3s
    setTimeout(()=>{if(ws.readyState===WebSocket.OPEN)ws.send('canvas');},500);
    setInterval(()=>{if(ws.readyState===WebSocket.OPEN)ws.send('canvas');},3000);
  };
  ws.onmessage=onMessage;
  ws.onerror=()=>{bar.className='err';bar.textContent='WebSocket error';};
  ws.onclose=()=>{
    bar.className='err';bar.textContent='Disconnected -- reconnecting...';
    addLog('Disconnected','warn');
    clearTimeout(reconnTimer);
    reconnTimer=setTimeout(connect,2000);
  };
}
connect();
</script>
</body></html>
)rawhtml";

// ── webUiLog -- push a log line to all WS clients ────────────────────────────
void webUiLog(const char* msg) {
    lastLogLine = String(msg);
    // Don't push immediately -- it goes in the next status frame
}

// ── webUiBegin ────────────────────────────────────────────────────────────────
void webUiBegin() {
    Serial.println("[WebUI] Starting WiFi AP...");

    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_AP);
    delay(100);

    bool cfgOk = WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    Serial.printf("[WebUI] softAPConfig: %s\n", cfgOk ? "OK" : "FAILED");

    bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WebUI] softAP: %s\n", apOk ? "OK" : "FAILED");

    delay(500);  // give AP time to start
    Serial.printf("[WebUI] SSID: %s  IP: %s  Channel: %d\n",
                  AP_SSID,
                  WiFi.softAPIP().toString().c_str(),
                  WiFi.channel());

    // WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Serve the page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", PAGE);
    });

    // Simple REST endpoints for quick actions
    server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (req->hasParam("v")) {
            String cmd = req->getParam("v")->value();
            processCommand(cmd);
            req->send(200, "text/plain", "ok");
        } else {
            req->send(400, "text/plain", "missing ?v=");
        }
    });

    // Captive portal -- redirect all unknown URLs to the main page.
    // This makes iOS/Android automatically open the browser when joining the AP.
    server.onNotFound([](AsyncWebServerRequest* req) {
        // Captive portal detection URLs used by iOS, Android, Windows
        String url = req->url();
        if (url == "/generate_204" ||          // Android
            url == "/hotspot-detect.html" ||   // iOS/macOS
            url == "/ncsi.txt" ||              // Windows
            url == "/connecttest.txt" ||       // Windows 10
            url == "/redirect" ||
            url == "/canonical.html") {
            req->redirect("http://192.168.4.1/");
        } else {
            req->redirect("http://192.168.4.1/");
        }
    });

    // DNS -- not needed for captive portal on most devices since we
    // redirect at HTTP level, but helps Android detect the portal faster.
    // (requires DNSServer library -- already in ESP32 Arduino core)

    server.begin();

    // DNS server: redirect all domains to our IP (captive portal)
    dns.start(53, "*", AP_IP);
    Serial.println("[WebUI] Server started -- connect to http://192.168.4.1");
    Serial.println("[WebUI] Or join WiFi '" AP_SSID "' and browser should auto-open");
}

// ── webUiLoop ─────────────────────────────────────────────────────────────────
void webUiLoop() {
    dns.processNextRequest();
    ws.cleanupClients();

    if (ws.count() == 0) return;  // no clients -- skip all serialization work

    uint32_t now = millis();

    // Status frame -- tiny JSON, sent every second
    if (now - lastStatusMs >= WS_STATUS_INTERVAL) {
        lastStatusMs = now;
        pushStatusOnly();
    }

    // Canvas frame -- larger, sent every 3s or on demand
    if (canvasRequested || (now - lastCanvasMs >= WS_CANVAS_INTERVAL)) {
        lastCanvasMs    = now;
        canvasRequested = false;
        pushCanvas();
    }
}