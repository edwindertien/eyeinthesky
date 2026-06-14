#pragma once
#include <Arduino.h>

// =========================================================================
//  web_ui.h -- WiFi AP + WebSocket configuration interface
//
//  Each EyeWatcher unit runs as a WiFi access point:
//    SSID:     EyeWatcher-{UNIT_ID}   (e.g. EyeWatcher-1)
//    Password: eyewatch
//    IP:       192.168.4.1
//
//  Connect with mobile browser to http://192.168.4.1
//
//  WebSocket protocol (JSON):
//    Server -> Client:  status frames at 5fps
//    Client -> Server:  command strings (same as serial commands)
//
//  Status frame format:
//  {
//    "state": "TRACKING",
//    "fps": 11,
//    "pan": 90.1, "tilt": 85.0, "arousal": 0.55,
//    "blobs": 2,
//    "bgSettled": true,
//    "motion": "base64...",   // DBG_W*DBG_H bytes, base64 encoded
//    "blobMap": "base64...",  // DBG_W*DBG_H bytes, base64 encoded
//    "blobList": [
//      {"id":3, "x":0.4, "y":0.3, "w":0.1, "h":0.2, "score":0.6}
//    ],
//    "log": "last log line"
//  }
// =========================================================================

void webUiBegin();
void webUiLoop();   // call from main loop -- handles WebSocket cleanup

// Push a log line to all connected WebSocket clients
void webUiLog(const char* msg);