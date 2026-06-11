// ============================================================
//  camera_stream.h — camera init + MJPEG HTTP server
// ============================================================
//  This is YOLO's frame source. cameraInit() brings up the sensor;
//  streamServerBegin() starts the HTTP server (/ and /stream);
//  streamServerHandle() must be pumped from loop().
// ============================================================
#pragma once

bool cameraInit();
void streamServerBegin();
void streamServerHandle();
