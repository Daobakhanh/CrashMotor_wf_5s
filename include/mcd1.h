#ifndef MCD1_H
#define MCD1_H
#include <Arduino.h>
#include <vector>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "mcd.h"
#include "Base64.h"
#include <WiFi.h>
#include <math.h>


struct Location
{
  double lat;
  double lng;
};


double calculateDistance(Location a,Location b);

DeviceStatus antiTheft(Location last, Location current);
bool mpuInit();
DeviceStatus checkFallandCrash();
bool handleDeviceStatus();


std::vector<String> splitString(String inputString, char delimiter);
void updateData();
void publishData(String data,String topic);

#endif