#ifndef MCD_H
#define MCD_H
#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>

enum DeviceStatus
{
  NONE = 0, // Bình thường 
  FALL = 1, // Xe bị đổ (sms + buzzer)
  CRASH = 2, // Xe tai nạn (call + buzzer)
  LOST1 = 3, // Xe ra khỏi vùng an toàn  R > 10m (sms) 
  LOST2 = 4, // Xe ra khỏi vùng an toàn  R > 50m (call)
  SOS = 5 // SOS (sms + call)
};


struct DataToSend
{
    String deviceId = "";
    std::vector<double> location = {0,0};
    DeviceStatus status = NONE;
    bool antiTheft = false;
    bool isConnected = false;
    int battery = 0;
    String toJson();
};

struct DataForReceive
{
    String deviceId;
    bool updateLocation = false;
    bool antiTheft;
    bool warning = true;    
    static DataForReceive fromJson(String jsonData);
};
#endif