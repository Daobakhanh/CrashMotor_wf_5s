#include "mcd.h"

String DataToSend::toJson()
{
    StaticJsonDocument<200> doc;
    StaticJsonDocument<200> doc1;
    String result = "";
    JsonArray jsonLocation = doc1.createNestedArray();
    jsonLocation.add(location[0]);
    jsonLocation.add(location[1]);
    // Add values to the document
    doc["deviceId"] = deviceId;
    doc["location"] = jsonLocation;
    doc["status"] = status;
    doc["antiTheft"] = antiTheft;
    doc["battery"] = battery;
    doc["isConnected"] = isConnected;   // Serialize the JSON document
    serializeJson(doc, result);
    return result;
}

DataForReceive DataForReceive::fromJson(String jsonData)
{
    DataForReceive dataForReceive;

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, jsonData);
    JsonObject obj = doc.as<JsonObject>();

    dataForReceive.deviceId = obj[F("deviceId")].as<String>();
    dataForReceive.updateLocation = obj[F("updateLocation")].as<bool>();
    dataForReceive.antiTheft = obj[F("antiTheft")].as<bool>();
    dataForReceive.warning = obj[F("warning")].as<bool>();

    return dataForReceive;
}
