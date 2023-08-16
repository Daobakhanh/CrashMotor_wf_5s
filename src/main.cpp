#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <vector>
#include "mcd.h"
#include "mcd1.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Base64.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SSID "PP"
#define PASSWORD "password"
// #define USE_SIM
#define USE_WIFI

#define BUTTON_SOS 18
#define BUZZER 5
#define DATA_UPDATE_CYCLE_DEFAUlT 30000
#define DATA_UPDATE_CYCLE_SOS 5000
#define SYSTEM_CYCLE_DEFAULT 100000
#define TIME_CHECK_FALL 2000
#define RX1 23
#define TX1 22
#define RX2 4
#define TX2 2
#define SimSerial Serial2
#define GpsSerial Serial1
#define BATTERY_PIN 15
#define POWER_CONNECT_PIN 16

const String MQTT_HOST = "nby.ddns.net";
const int MQTT_PORT = 63001;
const String MQTT_USER = "";
const String MQTT_PASSWORD = "";
const String PUB_LOCATION_TOPIC = "/noibinhyen/mcd/location";
const String SUB_UPDATE_TOPIC = "/noibinhyen/mcd/update";
const char* MQTT_PUB_TOPIC = "/noibinhyen/mcd/location";
const char* MQTT_SUB_TOPIC = "/noibinhyen/mcd/update";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

uint32_t countBaterry = 0;
TinyGPSPlus gps;
DataToSend dataToSend;
DataForReceive dataForReceive;
String dataReceiveMqtt = "";
DeviceStatus deviceStatus = NONE;

Location savedLocation;
Location currentLocation;

bool statusAntiTheft = false;
bool statusWarining = false;
bool tempFall = false;

bool checklengthDataReciver = 0;
int lengthDataReciver;
bool doneGetData = 1;
String dataReceiver = "";
bool isDataReceiver = false;
String deviceId = WiFi.macAddress();
bool statusBuzzer = HIGH;

unsigned long now;
unsigned long last;
unsigned long timeCheckAntiTheft;
uint32_t timeCheckCrash;
int dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT; // time update data
int systemCycle = SYSTEM_CYCLE_DEFAULT;			 // 60000ms = 60s = 1'
bool isPress;
uint32_t countPress = 0;
uint32_t timeUpdateData = 0;
uint32_t timeBuzzerToggle = 0;

void setupPin();
void getGPS();
bool checkSosAndAntiTheft(uint8_t sosPin, unsigned int timeOut = 2000U);
void initMQTT();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void pubDataUseWifi(String data);
void handlelDataReceiver();
void getBattery();

void IRAM_ATTR handleButton();

void setup() {
	setupPin();
	mpuInit();
	initMQTT();
	now = millis();
	last = millis();
	timeCheckAntiTheft = millis();
	// deviceStatus = CRASH;
}

void loop() {
	client.loop();
	// handel Data Reciver
	if (isDataReceiver == true)
	{
		handlelDataReceiver();
		isDataReceiver = false;
	}
	// reconnect
	if (!client.connected())
	{
		reconnect();
	}
	// check button
	if (isPress == true)
	{
		if (deviceStatus == NONE || deviceStatus == CRASH)
		{
			checkSosAndAntiTheft(BUTTON_SOS); // kiểm tra vào chức năng SOS hoặc chống trộm
			isPress = false;
		} else
		{
			countPress = 0;
			isPress = false;
		}
		timeUpdateData = true;
	}
	// CHECK location
	if (deviceStatus == NONE || deviceStatus == LOST1)
	{
		if (statusAntiTheft == true && now - timeCheckAntiTheft > 3000)
		{
			deviceStatus = antiTheft(savedLocation, currentLocation); // chống trộm
			timeCheckAntiTheft = now;
		}
	}
	// check đổ
	if (deviceStatus == NONE)
	{
		if (now - timeCheckCrash > 20)
		{
			deviceStatus = checkFallandCrash();
			timeCheckCrash = now;
		}
	}

	if (dataToSend.status != deviceStatus)
	{
		if (deviceStatus != NONE)
		{
			Serial.println("Update time data");
			statusWarining = true;
			// updateData();
			dataUpdateCycle = DATA_UPDATE_CYCLE_SOS;
			dataToSend.status = deviceStatus;
		} else
		{
			dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT;
			digitalWrite(BUZZER, HIGH);
		}
		// updateData();
	}

	if (deviceStatus == CRASH && millis() - timeBuzzerToggle > 500)
	{
		Serial.println("Warning");
		statusBuzzer = !statusBuzzer;
		digitalWrite(BUZZER, statusBuzzer);
		timeBuzzerToggle = millis();
	}

	if (now - timeUpdateData > dataUpdateCycle)
	{
		countBaterry++;
		getGPS();
		updateData();
		timeUpdateData = now;
	}
	// delay(10);
	now = millis();
	if (now - last > systemCycle)
	{
		last = now;
	}
}
void setupPin() {
	Wire.begin(32, 33);
	Serial.begin(9600);
	GpsSerial.begin(9600, SERIAL_8N1, TX1, RX1);
	// pinMode(BUTTON_SOS, INPUT_PULLDOWN);

	pinMode(BUTTON_SOS, INPUT_PULLDOWN);

	pinMode(BUZZER, OUTPUT);
	pinMode(POWER_CONNECT_PIN, INPUT);
	pinMode(BATTERY_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTON_SOS), handleButton, RISING);
	getBattery();
	dataToSend.deviceId = WiFi.macAddress();
	digitalWrite(BUZZER, statusBuzzer);
}

void IRAM_ATTR handleButton() {
	countPress++;
	isPress = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Message arrived [");
	Serial.print(topic);
	Serial.print("] ");
	dataReceiver = "";
	for (int i = 0; i < length; i++)
	{
		// Serial.print((char)payload[i]);
		dataReceiver += (char)payload[i];
	}
	//
	isDataReceiver = true;
	Serial.println(dataReceiver);
}

void reconnect() {
	while (!client.connected())
	{
		String clientId = "Client";
		// Attempt to connect
		if (client.connect(clientId.c_str()))
		{
			Serial.print("Reconnect ");
			// client.publish(MQTT_PUB_TOPIC, "hello world");
			client.subscribe(MQTT_SUB_TOPIC);
		} else
		{
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 5 seconds");
			// Wait 5 seconds before retrying
			delay(5000);
		}
	}
}

bool handleDeviceStatus() {
	bool updateLocation = false;
	unsigned long time1 = millis();
	unsigned long time2 = millis();
	switch (deviceStatus)
	{
	case NONE:
		// skip
		break;
	case CRASH:
		dataToSend.status = CRASH;
		deviceStatus = CRASH;
		break;
	case SOS:
		dataToSend.status = SOS;
		deviceStatus = SOS;

		break;
	}

	if (deviceStatus == LOST1 || deviceStatus == LOST2)
	{
		// updateData();
		time1 = millis();
		time2 = millis();
		unsigned long time3 = millis();
		statusWarining = true;
	}
	if (statusAntiTheft == true)
	{
		getGPS();
		savedLocation.lat = currentLocation.lat;
		savedLocation.lng = currentLocation.lng;
	}
	if (deviceStatus != NONE)
	{
		statusWarining = true;
		updateData();
		dataUpdateCycle = DATA_UPDATE_CYCLE_SOS;
	}
	return true;
}

void handlelDataReceiver() {
	dataForReceive = DataForReceive::fromJson(dataReceiver);

	if (statusAntiTheft != dataForReceive.antiTheft)
	{
		statusAntiTheft = dataForReceive.antiTheft;
		Serial.print("Status Anti Theft = ");
		Serial.println(statusAntiTheft);
	} else
	{
		if (statusWarining == true)
		{
			Serial.println("Turn off warning");
			statusWarining = false;
			deviceStatus = NONE;
			dataToSend.status = NONE;
			dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT;
			digitalWrite(BUZZER, HIGH);
		}
	}

	if (statusAntiTheft == true)
	{
		getGPS();
		savedLocation.lat = currentLocation.lat;
		savedLocation.lng = currentLocation.lng;
	}
	dataToSend.status = deviceStatus;
	dataReceiver = "";
	updateData();
	for (int i = 0; i < 6; i++)
	{
		statusBuzzer = !statusBuzzer;
		digitalWrite(BUZZER, statusBuzzer);
		delay(300);
	}
	digitalWrite(BUZZER, HIGH);
}
void updateData() {
	if (countBaterry >= 100)
	{
		dataToSend.battery = dataToSend.battery - random(0, 2);
		countBaterry = 0;
	}

	if (digitalRead(POWER_CONNECT_PIN) == HIGH)
	{
		dataToSend.isConnected = false;
	}

	else
	{

		dataToSend.isConnected = true;
	}

	dataToSend.status = deviceStatus;
	dataToSend.antiTheft = statusAntiTheft;
	pubDataUseWifi(dataToSend.toJson());
}

void pubDataUseWifi(String data) {
	// char dataConvert[200];
	// char array[data.length() + 1];
	// data.toCharArray(array, sizeof(array));
	// Base64.encode(dataConvert, array, data.length());
	// client.publish(MQTT_PUB_TOPIC, dataConvert);
	client.publish(MQTT_PUB_TOPIC, data.c_str());
}

void initMQTT() {
#ifdef USE_SIM
	String data;
	delay(1000);
	data = "AT+CMQNEW=\"" + MQTT_HOST + "\"" + ",\"" + String(MQTT_PORT) + "\",12000,1024\r\n";
	SimSerial.print(data);
	delay(100);
	data = "AT+CMQCON=0,3,\"gpsmqtt\",600,0,0\r\n";
	SimSerial.print(data);
	delay(2000);
	data = "AT+CMQSUB=0,\"" + SUB_UPDATE_TOPIC + "\",0\r\n";
	SimSerial.print(data);
	delay(1000);
#endif
#ifdef USE_WIFI
	WiFi.begin(SSID, PASSWORD);
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.println("connected");
	client.setServer("nby.ddns.net", 63001);
	client.setCallback(callback);
	if (client.connect("client"))
	{
		// client.publish(MQTT_PUB_TOPIC, "hello world");
		client.subscribe(MQTT_SUB_TOPIC);
	}
#endif
}

bool

checkSosAndAntiTheft(uint8_t sosPin, unsigned int timeOut) {
	Serial.print("Sos Pin press");
	Serial.println(digitalRead(sosPin));
	timeOut = 2000;
	if (statusWarining == true)
	{
		if (countPress >= 3)
		{
			statusWarining = false;
			deviceStatus = NONE;
			dataToSend.status = NONE;
			updateData();
			dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT;
			digitalWrite(BUZZER, HIGH);
			countPress = 0;
			delay(200);
			updateData();
			return true;
		}
	} else
	{
		if (countPress >= 5)
		{
			countPress = 0;
			Serial.print("AntiTheft ");
			statusAntiTheft = !statusAntiTheft;
			Serial.println(statusAntiTheft);
			updateData();
			for (int i = 0; i < 6; i++)
			{
				statusBuzzer = !statusBuzzer;
				digitalWrite(BUZZER, statusBuzzer);
				delay(300);
			}
			digitalWrite(BUZZER, HIGH);
			dataToSend.antiTheft = statusAntiTheft;
			now = millis();
			if (statusAntiTheft == true)
			{
				savedLocation.lat = currentLocation.lat;
				savedLocation.lng = currentLocation.lng;
			}
			updateData();
			delay(50);
			return true;
		}

		if (digitalRead(sosPin) == HIGH) // sosPin = 1
		{
			now = millis();
			// CHUA FIX
			while (millis() - now < 3000)
			{
				if (digitalRead(sosPin) != HIGH)
				{
					return true;
				}
				// now = millis();
				delay(100);
				/* code */
			}

			if (digitalRead(sosPin) == HIGH)
			{
				Serial.println("SOS");
				deviceStatus = SOS;
				dataToSend.antiTheft = statusAntiTheft;
				dataToSend.status = deviceStatus; // SOS
				statusWarining = true;
				dataUpdateCycle = DATA_UPDATE_CYCLE_SOS;
				updateData();
				return true;
			}
		}
	}
	return true;
}

void getBattery() {
	float volValue = analogRead(BATTERY_PIN);
	volValue = volValue * 3.3 * 6 / 4095;
	volValue += 0.8;
	Serial.println(volValue);
	volValue = (volValue - 6.2) / 2 * 100;
	if (volValue > 100)
		volValue = 95;
	dataToSend.battery = volValue;
}
void getGPS() {
	char* gpsStream = new char[100];
	char c;
	const char NEED_GET_DATA_PREFIX = '$';
	const char NEED_STOP_GET_DATA_SUFFIX = '\n';
	bool needGetData = false;
	int count = 0;
	gpsStream[73] = '\0';
	for (int i = 0; i < 3; i++)
	{
		while (GpsSerial.available())
		{
			c = GpsSerial.read();
			// Serial.print(c);
			if (c == NEED_GET_DATA_PREFIX)
			{
				needGetData = true;
			}
			if (needGetData)
			{
				gpsStream[count] = c;
				count++;
			}
			if (c == NEED_STOP_GET_DATA_SUFFIX && needGetData == true)
			{
				count = 0;
				needGetData = false;
				break;
			}
		}
		for (int i = 0; i < strlen(gpsStream); i++)
		{
			if (gps.encode(gpsStream[i]))
			{
			}
		}
	}
	currentLocation.lat = gps.location.lat();
	currentLocation.lng = gps.location.lng();
	dataToSend.location[0] = currentLocation.lat;
	dataToSend.location[1] = currentLocation.lng;
	// Serial.println(currentLocation.lat,8);
	// Serial.println(currentLocation.lng,8);
	delete (gpsStream);
}
