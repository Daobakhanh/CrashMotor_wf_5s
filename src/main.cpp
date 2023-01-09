#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <vector>
#include "mcd.h"
#include "mcd1.h"
#include <TinyGPSPlus.h>
#include <Base64.h>

#define BUTTON_SOS 26
#define BUZZER 32
#define DATA_UPDATE_CYCLE_DEFAUlT 30000
#define SYSTEM_CYCLE_DEFAULT 100000
#define TIME_CHECK_FALL 2000
#define RX1 2
#define TX1 15
#define SimSerial Serial2
#define GpsSerial Serial1

const String MQTT_HOST = "noibinhyen.ddns.net";
const int MQTT_PORT = 1883;
const String MQTT_USER = "";
const String MQTT_PASSWORD = "";
const String PUB_LOCATION_TOPIC = "/noibinhyen/mcd/location";
const String SUB_UPDATE_TOPIC = "/noibinhyen/mcd/update";

bool readDataFromSim();

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
String dataSimReciver = "";
String deviceId = WiFi.macAddress();
bool statusBuzzer = HIGH;

unsigned long now;
unsigned long last;
unsigned long timeCheckAntiTheft;
int dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT; //time update data
int systemCycle = SYSTEM_CYCLE_DEFAULT;			 // 60000ms = 60s = 1'

void getGPS();

bool checkSosAndAntiTheft(uint8_t sosPin, unsigned int timeOut = 2000U);

void initMQTT();

void setup()
{
	Serial.begin(9600);
	GpsSerial.begin(9600, SERIAL_8N1, TX1, RX1);
	SimSerial.begin(9600);
	pinMode(BUTTON_SOS, INPUT_PULLDOWN);
	pinMode(BUZZER, OUTPUT);

	digitalWrite(BUZZER, statusBuzzer);
	dataToSend.deviceId = WiFi.macAddress();
	now = millis();
	last = millis();
	mpuInit();
	initMQTT();
	timeCheckAntiTheft = millis();
}

void loop()
{
	while (readDataFromSim() == true)
		;
	delay(10);

	checkSosAndAntiTheft(BUTTON_SOS); // kiểm tra vào chức năng SOS hoặc chống trộm

	if (statusAntiTheft == true && now - timeCheckAntiTheft > 3000)
	{
		deviceStatus = antiTheft(savedLocation, currentLocation); // chống trộm
		timeCheckAntiTheft = now;
	}

	if (deviceStatus == NONE)
	{
		deviceStatus = checkFallandCrash();
	}

	handleDeviceStatus();

	if (statusWarining == true)
	{
		dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT / 6;
	}
	else
	{
		dataUpdateCycle = DATA_UPDATE_CYCLE_DEFAUlT;
	}
	if ((now - last) % dataUpdateCycle <= 20)
	{
		updateData();
	}
	delay(10);
	now = millis();
	if (now - last > systemCycle)
	{
		last = now;
	}
}

bool handleDeviceStatus()
{
	bool updateLocation = false;
	unsigned long time1 = millis();
	unsigned long time2 = millis();
	switch (deviceStatus)
	{
	case NONE:
		// skip
		break;
	case FALL:
		// coi keu + sms sau 5s
		while (millis() - time1 < TIME_CHECK_FALL)
		{
			if (checkFallandCrash() == CRASH)
			{
				// Serial.println("check");
				deviceStatus = CRASH;
				return false;
			}
			delay(10);
		}
		if (checkFallandCrash() == FALL)
		{
			// fall
			Serial.println("Fall");
			//dataToSend.status = FALL;
			deviceStatus = FALL;
			time1 = millis();
			time2 = millis();
			updateData();
			while ((millis() - time2) < 10000U)
			{
				if ((millis() - time1) > 500U)
				{
					Serial.println("Buzzer");
					digitalWrite(BUZZER, statusBuzzer);
					statusBuzzer = !statusBuzzer;
					time1 = millis();
				}
				if (checkFallandCrash() != FALL)
				{
					delay(1000);
					if (checkFallandCrash() != FALL)
					{
						statusBuzzer = HIGH;
						digitalWrite(BUZZER, statusBuzzer);
						deviceStatus = NONE;
						dataToSend.status = NONE;
						updateData();
						Serial.println("NONE");
						return true;
					}
				}
				delay(100);
			}
			updateData();
			statusBuzzer = HIGH;
			digitalWrite(BUZZER, statusBuzzer);
			time1 = millis();
			while (checkFallandCrash() == FALL)
			{
				if (millis() - time1 > dataUpdateCycle)
				{
					updateData();
					time1 = millis();
				}
				delay(100);
			}
			deviceStatus = NONE;
			updateData();
			return true;
		}
		break;
	case CRASH:
		dataToSend.status = CRASH;
		updateData();
		// dataToSend.status = NONE;
		statusWarining = true;
		while (statusWarining == true)
		{
			if ((millis() - time1) > 500U)
			{
				digitalWrite(BUZZER, statusBuzzer);
				statusBuzzer = !statusBuzzer;
				time1 = millis();
			}
			if ((millis() - time2) > DATA_UPDATE_CYCLE_DEFAUlT / 5)
			{
				updateData();
				time2 = millis();
			}
			while (readDataFromSim() == true)
				;
			delay(10);
		}
		statusBuzzer = HIGH;
		digitalWrite(BUZZER, statusBuzzer);
		deviceStatus = NONE;
		dataToSend.status = NONE;
		break;
	}

	if (deviceStatus == LOST1 || deviceStatus == LOST2)
	{
		// updateData();
		time1 = millis();
		time2 = millis();
		unsigned long time3 = millis();
		statusWarining = true;
		while (statusWarining == true)
		{
			if(time2 - time3 > 1000)
			{
				deviceStatus = antiTheft(savedLocation, currentLocation);
				time3 = time2;
			}
			
			while (readDataFromSim() == true)
				;
			if (time2 - time1 > DATA_UPDATE_CYCLE_DEFAUlT / 5)
			{
				updateData();
				time1 = time2;
			}
			time2 = millis();
			delay(10);
		}
		if(statusAntiTheft == true)
		{
			getGPS();
			savedLocation.lat = currentLocation.lat;
			savedLocation.lng = currentLocation.lng;
		}
	}
	return true;
}

void updateData()
{
	getGPS();
	dataToSend.status = deviceStatus;
	dataToSend.antiTheft = statusAntiTheft;
	publishData(dataToSend.toJson(), PUB_LOCATION_TOPIC);
}

bool readDataFromSim()
{
	String dataSubReciver = "";
	char c;
	std::vector<String> splittedString;
	if (SimSerial.available() > 0 || checklengthDataReciver == 1)
	{

		while (SimSerial.available() > 0)
		{
			c = SimSerial.read();
			dataSimReciver += String(c);
		}
		if (doneGetData == 0)
		{
			if ((dataSimReciver.length() - dataSimReciver.indexOf("{") - 5) < lengthDataReciver)
			{
				checklengthDataReciver = 0;
				doneGetData = 1;
				dataSubReciver = dataSimReciver.substring(dataSimReciver.indexOf("{"), dataSimReciver.indexOf("{") + lengthDataReciver);
				Serial.println(dataSubReciver);
				// Serial.println(dataSimReciver);
				dataForReceive = DataForReceive::fromJson(dataSubReciver);
				if (dataForReceive.deviceId == deviceId)
				{
					if (statusAntiTheft != dataForReceive.antiTheft)
					{
						for (int i = 0; i < 6; i++)
						{
							statusBuzzer = !statusBuzzer;
							digitalWrite(BUZZER, statusBuzzer);
							delay(300);
						}
						statusAntiTheft = dataForReceive.antiTheft;

						// Serial.print("Status Anti Theft = ");
						// Serial.println(statusAntiTheft);
					}
					if (statusAntiTheft == true)
					{
						getGPS();
						savedLocation.lat = currentLocation.lat;
						savedLocation.lng = currentLocation.lng;
					}
					if (dataForReceive.warning == false)
					{
						statusWarining = false;
						deviceStatus = NONE;
					}
					dataToSend.status = deviceStatus;
					updateData();
					// dataSubReciver = "";
					// dataSimReciver = "";
				}
				dataSubReciver = "";
				dataSimReciver = "";
				return true;
			}
		}
		if ((dataSimReciver.indexOf("+CMQPUB: 0,\"/noibinhyen/mcd/update\"") != -1) && checklengthDataReciver == 0)
		{
			splittedString = splitString(dataSimReciver, ',');
			if (splittedString.size() >= 6)
			{
				lengthDataReciver = splittedString[5].toInt();
			}
			Serial.println(dataSimReciver);
			if ((dataSimReciver.length() - dataSimReciver.indexOf("{")) < lengthDataReciver)
			{
				checklengthDataReciver = 1;
				doneGetData = 0;
			}
			else
			{
				doneGetData = 1;
				dataSubReciver = dataSimReciver.substring(dataSimReciver.indexOf("{"), dataSimReciver.indexOf("{") + lengthDataReciver);
				Serial.println(dataSubReciver);
				Serial.println(dataSimReciver);
				dataForReceive = DataForReceive::fromJson(dataSubReciver);
				if (dataForReceive.deviceId == deviceId)
				{
					if (statusAntiTheft != dataForReceive.antiTheft)
					{
						for (int i = 0; i < 6; i++)
						{
							statusBuzzer = !statusBuzzer;
							digitalWrite(BUZZER, statusBuzzer);
							delay(300);
						}
						statusAntiTheft = dataForReceive.antiTheft;
						Serial.print("Status Anti Theft = ");
						Serial.println(statusAntiTheft);
					}
					if (statusAntiTheft == true)
					{
						getGPS();
						//		savedLocation = currentLocation;
						savedLocation.lat = currentLocation.lat;
						savedLocation.lng = currentLocation.lng;
					}
					if (dataForReceive.warning == false)
					{
						statusWarining = false;
						deviceStatus = NONE;
					}

					dataToSend.status = deviceStatus;
					updateData();

					// dataSubReciver = "";
					// dataSimReciver = "";
				}
				dataSubReciver = "";
				dataSimReciver = "";
				return true;
			}
		}
		if (checklengthDataReciver == 0)
		{
			Serial.println(dataSimReciver);
			dataSimReciver = "";
		}
	}
	return false;
}

void initMQTT()
{
	String data;
	data = "AT+CMQNEW=\"" + MQTT_HOST + "\"" + ",\"" + String(MQTT_PORT) + "\",12000,1024\r\n";
	SimSerial.print(data);
	delay(100);
	data = "AT+CMQCON=0,3,\"gpsmqtt\",600,0,0\r\n";
	SimSerial.print(data);
	delay(2000);
	data = "AT+CMQSUB=0,\"" + SUB_UPDATE_TOPIC + "\",0\r\n";
	SimSerial.print(data);
	delay(1000);
}

bool checkSosAndAntiTheft(uint8_t sosPin, unsigned int timeOut)
{
	timeOut = 2000;
	if (digitalRead(sosPin) == HIGH) // sosPin = 1
	{
		int count = 1;
		int timePress = 3000; // 3s
		now = millis();
		while (1)
		{
			if ((millis() - now > timePress) && (digitalRead(sosPin) == HIGH))
			{
				Serial.println("SOS");
				deviceStatus = SOS;
				dataToSend.antiTheft = statusAntiTheft;
				dataToSend.status = deviceStatus; // SOS
				statusWarining = true;
				updateData();
				return true;
			}
			while (digitalRead(sosPin) == LOW) // neu nha
			{
				// count++;// count = 1
				if (millis() - now > timeOut)
				{
					return false;
				}
				if (digitalRead(sosPin) == HIGH) // neu bang 0
				{
					count++; // count = 2;
					now = millis();
					while (digitalRead(sosPin) == HIGH) // doi nha ra
					{
						if (millis() - now > timeOut)
						{
							return false;
						}
					}
					if (count >= 4)
					{
						// duoc bam
						updateData();
						Serial.print("AntiTheft ");
						statusAntiTheft = !statusAntiTheft;
						Serial.println(statusAntiTheft);
						for (int i = 0; i < 6; i++)
						{
							statusBuzzer = !statusBuzzer;
							digitalWrite(BUZZER, statusBuzzer);
							delay(300);
						}
						dataToSend.antiTheft = statusAntiTheft;
						now = millis();
						if (statusAntiTheft == true)
						{
							savedLocation.lat = currentLocation.lat;
							savedLocation.lng = currentLocation.lng;
						}
						count = 1;
						return true;
					}
				}
			}
			delay(10);
		}
	}
	return true;
}

void getGPS()
{
	char *gpsStream = new char[100];
	char c;
	const char NEED_GET_DATA_PREFIX = '$';
	const char NEED_STOP_GET_DATA_SUFFIX = '\n';
	bool needGetData = false;
	int count = 0;
	gpsStream[73] = '\0';
	for (int i = 0; i < 7; i++)
	{
		while (GpsSerial.available())
		{
			c = GpsSerial.read();
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
