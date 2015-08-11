#include <ESP8266WiFi.h>

const char* ssid     = "eXceed IOT";
const char* password = "1q2w3e4r";
const char* host = "exceed.cupco.de";
const int httpPort = 80;
const String group = "OAZeeD";

WiFiClient client;
String serialRecieveData = "";
String httpRecieveData = "";

bool hasSerialDataCome() {
	return Serial.available() > 0;
}

void waitForSerialData() {
	while (!hasSerialDataCome()) {
		delay(500);
	}
	Serial.flush();
}

bool isNotWifiConnect() {
	return WiFi.status() != WL_CONNECTED;
}

void waitForWiFiConnect() {
	while (isNotWifiConnect()) {
		delay(500);
                Serial.println("xxxx");
	}	
}

void waitForStart() {
	// waitForSerialData();
	waitForWiFiConnect();

}

void setup() {
	Serial.begin(115200);
	delay(100);
        Serial.println("xxxxxx");
	WiFi.begin(ssid, password);
	waitForStart();
        delay(100);
}

void httpRequest(String url) {
	client.setTimeout(15);
	client.connect(host, httpPort);
	client.print(String("GET ") + url + " HTTP/1.0\r\n" +
		"Host: " + host + "\r\n" +
		"Connection: close\r\n\r\n");
	delay(15);


}

bool isNotHeaderData(int countLine) {
	return countLine > 7;
}

String getEachLineDataHttp() {
	String line = client.readStringUntil('\r');	
	line.replace("\r", "");
	line.replace("\n", "");
	return line;
}

void setHttpRecieveData() {
	int countLine = 1;
	while (client.available()) {
		String line = getEachLineDataHttp();
		if (isNotHeaderData(countLine)) {
			httpRecieveData = line;
		}
		countLine++;
	}
}

void readHttpRequest() {
	String url = "/iot/" + group + "/toBoard";
	httpRequest(url);
	setHttpRecieveData();
}

void writeHttpRequest(String value) {
	String url = "/iot/" + group + "/fromBoard/" + value;
	httpRequest(url);
}

void setSerialRecieveData() {
	serialRecieveData = Serial.readStringUntil('\r');
	serialRecieveData.replace("\r", "");
	serialRecieveData.replace("\n", "");
}

void serialEvent() {
	if (hasSerialDataCome()) {
		setSerialRecieveData();
	}
}

void sendSerial(String value) {
	Serial.print(value);
	Serial.print('\r');
}

void recieveData() {
	serialEvent();
	readHttpRequest();
}

void sendData() {
	sendSerial(httpRecieveData);
	writeHttpRequest(serialRecieveData);
}

void loop() {
	recieveData();
	sendData();
//        Serial.println("xxx");
}