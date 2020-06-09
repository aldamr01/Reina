#include <WiFi.h>               // ESP8266 Library
#include <PubSubClient.h>       // MQTT Communication
#include <ArduinoJson.h>        // Parsing MQTT
#include <OneWire.h>            // For Sensor DS18B20
#include <DallasTemperature.h>  // For Sensor DS18B20
#include "DHT.h"                // For Sensor DHT
#include <LiquidCrystal_I2C.h>  // For LCD
#include <EEPROM.h>             // EEPROM
#include <HTTPClient.h>         // HTTP Communication


define DHTTYPE DHT22            // Define DHT Type as DHT22
define PHPIN 34                 // Define Pin 34 as pH Sensor Pin
define TDSPIN 35                // Define Pin 35 as TDS Sensor Pin
define VREF 5.0                 // Define analog reference voltage of the ADC for TDS Sensor
define SCOUNT 30                // Define sum of sample point for TDS Sensor

const int pin_relay_nutrisi_a   = 2;    // Define Pin 2 as Dosing Pump 1 Pin
const int pin_relay_nutrisi_b   = 15;   // Define Pin 15 as Dosing Pump 2 Pin
const int pin_relay_air         = 4;    // Define Pin 4 as Water Pump Pin
const int pin_suhu_air          = 32;   // Define Pin 32 as DS18B20(Water Sensor) Pin
const int pin_suhu_kelembaban   = 17;   // Define Pin 17 as DHT22(Humidity&Air Tempt) Pin
const int pin_button_green      = 19;   // Define Pin 19 as Green Button(Mixer Nutrient) Pin
const int pin_button_blue       = 18;   // Define Pin 18 as Blue Button(Restart Device) pin
const int ledPin                = 5;    // Define Pin 5 as LED pin

String serverName           = "https://dahlia.my.id/api/v1/";   // API Endpoint
const char * mqttServer     = "sibadaring.com";                 // MQTT Server Domain
const char * ssid           = "cyberpunk";                      // SSID WiFi
const char * password       = "sample0601";                     // PASSWORD WiFi
const char * deviceID       = "DS11X900";                       // Device Name
const char * devicePasword  = "";                               // Unused Variable
const char * deviceUsername = "";                               // Unused Variable
const int mqttPort          = 1883;                             // MQTT Server Port

char message_buff[900];         // Declare Variable for store message from MQTT
int analogBuffer[SCOUNT];       // Declare Variable for store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];   // Declare Temporary Variable for store the analog value in the array, read from ADC
int analogBufferIndex   = 0;    // Initiate analog buffer at 0
int copyIndex           = 0;    // initiate index at 0
float averageVoltage    = 0;    // initiate avg voltage at 0
float tdsValue          = 0;    // initiate TDS Sensor Value at 0
float temperature       = 25;   // initiate temperature for calibrate TDS Sensor & pH at 25


WiFiClient espClient;                   // Initiate ESP WiFi Client
PubSubClient client(espClient);         // Initiate MQTT Client
OneWire oneWire(pin_suhu_air);          // Initiate DS18B20
DallasTemperature sensors( & oneWire);  // Initiate DS18B20
DHT dht(pin_suhu_kelembaban, DHTTYPE);  // Initiate DHT22
LiquidCrystal_I2C lcd(0x27, 16, 2);     // Initiate LCD 16x2

/*
* MQTT Callback Function
* Receive any message from mqtt server with topic 'reina/input'
*/
void callback(char * topic, byte * payload, unsigned int length) {
    String messageTemp;
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
        messageTemp += (char) payload[i];
    }

    StaticJsonDocument < 256 > dataDevice;
    auto error = deserializeJson(dataDevice, messageTemp);
    Serial.println(messageTemp);
    if (error) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(error.c_str());
        return;
    }

    Serial.println();
    Serial.println("-----------------------");
    if (String(topic) == "reina/input") {
        Serial.print("Changing output to ");
        if (dataDevice["mixin"] == "on") {
            controlStage1();
            controlStage2();
            controlStage3();
            controlStage4();
        }
    }
}

/*
* Nutrient A Mixer Function
*/
void controlStage1() {
    HTTPClient http;
    String serverPath = serverName + "setdatacontrol1/";
    http.begin(serverPath.c_str());
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    http.end();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("==== MIXING ====");
    lcd.setCursor(0, 1);
    lcd.print("Nutrient A = ON");
    digitalWrite(pin_relay_nutrisi_a, HIGH);
    Serial.println("pin n a menyala");
    delay(15000);
    digitalWrite(pin_relay_nutrisi_a, LOW);
    Serial.println("pin n a mati");
}

/*
* Nutrient B Mixer Function
*/
void controlStage2() {
    HTTPClient http;
    String serverPath = serverName + "setdatacontrol2/";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("==== MIXING ====");
    lcd.setCursor(0, 1);
    lcd.print("Nutrient B = ON");
    digitalWrite(pin_relay_nutrisi_b, HIGH);
    Serial.println("pin n b menyala");
    //  delay(35000);
    delay(15000); //15 detik
    digitalWrite(pin_relay_nutrisi_b, LOW);
    Serial.println("pin n b mati");
}

/*
* Water Mixer Function
*/
void controlStage3() {
    HTTPClient http;
    String serverPath = serverName + "setdatacontrol3/";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("==== MIXING ====");
    lcd.setCursor(0, 1);
    lcd.print("Water M. = ON");
    digitalWrite(pin_relay_air, HIGH);
    Serial.println("pin air menyala");
    //  delay(133000);
    delay(55000); //2 liter
    digitalWrite(pin_relay_air, LOW);
    Serial.println("pin air mati");
}

/*
* WebServer Notification Function
*/
void controlStage4() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("==== MIXING ====");
    lcd.setCursor(0, 1);
    lcd.print("COMPLETED 100%");
    delay(2000);
    digitalWrite(pin_relay_nutrisi_a, LOW);
    digitalWrite(pin_relay_nutrisi_b, LOW);
    digitalWrite(pin_relay_air, LOW);
    Serial.println("pin semua mati");
    HTTPClient http;
    String serverPath = serverName + "setdatacontrol4/";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
}

/*
* TDS Sensor Calibration Function
*/
int getMedianNum(int bArray[], int iFilterLen) {
    int bTab[iFilterLen];
    for (byte i = 0; i < iFilterLen; i++)
        bTab[i] = bArray[i];
    int i, j, bTemp;
    for (j = 0; j < iFilterLen - 1; j++) {
        for (i = 0; i < iFilterLen - j - 1; i++) {
            if (bTab[i] > bTab[i + 1]) {
                bTemp = bTab[i];
                bTab[i] = bTab[i + 1];
                bTab[i + 1] = bTemp;
            }
        }
    }
    if ((iFilterLen & 1) > 0)
        bTemp = bTab[(iFilterLen - 1) / 2];
    else
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    return bTemp;
}

/*
* Setup Function
*/
void setup() {
    sensors.begin();
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    dht.begin();
    lcd.init();
    lcd.backlight();
    pinMode(TDSPIN, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(pin_relay_air, OUTPUT);
    pinMode(pin_relay_nutrisi_a, OUTPUT);
    pinMode(pin_relay_nutrisi_b, OUTPUT);
    pinMode(pin_button_green, INPUT_PULLUP);
    pinMode(pin_button_blue, INPUT_PULLUP);
    lcd.setCursor(0, 0);
    lcd.print("Initializing : ");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
        lcd.setCursor(0, 1);
        lcd.print("Connecting to WiFi..");
        delay(1000);
    }
    Serial.println("Connected to the WiFi network");
    lcd.setCursor(0, 1);
    lcd.print("WiFi Connected..");
    delay(1000);

    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        lcd.setCursor(0, 1);
        lcd.print("Connecting MQTT..");
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
            lcd.setCursor(0, 1);
            lcd.print("MQTT Connected..");
            delay(1000);
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            lcd.setCursor(0, 1);
            lcd.print("MQTT FailConnect");
            delay(2000);
        }
    }
    lcd.clear();
    client.subscribe("reina/input");
}

/*
* Main Logic
*/
void loop() {
    client.loop(); 
    sensors.requestTemperatures();
    lcd.setCursor(0, 0);

    static unsigned long analogSampleTimepoint  = millis();
    float suhu_lingkungan                       = dht.readTemperature();
    float kelembaban_lingkungan                 = dht.readHumidity();
    float suhu_air                              = sensors.getTempCByIndex(0);
    float nutrisi_air                           = 0;
    float ph                                    = 0;

    if (millis() - analogSampleTimepoint > 40 U) { //every 40 milliseconds,read the analog value from the ADC
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TDSPIN); //read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
            analogBufferIndex = 0;
    }

    static unsigned long printTimepoint = millis();

    if (millis() - printTimepoint > 800 U) {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float) VREF / 4096.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation

        // Calculate TDS Value
        tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
        tdsValue = tdsValue - 200;

        // Calculate pH Value
        int nilaiPengukuranPh = analogRead(PHPIN);
        double TeganganPh = 5 / 4095.0 * nilaiPengukuranPh;
        ph = 7.00 + ((3.593 - TeganganPh) / 0.381);

        // Debug to Serial Print
        Serial.print("TDS Value 2 = ");
        Serial.print(tdsValue, 0);
        Serial.println("ppm");
        Serial.print("TDS Value = ");
        Serial.print(analogRead(TDSPIN));
        Serial.println("ppm");
        Serial.print("pH = ");
        Serial.println(ph, 3);
        Serial.print("Suhu Lingkungan = ");
        Serial.println(suhu_lingkungan);
        Serial.print("Kelembaban Lingkungan = ");
        Serial.println(kelembaban_lingkungan);
        Serial.print("Suhu Air   = ");
        Serial.println(suhu_air);
        String tds_hasil = "TDS = " + String(tdsValue) + "ppm";
        String ph_hasil = "PH = " + String(ph);
        String humidity_hasil = "Humid = " + String(suhu_lingkungan) + "%";
        String suhu_udara_hasil = "Air Temp = " + String(suhu_lingkungan) + "C";
        String suhu_air_hasil = "Wtr Temp = " + String(suhu_air) + "C";

        // Displaying to LCD
        lcd.setCursor(0, 0);
        lcd.print(tds_hasil);
        lcd.setCursor(0, 1);
        lcd.print(ph_hasil);
        delay(1000);
        lcd.setCursor(0, 0);
        lcd.print(humidity_hasil);
        lcd.setCursor(0, 1);
        lcd.print(suhu_udara_hasil);
        delay(1000);
        lcd.setCursor(0, 0);
        lcd.print(suhu_air_hasil);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        delay(1000);
        lcd.clear();
    }

    String V_a_tempt = "";
    String V_humid = "";
    String V_w_tempt = String(suhu_air);
    String V_tds = String(tdsValue);
    String V_ph = String(ph);

    if (isnan(suhu_lingkungan)) {
        V_a_tempt = "0";
    } else {
        V_a_tempt = String(suhu_lingkungan);
    }

    if (isnan(kelembaban_lingkungan)) {
        V_humid = "0";
    } else {
        V_humid = String(kelembaban_lingkungan);
    }

    // Status Set Controller to Online or Offline
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("=======WIFI OFF=======");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("==OFFLINE MODE==");
        lcd.setCursor(0, 1);
        lcd.print(" ");

        if (digitalRead(pin_button_blue) == HIGH) {
            Serial.println("button blue OK");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Rebooting Device");
            ESP.restart();
        }
        if (digitalRead(pin_button_green) == HIGH) {
            Serial.println("button green OK");
            controlStage1();
            controlStage2();
            controlStage3();
            controlStage4();
        }
    } else {
        Serial.println("=======WIFI ON=======");

    }
    // Send Data Sensor to WebServer
    String pubString = "{'humid' :" + V_humid + ",'w_tempt':" + V_w_tempt + ",'a_tempt':" + V_a_tempt + ",'ph':" + V_ph + ",'tds':" + V_tds + "}";
    Serial.println(pubString);
    Serial.println(pubString.length());
    pubString.toCharArray(message_buff, pubString.length() + 1);
    client.publish("reina/sensorvalue", message_buff);
}