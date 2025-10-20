#include "esp_system.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <WiFi.h>
#include <NTPClient.h>
//#include <WiFiUdp.h>
#include <SensirionI2cSps30.h>
#include <Wire.h>
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define DS18B20PIN 4
#define DHTPIN 19      // GPIO DATA AMT2303
#define DHTTYPE DHT22 // AM2302 = DHT22

// ===== CONFIG WIFI =====
const char* ssid = "Rognes";
const char* password = "0fa26bc7a2";

// Paramètres NTP
const char* ntpServer = "ntp.ec-m.fr";    // serveur NTP
const long  gmtOffset_sec = 3600;          // décalage GMT+1 (Europe centrale)
const int   daylightOffset_sec = 3600;     // heure d'été (+1h)
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "ntp.ec-m.fr", 3600, 60000); 

SensirionI2cSps30 sensor;
DHT dht(DHTPIN, DHTTYPE);

OneWire oneWire(DS18B20PIN);
DallasTemperature sensors(&oneWire);

static char errorMessage[64];
static int16_t error;

extern "C" {
  uint8_t temprature_sens_read();  // fonction interne ESP-IDF
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
        delay(100);
    }
  //setup_wifi();
  WiFi.begin(ssid, password);
  Serial.print("Connexion au Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());

  //Config NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Init AM2302-DHT22
  dht.begin();
  Serial.println("Init Capteur AM2302 (DHT22)");

  // Init DS18B20
  sensors.begin();  // Initialisation du DS18B20 sensor.stopMeasurement();
  Serial.println("Init Capteur DS18B20");

  // Init I2C + capteur SPS30
  Wire.begin();
  sensor.begin(Wire, SPS30_I2C_ADDR_69);
  int8_t serialNumber[32] = {0};
  int8_t productType[8] = {0};
  sensor.readSerialNumber(serialNumber, 32);
  Serial.print("serialNumber: ");
  Serial.print((const char*)serialNumber);
  Serial.println();
  sensor.readProductType(productType, 8);
  Serial.print("productType: ");
  Serial.print((const char*)productType);
  Serial.println();
  sensor.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16);
  delay(100);
}

void loop() {
  //recup date/heure
  struct tm now = getLocalTimeStruct();
  if (now.tm_year+1900 > 2023) {
    Serial.printf("%02d/%02d/%04d %02d:%02d:%02d\n",
                  now.tm_mday, now.tm_mon + 1, now.tm_year + 1900,
                  now.tm_hour, now.tm_min, now.tm_sec);
  }
  else {
    Serial.println("Heure 00:00:00");
  }
  
  // Température CPU ESP32 en °C
  uint8_t temp_raw = temprature_sens_read();
  int temp = (int) temp_raw;  // converti en entier
  Serial.print("Température CPU: ");
  Serial.print(temp);
  Serial.println(" °C");

  // lecture humidité et température AM2302-DHT22
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Erreur lecture DHT22 !");
  } else {
    Serial.print("Humidité: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Température DHT: ");
    Serial.print(t);
    Serial.println(" °C");
  }
  //lecture temp DS18B20
  sensors.requestTemperatures();
  float tempDS18B20C = sensors.getTempCByIndex(0);
  //float tempDS18B20F = sensors.getTempFByIndex(0);
  Serial.print("Température DS: ");
  Serial.print(tempDS18B20C);
  Serial.println(" °C");

  //lecture SPS30
  uint16_t dataReadyFlag = 0;
  uint16_t mc1p0 = 0;
  uint16_t mc2p5 = 0;
  uint16_t mc4p0 = 0;
  uint16_t mc10p0 = 0;
  uint16_t nc0p5 = 0;
  uint16_t nc1p0 = 0;
  uint16_t nc2p5 = 0;
  uint16_t nc4p0 = 0;
  uint16_t nc10p0 = 0;
  uint16_t typicalParticleSize = 0;
  
  error = sensor.readDataReadyFlag(dataReadyFlag);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute readDataReadyFlag(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
  Serial.print("dataReadyFlag: ");
  Serial.print(dataReadyFlag);
  Serial.println();
  error = sensor.readMeasurementValuesUint16(mc1p0, mc2p5, mc4p0, mc10p0,
                                               nc0p5, nc1p0, nc2p5, nc4p0,
                                               nc10p0, typicalParticleSize);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute readMeasurementValuesUint16(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
  Serial.print("mc1p0: ");
  Serial.print(mc1p0);
  Serial.print("\t");
  Serial.print("mc2p5: ");
  Serial.print(mc2p5);
  Serial.print("\t");
  Serial.print("mc4p0: ");
  Serial.print(mc4p0);
  Serial.print("\t");
  Serial.print("mc10p0: ");
  Serial.print(mc10p0);
  Serial.print("\t");
  Serial.print("nc0p5: ");
  Serial.print(nc0p5);
  Serial.print("\t");
  Serial.print("nc1p0: ");
  Serial.print(nc1p0);
  Serial.print("\t");
  Serial.print("nc2p5: ");
  Serial.print(nc2p5);
  Serial.print("\t");
  Serial.print("nc4p0: ");
  Serial.print(nc4p0);
  Serial.print("\t");
  Serial.print("nc10p0: ");
  Serial.print(nc10p0);
  Serial.print("\t");
  Serial.print("typicalParticleSize: ");
  Serial.print(typicalParticleSize);
  Serial.println();

  delay(60000);
}

struct tm getLocalTimeStruct() {
  struct tm timeinfo;
  uint32_t timeout_ms = 15000;
  uint32_t start=millis();
  while (millis()-start < timeout_ms){
    if (!getLocalTime(&timeinfo,1000)) return timeinfo;
    Serial.println("Attente synchro NTP");
    delay(250);
  }
Serial.println("Échec de récupération de l'heure");
memset(&timeinfo, 0, sizeof(struct tm));
return timeinfo;
}
