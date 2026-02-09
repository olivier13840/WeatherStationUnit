#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <HTTPClient.h> 
#include "DHT.h"
#include <math.h>
#include <SensirionI2cSps30.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "FS.h"
#include <LittleFS.h>
#include <vector>

// macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define DHTPIN 19
#define DHTTYPE DHT22

// ======== CONFIG INFLUXDB ========
#define INFLUXDB_URL "InfluxDB-API"
#define INFLUXDB_TOKEN "TOKEN"
#define INFLUXDB_ORG "ORG"
#define INFLUXDB_BUCKET "BUCKET"

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point weather("weather");

SensirionI2cSps30 sensor;
DHT dht(DHTPIN, DHTTYPE);
const char* ssid = "WIFI-SSID";
const char* password = "PASSWORD";

// Global Variables
double meanT = 0.0, stdT = 0.0, meanRH = 0.0, stdRH = 0.0;
double BufferT[400], BufferRH[400];
int indexcount;
uint16_t mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typicalParticleSize;

RTC_DS3231 rtc;

struct WeatherRecord {
  unsigned long timestamp; 
  double mT, sT, mRH, sRH;
  uint16_t m1, m25, m4, m10, n05, n1, n25, n4, n10, tps;
};

std::vector<WeatherRecord> dataBuffer;
const char* STORAGE_FILENAME = "/backlog.bin";

// --- FLASH HELPERS ---
void saveToFlash(WeatherRecord& rec) {
  File file = LittleFS.open(STORAGE_FILENAME, "a");
  if (file) { 
    file.write((uint8_t*)&rec, sizeof(WeatherRecord)); 
    file.close(); 
  }
}

void loadFromFlash() {
  if (!LittleFS.exists(STORAGE_FILENAME)) return;
  File file = LittleFS.open(STORAGE_FILENAME, "r");
  if (!file) return;
  dataBuffer.clear();
  while (file.available()) {
    WeatherRecord rec;
    if (file.read((uint8_t*)&rec, sizeof(WeatherRecord)) == sizeof(WeatherRecord)) {
      dataBuffer.push_back(rec);
    }
  }
  file.close();
}

void syncTimeHTTP() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin("http://www.google.com"); 
  const char* headerKeys[] = {"Date"};
  http.collectHeaders(headerKeys, 1);
  int httpCode = http.GET();
  if (httpCode > 0) {
    String date = http.header("Date"); 
    int day = date.substring(5, 7).toInt();
    String monthStr = date.substring(8, 11);
    int year = date.substring(12, 16).toInt();
    int hr = date.substring(17, 19).toInt();
    int min = date.substring(20, 22).toInt();
    int sec = date.substring(23, 25).toInt();
    int month = 1;
    String months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i=0; i<12; i++) { if (monthStr == months[i]) month = i + 1; }
    if (year >= 2025) rtc.adjust(DateTime(year, month, day, hr, min, sec));
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  if (!LittleFS.begin(true)) Serial.println("LittleFS Fail");
  loadFromFlash();

  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
    delay(500); Serial.print(".");
  }

  Wire.begin(21, 22, 100000);
  if (!rtc.begin()) { Serial.println("RTC Fail!"); while (1); }
  if (WiFi.status() == WL_CONNECTED) syncTimeHTTP();

  dht.begin();
  Wire1.begin(17, 16, 100000);
  sensor.begin(Wire1, SPS30_I2C_ADDR_69);
  sensor.stopMeasurement();
  
  weather.addTag("device", "WS1");
  weather.addTag("location", "Fablab");
}

void loop() {
  Acquisition();
  processQueue();
}

void Acquisition() {
  DateTime now = rtc.now();
  
  // Calculate seconds until the exact start of the next hour (HH:00:00)
  int minutesToNextHour = 60 - now.minute();
  int secondsToNextHour = (minutesToNextHour * 60) - now.second();
  
  // If at the top of an hour, target the next one
  if (secondsToNextHour <= 0) secondsToNextHour = 3600;

  // Add 2-second buffer to ensure cycle includes the top-of-hour measurement
  unsigned long totalCycleMs = (unsigned long)(secondsToNextHour + 2) * 1000UL;
  unsigned long cycleStartMs = millis();

  Serial.printf("\n=== Début cycle Horaire (Synchro : %ds restantes) ===\n", secondsToNextHour);
  
  uint16_t mc1p0R, mc2p5R, mc4p0R, mc10p0R, nc0p5R, nc1p0R, nc2p5R, nc4p0R, nc10p0R, typicalParticleSizeR;
  uint16_t dataReadyFlag = 0;

  meanT=0.; meanRH=0.; stdT=0.; stdRH=0.;
  mc1p0=0; mc2p5=0; mc4p0=0; mc10p0=0; nc0p5=0; nc1p0=0; nc2p5=0; nc4p0=0; nc10p0=0; typicalParticleSize=0;
  indexcount = 0;

  sensor.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16);

  while (millis() - cycleStartMs < totalCycleMs) {
    float h = dht.readHumidity();
    float temp = dht.readTemperature();
    
    if (!isnan(h) && !isnan(temp)) {
      Serial.printf("[%02d:%02d:%02d UTC] Live Measure: Temp=%.1f C, Hum=%.1f%%\n", rtc.now().hour(), rtc.now().minute(), rtc.now().second(), temp, h);
      meanT += temp; meanRH += h;
      BufferT[indexcount] = temp; BufferRH[indexcount] = h;
    }
    
    sensor.readDataReadyFlag(dataReadyFlag);
    if (sensor.readMeasurementValuesUint16(mc1p0R, mc2p5R, mc4p0R, mc10p0R, nc0p5R, nc1p0R, nc2p5R, nc4p0R, nc10p0R, typicalParticleSizeR) == NO_ERROR) {
      mc1p0 += mc1p0R; mc2p5 += mc2p5R; mc4p0 += mc4p0R; mc10p0 += mc10p0R;
      nc0p5 += nc0p5R; nc1p0 += nc1p0R; nc2p5 += nc2p5R; nc4p0 += nc4p0R;
      nc10p0 += nc10p0R; typicalParticleSize += typicalParticleSizeR;
    }
    indexcount++;
    
    // Smart delay to target the next :00 second mark
    DateTime timeCheck = rtc.now();
    int waitSeconds = 60 - timeCheck.second();
    
    // Exit loop if next wait goes beyond cycle limit
    if ((millis() - cycleStartMs) + (waitSeconds * 1000UL) > totalCycleMs) break;

    delay(waitSeconds * 1000UL); 
  }
  sensor.stopMeasurement();

  if (indexcount > 0) {
    meanT /= (double)indexcount; meanRH /= (double)indexcount;
    for (int i=0; i<indexcount; i++) {
      stdT += pow(meanT - BufferT[i], 2);
      stdRH += pow(meanRH - BufferRH[i], 2);
    }
    stdT = sqrt(stdT / (double)indexcount); 
    stdRH = sqrt(stdRH / (double)indexcount);
    
    mc1p0/=indexcount; mc2p5/=indexcount; mc4p0/=indexcount; mc10p0/=indexcount;
    nc0p5/=indexcount; nc1p0/=indexcount; nc2p5/=indexcount; nc4p0/=indexcount;
    nc10p0/=indexcount; typicalParticleSize/=indexcount;

    DateTime topOfHour = rtc.now();
    WeatherRecord rec = { (unsigned long)topOfHour.unixtime(), meanT, stdT, meanRH, stdRH, mc1p0, mc2p5, mc4p0, mc10p0, nc0p5, nc1p0, nc2p5, nc4p0, nc10p0, typicalParticleSize };
    
    dataBuffer.push_back(rec);
    saveToFlash(rec);
  }
}

void processQueue() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n>>> WiFi Offline. Mesures sauvegardées en Flash.");
    return;
  }

  while (!dataBuffer.empty()) {
    WeatherRecord& rec = dataBuffer.front();
    weather.clearFields();
    
    weather.setTime(WritePrecision::S); 
    weather.setTime((unsigned long long)rec.timestamp); 
    
    weather.addField("Temp", rec.mT);
    weather.addField("Temp_Std", rec.sT); 
    weather.addField("RH", rec.mRH);
    weather.addField("RH_Std", rec.sRH);   
    weather.addField("mc_pm1p0", rec.m1);
    weather.addField("mc_pm25", rec.m25);
    weather.addField("mc_pm40", rec.m4);
    weather.addField("mc_pm100", rec.m10);
    weather.addField("nc_pm05", rec.n05);
    weather.addField("nc_pm10", rec.n1);
    weather.addField("nc_pm25", rec.n25);
    weather.addField("nc_pm40", rec.n4);
    weather.addField("nc_pm100", rec.n10);
    weather.addField("typical_size", rec.tps);

    Serial.println("\n--- Envoi vers InfluxDB (Line Protocol) ---");
    Serial.println(weather.toLineProtocol()); 
    
    DateTime dt = DateTime(rec.timestamp);
    Serial.printf("Log Time: %02d:%02d:%02d | T_avg=%.2f, RH_avg=%.2f\n", dt.hour(), dt.minute(), dt.second(), rec.mT, rec.mRH);

    if (client.writePoint(weather)) {
      Serial.println(">>> Transmission OK.");
      dataBuffer.erase(dataBuffer.begin());
    } else {
      Serial.printf(">>> Erreur Influx: %s\n", client.getLastErrorMessage().c_str());
      break; 
    }
  }

  if (dataBuffer.empty() && LittleFS.exists(STORAGE_FILENAME)) {
    LittleFS.remove(STORAGE_FILENAME);
  }
}