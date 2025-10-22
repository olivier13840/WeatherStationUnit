#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "DHT.h"
#include <math.h>
#include <SensirionI2cSps30.h>

// macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define DHTPIN 19        // Broche du signal du capteur AM2302 (DHT22)
#define DHTTYPE DHT22   // Type du capteur

const unsigned long dureeAcquisition = 2UL * 60UL * 1000UL; // 2 min en ms
const unsigned long intervalLecture = 1UL * 1000UL; // lecture toutes les 1 s 
int lastMinute = -1;

// === CONFIG SPS30 ===
SensirionI2cSps30 sensor;


// === CONFIG DHT ===
DHT dht(DHTPIN, DHTTYPE);

// === CONFIG WIFI ===
const char* ssid = "HONORX7b";
const char* password = "12345678";

double meanT = 0.0;
double stdT= 0.0;
double BufferT[400];
double meanRH= 0.0;
double stdRH= 0.0;
double BufferRH[400]; 
int indexcount;

DateTime t;
int minute;
int heure;

static char errorMessage[64];
static int16_t error;

// === CONFIG NTP ===
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.ec-m.fr", 3600, 60000); // UTC+1

// === RTC DS3231 ===
RTC_DS3231 rtc;


void setup() {
  Serial.begin(115200);
  while (!Serial) {
        delay(100);
    }

  // Connexion au Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connexion au WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnecté au WiFi");

  // Démarrage client NTP
  timeClient.begin();

  // Attente de l'heure NTP
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  // Extraction des infos de temps
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);

  // Initialisation RTC avec l'heure NTP
  if (!rtc.begin()) {
    Serial.println("Erreur : RTC non détecté !");
    while (1);
  }
  rtc.adjust(DateTime(ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
                      ptm->tm_hour, ptm->tm_min, ptm->tm_sec));

  Serial.println("RTC mis à l'heure via NTP");
  
  // === Init du DHT AMD2302
  dht.begin();

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
}
// === Boucle principale ===
void loop() {
  t = rtc.now();
  minute = t.minute();
  heure = t.hour();

  Serial.printf("Heure actuelle : %02d:%02d:%02d\n", heure, minute, t.second());

  // Vérifie si on est à xx:00 ou xx:30
  if ((minute == 0 || minute == 30) && lastMinute != minute) {
    Serial.println("⏰ Début d’acquisition !");
    Acquisition();

    lastMinute = minute;
  }
 
}
 
void Acquisition() {
  Serial.println("=== Début acquisition ===");
  meanT=0.;
  meanRH=0.;
  stdT=0.;
  stdRH=0.;
  indexcount =0;
  for (int i=0;i<400;i++) {
    BufferT[i]=0.0;
    BufferRH[i]=0.0;
  }
  unsigned long maintenant = millis();
  unsigned long debutAcquisition = maintenant;

  // Si on est dans la fenêtre d'acquisition
  while (maintenant - debutAcquisition < dureeAcquisition) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      Serial.printf("Mesure: T=%.1f°C, RH=%.1f%%\n", t, h);
      meanT+=t;
      meanRH+=h;
      BufferT[indexcount]=t;
      BufferRH[indexcount]=h;
      indexcount++;
      delay(1000);
    }
    maintenant = millis();
  } 
  // Fin d’acquisition : envoi des données
  Serial.println("=== Fin acquisition, calcul et envoi des données ===");
  meanT=meanT/(double)indexcount;
  meanRH=meanRH/(double)indexcount;
  for (int i=0;i<indexcount;i++) {
    stdT+=(meanT-BufferT[i])*(meanT-BufferT[i]);
    stdRH+=(meanRH-BufferRH[i])*(meanRH-BufferRH[i]);
  }
  stdT=sqrt(stdT/double(indexcount));
  stdRH=sqrt(stdRH/double(indexcount));
  envoyerDonnees();
} 

void envoyerDonnees() {
  Serial.printf("TimeStamp : %02d:%02d:%02d\n", heure, minute, t.second());
  Serial.print(F("Humidité MEAN: "));
  Serial.print(meanRH);
  Serial.print(F(" % | Humidité STD: "));
  Serial.print(stdRH);
  Serial.print(F(" %  |  Température MEAN: "));
  Serial.print(meanT);
  Serial.print(F(" °C |  Température STD: "));
  Serial.print(stdT);
  Serial.println(F(" °C"));

}
 

 
 