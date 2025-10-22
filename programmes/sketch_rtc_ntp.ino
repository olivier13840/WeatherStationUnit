#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "DHT.h"

#define DHTPIN 19        // Broche du signal du capteur AM2302 (DHT22)
#define DHTTYPE DHT22   // Type du capteur

const unsigned long periodeRepos = 1UL * 60UL * 1000UL;  // 2 min en ms
const unsigned long dureeAcquisition = 2UL * 60UL * 1000UL; // 1 min en ms
const unsigned long intervalLecture = 1UL * 1000UL; // lecture toutes les 1 s 

unsigned long debutAcquisition = 0;
unsigned long dernierCycle = 0;
bool enAcquisition = false;


// === CONFIG DHT ===
DHT dht(DHTPIN, DHTTYPE);

// === CONFIG WIFI ===
const char* ssid = "HONORX7b";
const char* password = "12345678";

float meanT, stdT;
float meanRH, stdRH; 
int index;

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
}

void loop() {
  unsigned long maintenant = millis();

  // Démarrage d'un nouveau cycle d'acquisition
  if (!enAcquisition && (maintenant - dernierCycle >= periodeRepos)) {
    enAcquisition = true;
    debutAcquisition = maintenant;
    dernierCycle = maintenant;
    Serial.println("=== Début acquisition ===");
    meanT=0.;
    meanRH=0.;
    stdT=0.;
    stdRH=0.;
    index=0;
  }

  // Si on est dans la fenêtre d'acquisition
  if (enAcquisition) {
    if (maintenant - debutAcquisition < dureeAcquisition) {
      static unsigned long dernierSample = 0;
      if (maintenant - dernierSample >= intervalLecture) {
        dernierSample = maintenant;

        float h = dht.readHumidity();
        float t = dht.readTemperature();

        if (!isnan(h) && !isnan(t)) {
          Serial.printf("Mesure: T=%.1f°C, RH=%.1f%%\n", t, h);
          meanT=meanT+t;
          meanRH=meanRH+h;
          index++
        }
      }
    } else {
      // Fin d’acquisition : envoi des données
      Serial.println("=== Fin acquisition, calcul et envoi des données ===");
      meanT=meanT/index;
      meanRH=meanRH/index;
      envoyerDonnees();
      enAcquisition = false;
    }
  }
 
 
void envoyerDonnees() {
  DateTime now = rtc.now();

  // Affichage heure
  Serial.print("Heure RTC : ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  // if (WiFi.status() == WL_CONNECTED) {
  //   HTTPClient http;
  //   http.begin("http://ton_serveur.local/api/mesures");
  //   http.addHeader("Content-Type", "application/json");

  //   String payload = "{\"message\":\"Cycle terminé\"}";
  //   int code = http.POST(payload);
  //   Serial.printf("Code HTTP : %d\n", code);
  //   http.end();
  //}
}
 

 
 
  

  // Lecture de l'humidité relative
  float humidity = dht.readHumidity();
  // Lecture de la température en °C
  float temperature = dht.readTemperature();

  // Vérification d’erreur de lecture
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Erreur de lecture du capteur DHT22 !"));
    return;
  }

  Serial.print(F("Humidité : "));
  Serial.print(humidity);
  Serial.print(F(" %  |  Température : "));
  Serial.print(temperature);
  Serial.println(F(" °C"));

  delay(5000);
}

