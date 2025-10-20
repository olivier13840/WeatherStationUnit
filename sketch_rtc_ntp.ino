#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// === CONFIG WIFI ===
const char* ssid = "SgOb24";
const char* password = "azqswx63";

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
}

void loop() {
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

  delay(5000);
}

