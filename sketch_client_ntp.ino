#include <WiFi.h>
#include "time.h"

const char* ssid     = "SgOb24";
const char* password = "azqswx63";

// Paramètres NTP
const char* ntpServer = "ntp.ec-m.fr";    // serveur NTP
const long  gmtOffset_sec = 3600;          // décalage GMT+1 (Europe centrale)
const int   daylightOffset_sec = 3600;     // heure d'été (+1h)

// Fonction d’attente pour la synchro NTP
bool waitForTime(uint32_t timeout_ms = 10000) {
  struct tm timeinfo;
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (getLocalTime(&timeinfo)) {
      if (timeinfo.tm_year >= (2023 - 1900)) return true; // valide
    }
    delay(500);
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  // Connexion WiFi
  Serial.printf("Connexion à %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✔ WiFi connecté");
  Serial.print("IP locale : ");
  Serial.println(WiFi.localIP());

  // Configure NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Attend la synchro
  if (waitForTime()) {
    Serial.println("✔ NTP synchro OK");
  } else {
    Serial.println("⛔ Timeout NTP (pas d'heure valide)");
  }
}

void loop() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.printf("Date/Heure : %02d/%02d/%04d %02d:%02d:%02d\n",
                  timeinfo.tm_mday,
                  timeinfo.tm_mon + 1,
                  timeinfo.tm_year + 1900,
                  timeinfo.tm_hour,
                  timeinfo.tm_min,
                  timeinfo.tm_sec);
  } else {
    Serial.println("⛔ Impossible de lire l'heure");
  }
  delay(1000);
}

