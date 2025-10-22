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

const unsigned long dureeAcquisition = 5UL * 60UL * 1000UL; // 2 min en ms
int lastMinute = -1;

// === CONFIG SPS30 ===
SensirionI2cSps30 sensor;


// === CONFIG DHT ===
DHT dht(DHTPIN, DHTTYPE);

// === CONFIG WIFI ===
const char* ssid = "labidi": 
const char* password = "labidilabidi";

double meanT = 0.0;
double stdT= 0.0;
double BufferT[400];
double meanRH= 0.0;
double stdRH= 0.0;
double BufferRH[400]; 
int indexcount;
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

DateTime t;
int minute;
int heure;

static char errorMessage[64];
static int16_t error;

// === CONFIG NTP ===
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.unice.fr", 3600, 60000); // UTC+1

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
    Serial.println("\nAttente synchro NTP..");
    timeClient.forceUpdate();
  }

  // Extraction des infos de temps
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);

  // Initialisation RTC avec l'heure NTP
   //Init protocole I2C
  Wire.begin(21, 22,100000);  // RTC sur SDA=21, SCL=22
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
  Wire1.begin(17, 16,100000);
  sensor.begin(Wire1, SPS30_I2C_ADDR_69);
  sensor.stopMeasurement();
  while (Wire.available()) Wire.read();
  delay(5000);
}
// === Boucle principale ===
void loop() {
  t = rtc.now();  //recupere l'heure
  minute = t.minute();
  heure = t.hour();

  Serial.printf("Heure actuelle : %02d:%02d:%02d\n", heure, minute, t.second());
  delay(2000);

  // Vérifie si on est à xx:00 ou xx:30
  if ((minute == 0 || minute == 30) && lastMinute != minute) {
    Serial.println("⏰ Début d’acquisition !");
    Acquisition();
    lastMinute = minute;
  }
}
 
void Acquisition() {     //fonction qui recupere les donnees sur les capteurs
  Serial.println("=== Début acquisition ===");
  uint16_t mc1p0R;
  uint16_t mc2p5R;
  uint16_t mc4p0R;
  uint16_t mc10p0R;
  uint16_t nc0p5R;
  uint16_t nc1p0R;
  uint16_t nc2p5R;
  uint16_t nc4p0R;
  uint16_t nc10p0R;
  uint16_t typicalParticleSizeR;
  //Initialisation a zero des variables avant acquisition
  meanT=0.;
  meanRH=0.;
  stdT=0.;
  stdRH=0.;
  mc1p0 = 0;
  mc2p5 = 0;
  mc4p0 = 0;
  mc10p0 = 0;
  nc0p5 = 0;
  nc1p0 = 0;
  nc2p5 = 0;
  nc4p0 = 0;
  nc10p0 = 0;
  typicalParticleSize = 0;

  uint16_t dataReadyFlag = 0;
  for (int i=0;i<400;i++) {  //met a zero le buffer pour calculer l'ecart type de la temp et RH
    BufferT[i]=0.0;
    BufferRH[i]=0.0;
  }
  indexcount =0;  //nombre d'acquisition
  
  unsigned long maintenant = millis(); //on recupere le temps de l'ESP32 !!!
  unsigned long debutAcquisition = maintenant; //on sauve le temps au demarrage de l'acquisition
  sensor.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16); //SPS30 en mode MESURE
  delay(1000);

  // Si on est dans la fenêtre d'acquisition
  while (maintenant - debutAcquisition < dureeAcquisition) {   //on reste ds la boucle while 5minutes!
    //lecture AMD2302
    float h = dht.readHumidity();   //on lit l'humidite
    float t = dht.readTemperature(); //on lit la temperature
    if (!isnan(h) && !isnan(t)) {   //on sauve les donnees pour le calcul de la moyenne et de l'ecart type
      Serial.printf("Mesure: T=%.1f°C, RH=%.1f%%\n", t, h);
      meanT+=t;  //meanT=meanT+t
      meanRH+=h;
      BufferT[indexcount]=t;
      BufferRH[indexcount]=h;
    }

    //lecture SPS30
    error = sensor.readDataReadyFlag(dataReadyFlag);
    if (error != NO_ERROR) {
      Serial.print("Error trying to execute readDataReadyFlag(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
    error = sensor.readMeasurementValuesUint16(mc1p0R, mc2p5R, mc4p0R, mc10p0R,
                                               nc0p5R, nc1p0R, nc2p5R, nc4p0R,
                                               nc10p0R, typicalParticleSizeR);
    if (error != NO_ERROR) {
      Serial.print("Error trying to execute readMeasurementValuesUint16(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
    mc1p0+=mc1p0R; mc2p5+=mc2p5R; mc4p0+=mc4p0R; mc10p0+=mc10p0R;
    nc0p5+=nc0p5R; nc1p0+=nc1p0R; nc2p5+=nc2p5R; nc4p0+=nc4p0R;
    nc10p0+=nc10p0R; typicalParticleSize+=typicalParticleSizeR;
    Serial.print("mc1p0: ");
    Serial.print(mc1p0);
    indexcount++;

    //on attend en gros une seconde pour prechaine acquisition
    delay(1000); 
    maintenant = millis();
  } 
  // Fin d’acquisition
  // Stop SPS30 

  Serial.println("=== Fin acquisition, calcul et envoi des données ===");
  meanT=meanT/(double)indexcount;
  meanRH=meanRH/(double)indexcount;
  for (int i=0;i<indexcount;i++) {
    stdT+=(meanT-BufferT[i])*(meanT-BufferT[i]);
    stdRH+=(meanRH-BufferRH[i])*(meanRH-BufferRH[i]);
  }
  //calcul ecart type T et RH
  stdT=sqrt(stdT/double(indexcount));
  stdRH=sqrt(stdRH/double(indexcount));

  // Calcul moyenne PM
  mc1p0=mc1p0/indexcount; mc2p5=mc2p5/indexcount; mc4p0=mc4p0/indexcount; mc10p0=mc10p0/indexcount;
  nc0p5=nc0p5/indexcount; nc1p0=nc1p0/indexcount; nc2p5=nc2p5/indexcount; nc4p0=nc4p0/indexcount;
  nc10p0=nc10p0/indexcount; typicalParticleSize=typicalParticleSize/indexcount;
 // Envoi des données
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

}
 

 
 
