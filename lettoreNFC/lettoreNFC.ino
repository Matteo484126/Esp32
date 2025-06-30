#include <Wire.h>
#include <Adafruit_PN532.h>
#include <MPU6050_tockn.h>


// Pin I2C
#define SDA_PIN 21
#define SCL_PIN 22

MPU6050 mpu6050(Wire);

#define echo 32
#define trigger 33
#define ledSensore 14
#define ledVerde 2
#define ledRosso 19
#define relay 13

// Numero di carte salvate
const int NUM_CARTE = 4;
const int UID_LENGTH = 4;

int erroriConsec = 0;

float accX_offset = 0.0;
int16_t ax;

float segnale, distanza;

uint8_t carteRiconosciute[NUM_CARTE][UID_LENGTH] = {
  {0xFD, 0x40, 0x46, 0x05},
  {0xA5, 0xDD, 0x9B, 0x02},
  {0x4D, 0x9A, 0xDA, 0x03},
  {0xA5, 0x8B, 0x44, 0x05}
};

// Inizializza l'oggetto PN532 usando l'I2C
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

void setup(void) {
  Serial.begin(115200);
  
  configuraPin();
  
  // Inizializza bus I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("Calibrazione asse X in corso... Tieni il sensore fermo.");
  accX_offset = calibraAccX();

  Serial.println("Inizializzazione del lettore NFC (PN532)...");
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Errore: non trovo un modulo PN532!");
    while (1);
  }

  Serial.println("Trovato chip PN5");
  nfc.SAMConfig();
  Serial.println("In attesa di una carta NFC...");
}

void configuraPin(){
  pinMode(relay, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(ledSensore, OUTPUT);
  pinMode(ledRosso, OUTPUT);
  pinMode(ledVerde, OUTPUT);
}


//////////////////////////////////////////////////////////////////////////////// loop
void loop(void){ 
  uint8_t success;
  uint8_t uid[7];
  uint8_t uidLength;

  if(inMovimento(accX_offset)){
    blocca();
  }

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  if (success) {
    Serial.println("\nCarta NFC rilevata!");
    Serial.print("UID (lunghezza ");
    Serial.print(uidLength, DEC);
    Serial.print("): ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print("0x");
      Serial.print(uid[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    delay(500);

    bool accettata = false;
    if (uidLength == UID_LENGTH) {
      for (uint8_t i = 0; i < NUM_CARTE; i++) {
        bool match = true;
        for (uint8_t j = 0; j < UID_LENGTH; j++) {
          if (uid[j] != carteRiconosciute[i][j]) {
            match = false;
            break;
          }
        }
        if (match) {
          accettata = true;
          break;
        }
      }
    }
    if (accettata) {
      if (ostacolo()) {
        if (inMovimento(accX_offset)) {
          blocca(); // cancello si sta muovendo, blocca subito
        } else {
          lampeggia(); // ostacolo ma cancello fermo, segnala
          Serial.println("sto lampeggiando");
        }
      } else {
        cartaAccettata();
      }
    } else {
      cartaRifiutata();
    }
  }
}


void cartaRifiutata() {
  Serial.println("Carta Rifiutata!!");
  digitalWrite(ledRosso, HIGH);

  erroriConsec++;
  if (erroriConsec == 3) {
    Serial.print("terzo errore riprovare tra 1 min");
    delay(60000); // Attende un minuto dopo 3 errori consecutivi
  }
  delay(500);
  
  digitalWrite(ledRosso, LOW);
}

void cartaAccettata() {
  Serial.print("carta accettata");
  erroriConsec = 0;
  digitalWrite(ledVerde, HIGH);

  digitalWrite(relay, HIGH);
  delay(100);
  digitalWrite(relay, LOW);
  delay(100);

  digitalWrite(ledVerde, LOW);
}

void blocca() {
  digitalWrite(relay, HIGH);
  delay(100);
  digitalWrite(relay, LOW);
  delay(100);
}

boolean inMovimento(float accX_offset) {
  mpu6050.update();
  float accX_corrected = mpu6050.getAccX() - accX_offset;
  Serial.print("Movimento: ");
  Serial.println(accX_corrected);

  return accX_corrected > 1 || accX_corrected < -1;
}

void lampeggia(){
  for(int i = 0; i<3; i++){
    digitalWrite(ledVerde, HIGH);
    delay(500);
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRosso, HIGH);
    delay(500);
    digitalWrite(ledRosso, LOW);
  }
}

bool ostacolo() {
  digitalWrite(trigger, HIGH);  
  delayMicroseconds(10); 
  digitalWrite(trigger, LOW);
  segnale = pulseIn(echo, HIGH, 30000);
  distanza = segnale / 58.2;
  Serial.print("Distanza: ");
  Serial.println(distanza);
  return distanza > 0 && distanza < 15;
}


float calibraAccX() {
  float somma = 0;
  const int N = 100;
  for (int i = 0; i < N; i++) {
    mpu6050.update();
    somma += mpu6050.getAccX();
    delay(10);
  }
  return somma / N;
}
