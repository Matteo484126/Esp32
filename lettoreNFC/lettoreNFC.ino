#include <Wire.h>
#include <Adafruit_PN532.h>
#include <MPU6050_tockn.h>


// Pin I2C
#define SDA_PIN 21
#define SCL_PIN 22

MPU6050 mpu6050(Wire);

#define IR 4
#define ledVerde 2
#define ledRosso 15
#define relay 35

// Numero di carte salvate
const int NUM_CARTE = 4;
const int UID_LENGTH = 4;

int erroriConsec = 0;

//define acceleration values of 3 axes 
int16_t ax,ay,az;

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

  pinMode(relay, OUTPUT);
  pinMode(IR, INPUT);

  // Inizializza un solo bus I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("Inizializzazione del lettore NFC (PN532)...");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Errore: non trovo un modulo PN532!");
    while (1);
  }

  Serial.print("Trovato chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware version: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.SAMConfig();

  Serial.println("In attesa di una carta NFC...");
}

void loop(void){ ///////////////////////Loop
  uint8_t success;
  uint8_t uid[7];
  uint8_t uidLength;

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

    delay(2000); // Evita letture troppo rapide

    if(digitalRead(IR) && inMovimento()){
      blocca();
      
    } else if(digitalRead(IR)){
      lampeggia();

    } else {
      bool accettata = false;
      for (uint8_t i = 0; i < NUM_CARTE; i++) { //////////////////////////////////Lettura carta
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
        } else {
          accettata = false;
        }
      }

      if (accettata) {
        cartaAccettata();
      } else {
        cartaRifiutata();
      }

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

boolean inMovimento() {
  mpu6050.update();
  return mpu6050.getRawAccX()!=0;
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
