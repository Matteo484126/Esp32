#include <Wire.h>
#include <Adafruit_PN532.h>

// Definizione dei pin I2C per ESP32
#define SDA_PIN 21
#define SCL_PIN 22
int redLed = 25;
int greenLed = 26;
int buzzer = 27;
const int NUM_CARTE = 2;
const int UID_LENGTH = 4;

uint8_t carteRiconosciute[NUM_CARTE][UID_LENGTH] = {
  {0xFD, 0x40, 0x46, 0x05},
  {0xA5, 0xDD, 0x9B, 0x02}
};


// Inizializza l'oggetto PN532 usando l'I2C
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

void setup(void) {
  Serial.begin(115200);

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  Serial.println("Inizializzazione del lettore NFC (PN532)...");

  // Inizializzazione del lettore NFC
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Errore: non trovo un modulo PN532!");
    while (1); // Blocco esecuzione
  }

  // Stampa versione firmware
  Serial.print("Trovato chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware version: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  // Configura il modulo per leggere tag ISO14443A (NFC tipo Mifare)
  nfc.SAMConfig();

  Serial.println("In attesa di una carta NFC...");
}

void loop(void) {
  uint8_t success;
  uint8_t uid[7];      // Buffer per l'UID
  uint8_t uidLength;   // Lunghezza dell'UID

  // Cerca un tag
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

    delay(2000); // Piccola pausa per evitare letture multiple troppo ravvicinate
    
    bool accettata = false;

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

        if (accettata) {
          cartaAccettata(greenLed);
        } else {
          cartaRifiutata(redLed, buzzer);
        }
      }
}

void cartaRifiutata(int redLed, int buzzer){
  Serial.println("Carta Rifiutata!!");

  for (int i = 0; i < 3; i++) {
    digitalWrite(redLed, HIGH);
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(redLed, LOW);
    digitalWrite(buzzer, LOW);
    delay(200);
  }
}

void cartaAccettata(int greenLed){
  digitalWrite(greenLed, HIGH);
  Serial.println("Carta Accettata!!");
  delay(1000);
  digitalWrite(greenLed, LOW);
}     