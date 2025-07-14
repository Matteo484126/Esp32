#include <Wire.h>
#include <Adafruit_PN532.h>
#include <MPU6050_tockn.h>
#include <NewPing.h>

// Pin I2C
#define SDA_PIN 21
#define SCL_PIN 22

// Pin per vari sensori
#define echo 32
#define trigger 33
#define ledSensore 14
#define ledVerde 26
#define ledRosso 25
#define relay 15

// Stato
bool bloccato = false;

// SENSORI
NewPing sonar(trigger, echo, 200);
MPU6050 mpu6050(Wire);
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

int erroriConsec = 0;
float accX_offset = 0.0;
int16_t ax;

// Variabili per il multi-core
TaskHandle_t SonarTask;
volatile int distanzaAttuale = 0;
volatile bool nuovaLettura = false;
SemaphoreHandle_t distanzaSemaphore;

// Numero di carte salvate
const int NUM_CARTE = 4;
const int UID_LENGTH = 4;

uint8_t carteRiconosciute[NUM_CARTE][UID_LENGTH] = {
  {0xFD, 0x40, 0x46, 0x05},
  {0xA5, 0xDD, 0x9B, 0x02},
  {0x4D, 0x9A, 0xDA, 0x03},
  {0xA5, 0x8B, 0x44, 0x05}
};

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

  // Crea semaforo per proteggere l'accesso alla variabile distanza
  distanzaSemaphore = xSemaphoreCreateMutex();

  // Crea task per il sonar sul core 0
  xTaskCreatePinnedToCore(
    taskSonar,        // Funzione del task
    "SonarTask",      // Nome del task
    2048,             // Stack size
    NULL,             // Parametri del task
    1,                // Priorità del task
    &SonarTask,       // Handle del task
    0                 // Core su cui eseguire (0 o 1)
  );

  Serial.println("Task sonar avviato sul core 0");
}

void configuraPin(){
  pinMode(relay, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(ledSensore, OUTPUT);
  pinMode(ledRosso, OUTPUT);
  pinMode(ledVerde, OUTPUT);
}

// Task dedicato alla lettura del sonar (eseguito sul core 0)
void taskSonar(void *pvParameters) {
  for(;;) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    long durata = pulseIn(echo, HIGH, 25000);
    int distanza = durata * 0.034 / 2;

    // Protezione thread-safe per aggiornare la distanza
    if (xSemaphoreTake(distanzaSemaphore, portMAX_DELAY) == pdTRUE) {
      distanzaAttuale = distanza;
      nuovaLettura = true;
      xSemaphoreGive(distanzaSemaphore);
    }

    Serial.print("Distanza: ");
    Serial.println(distanza);
    
    // Lettura ogni 100ms per evitare interferenze
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Funzione thread-safe per leggere la distanza
int leggiDistanza() {
  int distanza = 0;
  if (xSemaphoreTake(distanzaSemaphore, portMAX_DELAY) == pdTRUE) {
    distanza = distanzaAttuale;
    xSemaphoreGive(distanzaSemaphore);
  }
  return distanza;
}

void loop(void){ 
  uint8_t success;
  uint8_t uid[7];
  uint8_t uidLength;
  
  if(!bloccato){
    if(inMovimento(accX_offset) && ostacolo()){
      blocca();
    }
  } else {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50);
    if (success) {
      stampaDatiCarta(uid, uidLength);

      delay(700);

      bool accettata = trovaCarta(uid, uidLength);

      if (accettata) {
        if (ostacolo()) {
          if (inMovimento(accX_offset)) {
            blocca(); // cancello si sta muovendo --> blocca subito
          } else {
            lampeggia(); // rilevato ostacolo con cancello fermo --> segnala
            //Serial.println("sto lampeggiando");
          }
        } else {
          cartaAccettata();
        }
      } else {
        cartaRifiutata();
      }
    }
  }
  delay(50);
}

void cartaRifiutata() {
  Serial.println("Carta Rifiutata!!");
  digitalWrite(ledRosso, HIGH);

  erroriConsec++;
  if (erroriConsec == 3) {
    Serial.print("Terzo errore riprovare tra 1 min");
    delay(60000); // Attende un minuto dopo 3 errori consecutivi
  }
  delay(500);
  
  digitalWrite(ledRosso, LOW);
}

void cartaAccettata() {
  bloccato = false;
  Serial.println("carta accettata");
  erroriConsec = 0;
  digitalWrite(ledVerde, HIGH);

  digitalWrite(relay, HIGH);
  delay(100);
  digitalWrite(relay, LOW);
  delay(100);

  digitalWrite(ledVerde, LOW);
}

void blocca() {
  bloccato = true;
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

  return accX_corrected > 0.3 || accX_corrected < -0.3;
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

// Nuova funzione ostacolo che utilizza la lettura thread-safe
bool ostacolo() {
  int distanza = leggiDistanza();
  return distanza > 0 && distanza < 15;
}

void stampaDatiCarta(uint8_t uid[],  uint8_t uidLength){
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
}

bool trovaCarta(uint8_t uid[], uint8_t uidLength){
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
          return true;
          break;
        }
      }
    }
  return false;
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
