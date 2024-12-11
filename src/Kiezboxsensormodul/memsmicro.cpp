#include "memsmicro.h"

const uint8_t I2S_SCK = 7;
const uint8_t I2S_WS = 5;
const uint8_t I2S_DIN = 4;
const uint8_t I2S_MCLK = -1;
const uint8_t I2S_SDOUT = -1;

I2SClass i2s;
bool i2sInitialized = false; // Flag für Initialisierung

void tell_me_the_noise() {
  // Falls nicht initialisiert, I2S starten
  if (!i2sInitialized) {
    Serial.begin(9600); // Nur beim ersten Aufruf initialisieren
    i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT, I2S_DIN, I2S_MCLK);

    if (!i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT)) {
      Serial.println("Failed to initialize I2S!");
      return;
    }

    i2sInitialized = true; // Initialisierung erfolgreich
    delay(1000);           // Stabilisierung
  }

  // Daten lesen
  int sample = i2s.read();

  if (sample == -1) {
    // Kein gültiges Sample
    return;
  }

  sample >>= 6; // Optionale Reduktion der Daten
  Serial.print("Mico misst: ");
  Serial.println(sample);
}
