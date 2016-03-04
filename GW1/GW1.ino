/*
    Weichensteuerung 
    4 Servos D3...D6
    7 Relais D7...D13 via ULN2003A (angezogen HIGH)
    I2C Port Expander A4, A5
      - PCF8574 P0...P7 LEDs (gegen VCC) Taster (gegen GND)
    (Später: DCC Eingang D2)

    Konfiguration für Grindelwald Weichen 1-4
    
    
    Copyright 2016 Franz Hennies

    Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
    der GNU General Public License, wie von der Free Software Foundation,
    Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
    veröffentlichten Version, weiterverbreiten und/oder modifizieren.

    Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
    OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
    Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
    Siehe die GNU General Public License für weitere Details.

    Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
    Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define DEBUG

#include <Wire.h>
#include <Servo.h>

boolean commAck = true;
// DCC pin 
const int dccPin = 2;

// ------- Konfiguration beginnt hier

// Servos an pins
const int servoPin[] = {
  3, 4, 5, 6 };     // Servos an D3...D6

// Zu jedem Servo ein Herzstückrelais
const int herzPin[] = {
  7, 8, 9, 10 };    // Herzstück-Relais an D7...D10
const int servoPos[][2] = {
  {110, 10},
  {120, 20},
  {130, 30},
  {140, 40} 
};       // geradePos, abzwPos
const boolean herzPol[][2] = {
  {1, 0},
  {1, 0},
  {1, 0},
  {1, 0} 
};       // geradePos, abzwPos

// Relais für Gleisspannug
const int gleisPin[] = {
  11, 12, 13 };     // Gleis-Relais an D11...D13
const boolean gleisZust[][2] = {
  {1, 0},
  {1, 0},
  {1, 0}
};       // On, Off

// Definiere Gruppen
const int tasterZuord[] = { // Welcher Taster gehört zu welcher Gruppe
  0, 0, 1, 1
};
const byte tasterGruppen[] = {  // Könnte man auch aus tasterZuord ausrechnen
  B11111100,                // 0,1 gehören zu Gruppe 0
  B11110011                 // 2,3 gehören zu Gruppe 1
};
const int weichenGruppen[][2] = { // Welche Weiche gehört zu welcher Gruppe
  {0, 1},
  {2, 3}
};                  // Gruppen: (W1, W2, G1) (W3, _W4, G2, _G3)
const int weichenMatrix[][2] = { // jeder Taster eine Zeile
  {0, 0},                        // jeder eintrag ein Weichenbefehl
  {1, 1},                        // O gerade 1 Abzweig
  {0, 0},
  {1, 1}
};
const int gleisGruppen[][2] = {  // Welches Gleisrelais gehört zu welcher Gruppe
  {0, 0},
  {1, 2}
};                  // Gruppen: (W1, W2, G1) (W3, _W4, G2, _G3)
const boolean gleisMatrix[][4] = { // jeder Taster eine Zeile
  {0, 0},                             // jeder Eintrag ein Relaisbefehl
  {1, 1},                             // O Gleis aus 1 Gleis an
  {0, 0},
  {1, 1}
};

// ------- Konfiguration endet hier


// Servoobjekte erstellen
Servo weichenServo[sizeof(servoPin)/2];

// Definiere PCF8574 Port Expander
#define PCF8574 0x39
byte pcfDaten = 0xFF;
byte pcfAlt = 0xFF;
byte pcfNeu = 0xFF;



// Funktionen

// Funktion zum Schreiben auf einen PCF8574-Baustein
void PCF8574_Write(int adresse, byte daten) {
  Wire.beginTransmission(adresse);
  Wire.write(daten);
  Wire.endTransmission();
  delay(5);
}

// Funktion zum Lesen des PCF8574-Bausteins
byte PCF8574_Read(int adresse) {
  byte datenByte=0xff;
  Wire.requestFrom(adresse,1);
  if(Wire.available()){
    datenByte=Wire.read();
    commAck=true;
    }
  else {
    commAck=false;
  }
  return datenByte;
}


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize I2C
  Wire.begin();
  // Attach Servos
  for (int i = 0; i < sizeof(servoPin[i])/2; i++) {
   weichenServo[i].attach(servoPin[i]);
  }
  // Alle Herzstück-Pins output und LOW
  for (int i = 0; i < sizeof(herzPin)/2; i++) {
   pinMode(herzPin[i], OUTPUT);
   digitalWrite(herzPin[i], LOW);
  }
  // Alle Gleis-Pins output und LOW
  for (int i = 0; i < sizeof(gleisPin)/2; i++) {
   pinMode(gleisPin[i], OUTPUT);
   digitalWrite(gleisPin[i], LOW);
  }
  // Lese Gruppenposition aus eeprom in pcfAlt

  // Alle PCF8574-Tasterpins wie vorher
  PCF8574_Write(PCF8574,pcfAlt);
}

void loop() {
  // Read PCF8574
  pcfDaten=PCF8574_Read(PCF8574);
  if (commAck) {
    pcfNeu = pcfAlt ^ pcfDaten;
  }
  else {
    pcfNeu = 0x00;
    pcfDaten = pcfAlt;
  }
  // Hier würde man den DCC befehl auslesen und in einen Tastendruck übersetzen
  // Wahrscheinlich eher nicht, sondern eine Funktion würde aufgerufen, hmmm
  // Also das hierunter in eine Funktion auslagern?
  // Oder die Funktion schreibt pcfNeu, aber wie sichern, dass zur rechten Zeit?



  // Lese Taster, schreibe Weichen und Gleise
  for (int i = 0; i < sizeof(tasterZuord)/2; i++) {
   int k = tasterZuord[i];                                         // k welche Weichengruppe
   boolean isSet = bitRead(pcfNeu,i);
   if (isSet) {                                                    // Taster gedrückt
    for (int j = 0; j < sizeof(weichenGruppen[k])/2; j++) {
     int l = weichenGruppen[k][j];                                 // l welche Weiche index 
     int m = weichenMatrix[i][j];                                  // m welche Position Taster i Weiche l (an stelle j)

#ifdef DEBUG
    Serial.print("Taster ");
    Serial.print(i);
    Serial.print(" Gruppe ");
    Serial.print(k);
    Serial.print(" Weiche ");
    Serial.print(l);
    Serial.print(" Position ");
    Serial.print(m);
    Serial.print(" ServoPosition ");
    Serial.print(servoPos[l][m]);
    Serial.print(" HerzPol ");
    Serial.print(herzPol[l][m]);
    Serial.println();
#endif
     
     weichenServo[l].write(servoPos[l][m]);
     digitalWrite(herzPin[l], herzPol[l][m]);
    }
    for (int j = 0; j < sizeof(gleisGruppen[k])/2; j++) {
     int l = gleisGruppen[k][j];                                   // l welches Gleisrelais
     int m = gleisMatrix[i][j];                                    // m welchen Zustand Taster i Relais l

#ifdef DEBUG
    Serial.print("Taster ");
    Serial.print(i);
    Serial.print(" Gruppe ");
    Serial.print(k);
    Serial.print(" Gleis ");
    Serial.print(l);
    Serial.print(" Zustand ");
    Serial.print(m);
    Serial.print(" Pol ");
    Serial.print(gleisZust[l][m]);
    Serial.println();
#endif

     digitalWrite(gleisPin[l], gleisZust[l][m]);
    }
    pcfDaten = pcfDaten & tasterGruppen[k];                        // Clear (set High) alle anderen Taster nur der gleichen Gruppe
    pcfDaten = pcfDaten | (tasterGruppen[k] ^ 0xFF);
    bitClear(pcfDaten,i);
   }
  }
  // Schreibe Tasten/LED status
  PCF8574_Write(PCF8574,pcfDaten);
  
  // Vergleiche vorher nachher
  //  if (pcfDaten != pcfAlt) {
                                                                   // Schreibe Weichen-/Gleisfunktionen ins Eeprom, nur wenn geändert
  pcfAlt = pcfDaten;
  //}
  delay(20);
  }