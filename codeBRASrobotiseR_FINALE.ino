/************************************************************
 * Projet : Bras robotise (4 servomoteurs)
 * Auteur : Lotfi Chenoun - 6A
 * Carte  : M5Core (ESP32) + PCA9685
 *
 * 2 modes au demarrage :
 *   [A] PS5    -> manette Bluetooth (librairie F. Kapita)
 *   [B] ANALOG -> 2 joysticks (2 servos chacun)
 *
 * Dans les 2 modes : la pince est protegee par une mesure
 * de courant (ACS712). On ne serre plus si le courant est
 * trop haut.
 *
 * Ressources : Claude.AI, code de James Gordon-Ball
 ************************************************************/

#include <M5Unified.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "controller.h"

/* ===== 1) MATERIEL ===== */

// Driver des servos (PCA9685, adresse I2C 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 100   // PWM pour 0 degre
#define SERVOMAX 610   // PWM pour 180 degres

// Canaux des servos sur le PCA9685
#define SERVO_BASE   0
#define SERVO_EPAULE 4
#define SERVO_COUDE  8
#define SERVO_PINCE  12

// Capteur de courant ACS712 (pince)
#define PIN_COURANT_PINCE  36
#define SENSIBILITE_ACS712 0.185   // 0.185 V/A
#define COURANT_MAX_PINCE  1.5   // seuil de securite (A)

// Joysticks 
#define JS1_X 13   // Base
#define JS1_Y 12   // Epaule
#define JS2_X 15   // Pince
#define JS2_Y 25   // Coude
#define DEADZONE 100   // on ignore les petits mouvements (bruit)
#define VITESSE  2     // pas du servo a chaque cycle (en degres)

/* ===== 2) VARIABLES ===== */

// Mode actuel
#define MODE_MENU   0
#define MODE_PS5    1
#define MODE_ANALOG 2
int modeActuel = MODE_MENU;

// Position de repos des joysticks 
int js1x_zero = 2048, js1y_zero = 2048;
int js2x_zero = 2048, js2y_zero = 2048;

// Angles des servos 
int angleBase   = 90;
int angleEpaule = 90;
int angleCoude  = 90;
int anglePince  = 90;

// Tension du capteur de courant quand rien ne passe
float tensionZero = 0;

/* ===== 3) FONCTIONS OUTILS ===== */

// Convertit un angle (0-180) en impulsion PWM
int angleToPulse(int ang) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}

// Envoie un angle a un servo (borne entre 0 et 180)
void moveServo(int canal, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.setPWM(canal, 0, angleToPulse(angle));
}

// Lit une broche en moyennant 8 lectures (pour enlever le bruit)
int lireADC(int pin) {
  analogRead(pin);              // 1ere lecture jetee (stabilisation)
  int somme = 0;
  for (int i = 0; i < 8; i++) somme += analogRead(pin);
  return somme / 8;
}

// Regarde le SENS du joystick : renvoie -1, 0 ou +1
int lireSens(int pin, int zero) {
  int ecart = lireADC(pin) - zero;     // de combien on s'eloigne du centre
  if (ecart >  DEADZONE) return 1;     // pousse d'un cote
  if (ecart < -DEADZONE) return -1;    // pousse de l'autre cote
  return 0;                            // au centre : on ne bouge pas
}

// Lit le courant de la pince en amperes
float lireCourantPince() {
  float tension = analogRead(PIN_COURANT_PINCE) * 3.3 / 4095.0;
  float courant = (tension - tensionZero) / SENSIBILITE_ACS712;
  if (courant < 0) courant = 0;
  return courant;
}

/* ===== 4) DEPLACEMENT DU BRAS (commun aux 2 modes) ===== */

// Deplace la pince en securite
//   delta < 0 = on ouvre  -> toujours permis
//   delta > 0 = on serre  -> seulement si le courant n'est pas trop haut
void deplacerPince(int delta, float courant) {
  if (delta < 0) {
    anglePince += delta;
  } else if (courant < COURANT_MAX_PINCE) {
    anglePince += delta;
  }
}

// Empeche les servos de depasser leurs limites
void appliquerLimites() {
  angleBase   = constrain(angleBase,   0,   180);
  angleEpaule = constrain(angleEpaule, 5,   170);
  angleCoude  = constrain(angleCoude,  10,  160);
  anglePince  = constrain(anglePince,  54,  180);
}

// Envoie les 4 angles aux 4 servos
void envoyerAuxServos() {
  moveServo(SERVO_BASE,   angleBase);
  moveServo(SERVO_EPAULE, angleEpaule);
  moveServo(SERVO_COUDE,  angleCoude);
  moveServo(SERVO_PINCE,  anglePince);
}

// Remet le bras au centre (90 degres)
void centrerBras() {
  angleBase = 90;   moveServo(SERVO_BASE, angleBase);   delay(200);
  angleEpaule = 90; moveServo(SERVO_EPAULE, angleEpaule); delay(200);
  angleCoude = 90;  moveServo(SERVO_COUDE, angleCoude);  delay(200);
  anglePince = 90;  moveServo(SERVO_PINCE, anglePince);  delay(200);
}

// Affiche les angles + le courant (toutes les 200 ms)
void afficherEtat(float courant) {
  static unsigned long dernier = 0;
  if (millis() - dernier < 200) return;
  dernier = millis();

  M5.Lcd.fillRect(0, 80, 320, 160, BLACK);
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("Base   : %d\n", angleBase);
  M5.Lcd.printf("Epaule : %d\n", angleEpaule);
  M5.Lcd.printf("Coude  : %d\n", angleCoude);
  M5.Lcd.printf("Pince  : %d\n", anglePince);
  M5.Lcd.setTextColor(courant > COURANT_MAX_PINCE * 0.8 ? RED : GREEN);
  M5.Lcd.printf("Courant: %.2f A\n", courant);
  M5.Lcd.setTextColor(WHITE);
}

/* ===== 5) CALIBRATION DES JOYSTICKS ===== */

// Mesure le centre de chaque axe (ne pas toucher les joysticks)
void calibrerJoysticks() {
  M5.Lcd.setCursor(0, 180);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibration... ne touchez pas les joysticks !");

  long s1x = 0, s1y = 0, s2x = 0, s2y = 0;
  for (int i = 0; i < 50; i++) {
    s1x += lireADC(JS1_X);
    s1y += lireADC(JS1_Y);
    s2x += lireADC(JS2_X);
    s2y += lireADC(JS2_Y);
    delay(10);
  }
  js1x_zero = s1x / 50;
  js1y_zero = s1y / 50;
  js2x_zero = s2x / 50;
  js2y_zero = s2y / 50;
}

/* ===== 6) ECRAN ===== */

void afficherMenu() {
  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(40, 10);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("=== BRAS ROBOTISE ===");

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(20, 50);
  M5.Lcd.println("Choisissez le mode :");

  M5.Lcd.fillRoundRect(10, 85, 140, 50, 8, BLUE);
  M5.Lcd.setCursor(25, 100);
  M5.Lcd.println(" [A] PS5");

  M5.Lcd.fillRoundRect(165, 85, 145, 50, 8, RED);
  M5.Lcd.setCursor(172, 100);
  M5.Lcd.println("[B] Analog");

  M5.Lcd.setTextColor(LIGHTGREY);
  M5.Lcd.setCursor(10, 155);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("A = manette Bluetooth PS5");
  M5.Lcd.println("B = 2 joysticks analogiques");
  M5.Lcd.println("C = retour menu");
}

void afficherEcranPS5() {
  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.setCursor(50, 5);
  M5.Lcd.println("MODE PS5");
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(0, 35);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Stick gauche X -> Base");
  M5.Lcd.println("Stick gauche Y -> Epaule");
  M5.Lcd.println("Stick droit  Y -> Coude");
  M5.Lcd.println("Stick droit  X -> Pince");
  M5.Lcd.println("Btn C          -> Menu");
}

void afficherEcranAnalogique() {
  M5.Lcd.clear();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.setCursor(20, 5);
  M5.Lcd.println("MODE ANALOGIQUE");
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(0, 35);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("JS1 X -> Base");
  M5.Lcd.println("JS1 Y -> Epaule");
  M5.Lcd.println("JS2 Y -> Coude");
  M5.Lcd.println("JS2 X -> Pince");
  M5.Lcd.println("Btn C -> Menu");
}

/* ===== 7) SETUP (une seule fois au demarrage) ===== */

void setup() {
  M5.begin();
  Serial.begin(115200);

  Wire.begin(21, 22);   // I2C du PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);   // 50 Hz pour les servos

  // Calibration du zero du capteur de courant (moyenne de 100 lectures)
  float somme = 0;
  for (int i = 0; i < 100; i++) {
    somme += analogRead(PIN_COURANT_PINCE) * 3.3 / 4095.0;
    delay(5);
  }
  tensionZero = somme / 100.0;

  afficherMenu();
}

/* ===== 8) LOOP (boucle principale) ===== */

void loop() {
  M5.update();   // met a jour les boutons A/B/C

  // --- MENU ---
  if (modeActuel == MODE_MENU) {
    if (M5.BtnA.wasPressed()) {        // A -> PS5
      modeActuel = MODE_PS5;
      centrerBras();
      initBT_Controller();
      afficherEcranPS5();
    }
    if (M5.BtnB.wasPressed()) {        // B -> analogique
      modeActuel = MODE_ANALOG;
      centrerBras();
      afficherEcranAnalogique();
      calibrerJoysticks();
    }
    return;
  }

  // --- Bouton C : retour au menu ---
  if (M5.BtnC.wasPressed()) {
    modeActuel = MODE_MENU;
    afficherMenu();
    return;
  }

  // --- Courant de la pince (commun aux 2 modes) ---
  float courant = lireCourantPince();

  // --- On bouge selon le mode ---
  if (modeActuel == MODE_PS5) {
    controllerUpdate();
    angleBase   += var_axisX  / 300;
    angleEpaule -= var_axisY  / 300;
    angleCoude  -= var_axisRY / 300;
    deplacerPince(-var_axisRX / 300, courant);
  }
  else if (modeActuel == MODE_ANALOG) {
    angleBase   += lireSens(JS1_X, js1x_zero) * VITESSE;
    angleEpaule -= lireSens(JS1_Y, js1y_zero) * VITESSE;
    angleCoude  -= lireSens(JS2_Y, js2y_zero) * VITESSE;
    deplacerPince(lireSens(JS2_X, js2x_zero) * VITESSE, courant);
  }

  // --- Limites + envoi + affichage (commun) ---
  appliquerLimites();
  envoyerAuxServos();
  afficherEtat(courant);

  delay(20);
}
