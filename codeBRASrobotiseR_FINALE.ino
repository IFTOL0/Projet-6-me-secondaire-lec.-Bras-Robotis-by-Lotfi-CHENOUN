/************************************************************
 * Projet  : Bras robotisé (4 servomoteurs)
 * Auteur  : Lotfi Chenoun — 6A
 * Carte   : M5Core 
 ************************************************************
 * Deux modes au choix dans le menu de démarrage :
 *   [A] PS5      -> manette Bluetooth (librairie F. Kapita)
 *   [B] ANALOG   -> 2 joysticks analogiques (2 servos chacun)
 **************************************************************
 * Dans les DEUX modes, la pince est protégée par une mesure
 * de courant (capteur ACS712) : on ne peut pas continuer à
 * serrer si le courant dépasse le seuil de sécurité.
 ***************************************************************
 * Mapping :
 *   Joystick gauche  : X -> Base   , Y -> Épaule
 *   Joystick droit   : X -> Pince  , Y -> Coude
 *
 * **********************************************************
 * RESSOURCES Utilisée
 * Claude.AI
 * Code de James Gardon Ball
 ************************************************************/

#include <M5Unified.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "controller.h"   // librairie manette PS5 (F. Kapita)

/* *********************************************************
   1) MATÉRIEL
   ********************************************************* */

/* ---- Driver de servomoteurs PCA9685 (adresse I2C 0x40) ---- */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 100      // impulsion PWM correspondant à 0°
#define SERVOMAX 610      // impulsion PWM correspondant à 180°

/* ---- Canaux du PCA9685 utilisés pour chaque servo ---- */
#define SERVO_BASE    0
#define SERVO_EPAULE  4
#define SERVO_COUDE   8
#define SERVO_PINCE   12

/* ---- Capteur de courant ACS712 sur la pince ---- */
#define PIN_COURANT_PINCE  36     
#define SENSIBILITE_ACS712 0.1    // 0,185 V/A 
#define COURANT_MAX_PINCE  1.5    

/* ---- Joysticks analogiques ---- */
  
#define JS1_X 13     // Base
#define JS1_Y 12     // Épaule
#define JS2_X 15     // Pince
#define JS2_Y 25     // Coude
#define DEADZONE 200 // zone morte (sur 4095) pour ignorer le bruit au repos

/* =========================================================
   2) VARIABLES GLOBALES
   ========================================================= */

/* Mode courant : on utilise des noms plutôt que 0/1/2 pour la clarté */
#define MODE_MENU   0
#define MODE_PS5    1
#define MODE_ANALOG 2
int modeActuel = MODE_MENU;

/* Valeurs de repos des joysticks */
int js1x_zero = 2048, js1y_zero = 2048;
int js2x_zero = 2048, js2y_zero = 2048;

/* Angles des 4 servos.*/
float angleBase   = 90;
float angleEpaule = 90;
float angleCoude  = 90;
float anglePince  = 90;

/* Tension lue par l'ACS712*/
float tensionZero = 0;

/* **************************************************************
   3)FONCTIONS
***************************************************************** */

/* Convertit un angle (0–180°) en impulsion PWM pour le PCA9685 */
int angleToPulse(int ang) {
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}
/* Envoie un angle à un servo (entre 0 et 180°) */
void moveServo(int canal, float angle) {
  int a = constrain((int)round(angle), 0, 180);
  pwm.setPWM(canal, 0, angleToPulse(a));
}
/* Lit un axe de joystick et renvoie une valeur entre -1.0 et +1.0.*/
float lireAxe(int pin, int zero) {
  int val = analogRead(pin) - zero;
  if (abs(val) < DEADZONE) return 0.0;
  if (val > 0) return (float)val / (float)(4095 - zero);  // côté positif
  else         return (float)val / (float)zero;           // côté négatif
}
/* Lit le courant de la pince en ampères.*/
float lireCourantPince() {
  float tension = analogRead(PIN_COURANT_PINCE) * 3.3 / 4095.0;
  float courant = (tension - tensionZero) / SENSIBILITE_ACS712;
  if (courant < 0) courant = 0;
  return courant;
}
/* *********************************************************
   4) DÉPLACEMENT DU BRAS (code COMMUN aux deux modes)
   ********************************************************** */
/* Déplace la pince en sécurité.*/
void deplacerPince(float delta, float courant) {
  if (delta < 0) {
    anglePince += delta;                       
  } else if (courant < COURANT_MAX_PINCE) {
    anglePince += delta;                       
  }
}
/* Empêche chaque servo de dépasser ses limites mécaniques */
void appliquerLimites() {
  angleBase   = constrain(angleBase,   0,   180);
  angleEpaule = constrain(angleEpaule, 5,   170);
  angleCoude  = constrain(angleCoude,  10,  160);
  anglePince  = constrain(anglePince,  54,  180);
}
/* Envoie les 4 angles aux 4 servos d'un coup */
void envoyerAuxServos() {
  moveServo(SERVO_BASE,   angleBase);
  moveServo(SERVO_EPAULE, angleEpaule);
  moveServo(SERVO_COUDE,  angleCoude);
  moveServo(SERVO_PINCE,  anglePince);
}
/* Remet le bras au centre (90°)*/
void centrerBras() {
  angleBase = angleEpaule = angleCoude = anglePince = 90;
  envoyerAuxServos();
}
/* Affichage périodique*/
void afficherEtat(float courant) {
  static unsigned long dernier = 0;
  if (millis() - dernier < 200) return;   
  dernier = millis();

  M5.Lcd.fillRect(0, 80, 320, 160, BLACK);   
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.printf("Base   : %d\n", (int)angleBase);
  M5.Lcd.printf("Epaule : %d\n", (int)angleEpaule);
  M5.Lcd.printf("Coude  : %d\n", (int)angleCoude);
  M5.Lcd.printf("Pince  : %d\n", (int)anglePince);
  // courant en rouge s'il approche du seuil, sinon en vert
  M5.Lcd.setTextColor(courant > COURANT_MAX_PINCE * 0.8 ? RED : GREEN);
  M5.Lcd.printf("Courant: %.2f A\n", courant);
  M5.Lcd.setTextColor(WHITE);
}

/* *********************************************************
   5) CALIBRATION DES JOYSTICKS
   ********************************************************* */

/* Mesure la position de repos de chaque axe */
void calibrerJoysticks() {
  M5.Lcd.setCursor(0, 180);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.println("Calibration... ne touchez pas les joysticks !");

  long s1x = 0, s1y = 0, s2x = 0, s2y = 0;
  for (int i = 0; i < 50; i++) {
    s1x += analogRead(JS1_X);
    s1y += analogRead(JS1_Y);
    s2x += analogRead(JS2_X);
    s2y += analogRead(JS2_Y);
    delay(10);
  }
  js1x_zero = s1x / 50;
  js1y_zero = s1y / 50;
  js2x_zero = s2x / 50;
  js2y_zero = s2y / 50;

  Serial.printf("Zeros: JS1X=%d JS1Y=%d JS2X=%d JS2Y=%d\n",
                js1x_zero, js1y_zero, js2x_zero, js2y_zero);
}

/* ********************************************************
   6) ÉCRANS (menu + écrans d'info des deux modes)
   ******************************************************** */

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
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.println(" [A] PS5");

  M5.Lcd.fillRoundRect(165, 85, 145, 50, 8, RED);
  M5.Lcd.setCursor(172, 100);
  M5.Lcd.setTextColor(WHITE);          // <-- CORRIGÉ : "WHITE" et non "White"
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

/* **********************************************************
   7) SETUP — exécuté une seule fois au démarrage
   ********************************************************** */
void setup() {
  M5.begin();
  Serial.begin(115200);

  Wire.begin(21, 22);   
  pwm.begin();
  pwm.setPWMFreq(50);   

  /* Calibration du "zéro" du capteur de courant */
  float somme = 0;
  for (int i = 0; i < 100; i++) {
    somme += analogRead(PIN_COURANT_PINCE) * 3.3 / 4095.0;
    delay(5);
  }
  tensionZero = somme / 100.0;

  afficherMenu();       
}

/* *********************************************************
   8) LOOP — boucle principale
   ********************************************************* */
void loop() {
  M5.update();          

  /* ---------- MENU ---------- */
  if (modeActuel == MODE_MENU) {
    if (M5.BtnA.wasPressed()) {        // A -> mode PS5
      modeActuel = MODE_PS5;
      centrerBras();
      initBT_Controller();             // active la manette Bluetooth
      afficherEcranPS5();
    }
    if (M5.BtnB.wasPressed()) {        // B -> mode analogique
      modeActuel = MODE_ANALOG;
      centrerBras();
      afficherEcranAnalogique();
      calibrerJoysticks();             
    }
    return;                           
  }

  /* ---------- Bouton C : retour au menu ---------- */
  if (M5.BtnC.wasPressed()) {
    modeActuel = MODE_MENU;
    afficherMenu();
    return;
  }

  /* ---------- Mesure du courant ---------- */
  float courant = lireCourantPince();

  /* ---------- Calcul des déplacements SELON le mode ---------- */
  if (modeActuel == MODE_PS5) {
    controllerUpdate();                          // lit la manette
    angleBase   += var_axisX  / 300.0;           // stick gauche X
    angleEpaule -= var_axisY  / 300.0;           // stick gauche Y
    angleCoude  -= var_axisRY / 300.0;           // stick droit  Y
    deplacerPince(-var_axisRX / 300.0, courant); // stick droit  X
  }
  else if (modeActuel == MODE_ANALOG) {
    float vitesse = 1.5;                          // degrés max par cycle
    angleBase   += lireAxe(JS1_X, js1x_zero) * vitesse;
    angleEpaule -= lireAxe(JS1_Y, js1y_zero) * vitesse;
    angleCoude  -= lireAxe(JS2_Y, js2y_zero) * vitesse;
    deplacerPince(lireAxe(JS2_X, js2x_zero) * vitesse, courant);
  }

  appliquerLimites();
  envoyerAuxServos();
  afficherEtat(courant);

  delay(20);   
}

