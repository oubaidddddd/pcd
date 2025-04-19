#include <Encoder.h>
#include <Wire.h>
#include <MPU6050.h>
// ----------- Encodeurs -----------
Encoder encoderGauche(3, 5);
Encoder encoderDroite(4, 2);

// ----------- Moteurs L298N -----------
int leftF = 7;
int leftR = 8;
int rightF = 9;
int rightR = 10;

// ----------- Capteur ultrasonique HC-SR04 -----------
const int trigFront1 = 28, echoFront1 = 29; // Premier capteur avant
const int trigFront2 = 30, echoFront2 = 31; // Deuxième capteur avant
const int trigLeft = 24, echoLeft = 25;
const int trigRight = 26, echoRight = 27;
// ----------- MPU6050 -----------
MPU6050 mpu;
int16_t ax_raw, ay_raw, az_raw; // Accélérations brutes
int16_t gx_raw, gy_raw, gz_raw; // Vitesses angulaires brutes
float ax, ay, az;               // Accélérations en g
float gx, gy, gz;               // Vitesses angulaires en degrés par seconde
// Variables pour le capteur
long duration;
int distanceFront1, distanceFront2, distanceLeft, distanceRight;

void setup() {
  // Configuration des broches des moteurs
  pinMode(rightF, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightR, OUTPUT);
  pinMode(leftR, OUTPUT);

  // Configuration des broches du capteur
  pinMode(trigFront1, OUTPUT); pinMode(echoFront1, INPUT);
  pinMode(trigFront2, OUTPUT); pinMode(echoFront2, INPUT);
  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  // Initialisation du MPU6050
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connecté !");
  } else {
    Serial.println("Erreur de connexion MPU6050.");
  }

  // Initialisation du moniteur série
  Serial.begin(19200);
}

void loop() {
  // Lire la distance à l'obstacle
  distanceFront1 = getDistance(trigFront1, echoFront1);
  distanceFront2 = getDistance(trigFront2, echoFront2);
  distanceLeft = getDistance(trigLeft, echoLeft);
  distanceRight = getDistance(trigRight, echoRight);
  long gauche = encoderGauche.read();
  long droite = encoderDroite.read();
    // Lire les données du MPU6050
  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    // Convertir les données brutes en unités physiques
  ax = ax_raw / 16384.0; // Conversion en g
  ay = ay_raw / 16384.0;
  az = az_raw / 16384.0;
  gx = gx_raw / 131.0; // Conversion en °/s
  gy = gy_raw / 131.0;
  gz = gz_raw / 131.0;
  // Afficher la distance pour le débogage
  Serial.print("Front1: "); Serial.print(distanceFront1); Serial.print(" cm");
  Serial.print(" | Front2: "); Serial.print(distanceFront2); Serial.print(" cm");
  Serial.print(" | Left: "); Serial.print(distanceLeft);Serial.print(" cm");
  Serial.print(" | Right: "); Serial.print(distanceRight);Serial.print(" cm");
  Serial.print(" | Encoder G: "); Serial.print(gauche);
  Serial.print(" | Encoder D: "); Serial.print(droite);
  Serial.print(" | gx: "); Serial.print(gx, 3);
  Serial.print(" | gy: "); Serial.print(gy, 3);
  Serial.print(" | gz: "); Serial.print(gz, 3);
  Serial.print(" | ax: "); Serial.print(ax, 3);
  Serial.print(" | ay: "); Serial.print(ay, 3);
  Serial.print(" | az: "); Serial.println(az, 3);
  Serial.println("=======================================");

  // Comportement du robot basé sur la distance
  int minFront = min(distanceFront1, distanceFront2); // Distance minimale à l'avant
  if (minFront > 20 || minFront == 0) {
    if (distanceLeft > 10 && distanceRight > 10) {
      forward(50); // Avancer si pas d'obstacles
    } else if (distanceLeft < distanceRight) {
      right(500); // Tourner à droite si obstacle à gauche
    } else {
      left(500); // Tourner à gauche si obstacle à droite
    }
  } else {
    stp(500); // Stop temporaire
    back(500); // Reculer légèrement
    left(200); // Tourner à gauche pour éviter l'obstacle
  }
  delay(100); // Pause pour la stabilité
}

// ----------- Fonctions de mouvement -----------

void forward(int x) {
  analogWrite(rightF, 80);
  analogWrite(leftF, 80);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}

void right(int x) {
  analogWrite(rightF, 130);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}

void left(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 140);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}

void stp(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 0);
  analogWrite(leftR, 0);
  delay(x);
}

void back(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightR, 120);
  analogWrite(leftR, 118);
  delay(x);
}

// ----------- Fonction pour obtenir la distance -----------

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int dist = duration * 0.034 / 2;

  if (dist == 0 || dist > 200) return 0;
  return dist;
}
