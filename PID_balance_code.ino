#include <Servo.h>

Servo myservo;
const int SERVO_PIN = 6;
const int ANG_CENTRO = 90;
const int ANG_MIN = 0;
const int ANG_MAX = 140;


const int trigPin = 10;
const int echoPin = 11;
const float DIST_CENTRO = 20.0; 

float kp = 38; //30; 45
float ki = .068; //.065 066;
float kd = 450; //450 488;

float referencia = 0;  

float error = 0;
float errorAnterior = 0;
float integral = 0;
float maxIntegral = 50;

unsigned long tPrev = 0;
const int T_MUESTREO = 15;

float PID_output = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  myservo.write(ANG_CENTRO);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  delay(500);
  Serial.println("Sistema listo.");
}

void loop() {
  
  leerComandosSerial();

  unsigned long tActual = millis();
  if (tActual - tPrev >= T_MUESTREO) {
    tPrev = tActual;

    float distancia = calculaDistancia();
    float posicion = distancia - DIST_CENTRO;  

    error = referencia - posicion;

    float P = kp * error;
    
    integral += error;
    integral = constrain(integral, -maxIntegral, maxIntegral);
    float I = ki * integral;
    
    float D = kd * (error - errorAnterior);
    errorAnterior = error;

    PID_output = P + I + D;

    int angulo = ANG_CENTRO - (int)PID_output;
    angulo = constrain(angulo, ANG_MIN, ANG_MAX);
    
    myservo.write(angulo);

    Serial.print("Dist: "); Serial.print(distancia);
    Serial.print("  Pos: "); Serial.print(posicion);
    Serial.print("  Ref: "); Serial.print(referencia);
    Serial.print("  Error: "); Serial.print(error);
    Serial.print("  Servo: "); Serial.print(angulo);
    Serial.print("  KP: "); Serial.print(kp);
    Serial.print("  KI: "); Serial.print(ki);
    Serial.print("  KD: "); Serial.println(kd);
  }
}


float calculaDistancia() {
  float suma = 0;
  int lecturas = 5;
  
  for (int i = 0; i < lecturas; i++) {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(trigPin, LOW);

    long duracion = pulseIn(echoPin, HIGH, 30000);
    suma += duracion * 0.034 / 2.0;
    delayMicroseconds(500);
  }
  
  return constrain(suma / lecturas, 2.0, 40.0);
}


void leerComandosSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  int espacio = cmd.indexOf(' ');
  if (espacio == -1) return;   // No hay número → ignorar

  String clave = cmd.substring(0, espacio);
  float valor = cmd.substring(espacio + 1).toFloat();

  if (clave == "kp") {
    kp = valor;
    Serial.print("Nuevo Kp: "); Serial.println(kp);
  }
  else if (clave == "ki") {
    ki = valor;
    Serial.print("Nuevo Ki: "); Serial.println(ki);
  }
  else if (clave == "kd") {
    kd = valor;
    Serial.print("Nuevo Kd: "); Serial.println(kd);
  }
  else if (clave == "r") {
    referencia = valor;
    integral = 0;
    Serial.print("Nueva referencia: "); Serial.println(referencia);
  }
}

