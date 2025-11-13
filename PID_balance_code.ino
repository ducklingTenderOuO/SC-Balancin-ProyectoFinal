/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  Modified for serial communication with Python interface
 */
#include <Wire.h>
#include <Servo.h>

///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo;  // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////

////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

///////////////////PID constants///////////////////////
float kp = 8; //Mine was 8
float ki = 0.2; //Mine was 0.2
float kd = 3100; //Mine was 3100
float distance_setpoint = 21;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

// Variables modificables desde serial
bool pid_active = true;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(125); //Put the servco at angle 125, so the balance is in the middle
  pinMode(Analog_in, INPUT);  
  time = millis();
  
  // Reservar espacio para el string de entrada
  inputString.reserve(200);
  
  Serial.println("Balancin PID - Listo");
  Serial.println("Comandos: KP,KI,KD,SET,TOGGLE,STATUS");
}

void loop() {
  // Leer comandos seriales
  if (stringComplete) {
    procesarComando();
    inputString = "";
    stringComplete = false;
  }
  
  if (millis() > time+period) {
    time = millis();    
    distance = get_dist(100);   
    
    if (pid_active) {
      // Ejecutar PID normal
      distance_error = distance_setpoint - distance;   
      PID_p = kp * distance_error;
      float dist_diference = distance_error - distance_previous_error;     
      PID_d = kd*((distance_error - distance_previous_error)/period);
        
      if(-3 < distance_error && distance_error < 3) {
        PID_i = PID_i + (ki * distance_error);
      } else {
        PID_i = 0;
      }
    
      PID_total = PID_p + PID_i + PID_d;  
      PID_total = map(PID_total, -150, 150, 0, 150);
    
      if(PID_total < 20){PID_total = 20;}
      if(PID_total > 160) {PID_total = 160; } 
    
      myservo.write(PID_total+30);  
      distance_previous_error = distance_error;
    }
    
    // Enviar datos cada 10 ciclos (aprox 500ms)
    static int counter = 0;
    if(counter++ > 10) {
      enviarDatos();
      counter = 0;
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void procesarComando() {
  inputString.trim();
  
  if (inputString.startsWith("KP:")) {
    kp = inputString.substring(3).toFloat();
    Serial.print("KP cambiado a: ");
    Serial.println(kp);
  }
  else if (inputString.startsWith("KI:")) {
    ki = inputString.substring(3).toFloat();
    Serial.print("KI cambiado a: ");
    Serial.println(ki);
  }
  else if (inputString.startsWith("KD:")) {
    kd = inputString.substring(3).toFloat();
    Serial.print("KD cambiado a: ");
    Serial.println(kd);
  }
  else if (inputString.startsWith("SET:")) {
    distance_setpoint = inputString.substring(4).toFloat();
    Serial.print("Setpoint cambiado a: ");
    Serial.println(distance_setpoint);
  }
  else if (inputString == "TOGGLE") {
    pid_active = !pid_active;
    Serial.print("PID ");
    Serial.println(pid_active ? "Activado" : "Desactivado");
  }
  else if (inputString == "STATUS") {
    enviarEstado();
  }
  else if (inputString == "RESET") {
    PID_i = 0;
    distance_previous_error = 0;
    Serial.println("PID resetado");
  }
  else {
    Serial.println("Comando no reconocido");
  }
}

void enviarDatos() {
  Serial.print("DATA:");
  Serial.print(distance);
  Serial.print(",");
  Serial.print(PID_total);
  Serial.print(",");
  Serial.print(pid_active);
  Serial.print(",");
  Serial.print(kp);
  Serial.print(",");
  Serial.print(ki);
  Serial.print(",");
  Serial.print(kd);
  Serial.print(",");
  Serial.println(distance_setpoint);
}

void enviarEstado() {
  Serial.print("ESTADO:KP=");
  Serial.print(kp);
  Serial.print(",KI=");
  Serial.print(ki);
  Serial.print(",KD=");
  Serial.print(kd);
  Serial.print(",SET=");
  Serial.print(distance_setpoint);
  Serial.print(",PID=");
  Serial.println(pid_active ? "ON" : "OFF");
}

float get_dist(int n) {
  long sum=0;
  for(int i=0;i<n;i++) {
    sum=sum+analogRead(Analog_in);
  }  
  float adc=sum/n;
  float distance_cm = 17569.7 * pow(adc, -1.2062);
  return(distance_cm);
}