#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include "driver/gpio.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); 

int dt_us = 2000;             // muestreo en micro-segundos
int tdt_us = 0;
float dt = dt_us*0.000001;    // muestro en segudos
unsigned long t1 = 0, t2 = 0; // Tiempos para intervalos
unsigned long k = 0;          // Contador de muestras

// Pines del encoder
const gpio_num_t chA_pin = GPIO_NUM_12;
const gpio_num_t chB_pin = GPIO_NUM_13;
volatile int Np = 0;                   // Número de pulsos

const float R = 0.3185840708;       // Resolución de salida Np =1130
float th = 0, thp =0;         // Posición angular y valor pasado
float dth_d = 0, dth_f =0;    // derivada discreta y s. filtrada
float alpha = 0.05;            // Coeficiente de filtro

//float kp = 0.48222, kd = 0.20961, ki = 0.23562;
float kp = 0.18, kd = 0.0001, ki = 0.05; // P
float tkp = 0.0, tkd = 0.0, tki = 0.0;  //Valores temporales
//float kp = 0.23, kd = 0.0470, ki = 0; // PD
 //float kp = 0.23, kd = 0.023, ki = 0.1;   // PID
float e = 0, de = 0, inte = 0; 
float u = 0, usat = 0;        // SEÑAL DE CONTROL
float PWM = 0;                  // SEÑAL PWM
float th_des = 0;             // CONSIGAN DEL CONTROL
float th_dest = 0;
float sp = 0;

// REVISAR CAUTELOSAMENTE EL SENTIDO DE GIRO
const int sen1 = 15;  //IN4
const int sen2 = 2;   //IN3

String consigna;

//Pulsadores
const int Btn1 = 26;
const int Btn2 = 27;
const int Btn3 = 14;

//Interfase:
uint LCDp = 0;
bool LCDok = false;

//Configuración Firebase
#define WIFI_SSID "Nistiquil"
#define WIFI_PASSWORD "techalote"

//----------------Accesos para claseiva
#define API_KEY "AIzaSyCEY1RBR5RWlt9i9w4F74V6nV-DjMblwFU"     
#define DATABASE_URL "https://final-iva-default-rtdb.firebaseio.com/"
#define USER_EMAIL "aaa@gmail.com"
#define USER_PASSWORD "321456"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Rutina de interrupción para canal A
void IRAM_ATTR CH_A() {
  int A = gpio_get_level(chA_pin);
  int B = gpio_get_level(chB_pin);
  if (A == B) Np++;
  else        Np--;
}

// Rutina de interrupción para canal B
void IRAM_ATTR CH_B() {
  int A = gpio_get_level(chA_pin);
  int B = gpio_get_level(chB_pin);
  if (A != B) Np++;
  else        Np--;
}

//Funcion enviar datos a Firebase RTDB
void updateFirebaseFromLocal() {
  Firebase.RTDB.setFloat(&fbdo, "/BTS/kp", kp);
  Firebase.RTDB.setFloat(&fbdo, "/BTS/ki", ki);
  Firebase.RTDB.setFloat(&fbdo, "/BTS/kd", kd);
  Firebase.RTDB.setInt(&fbdo, "/BTS/dt", dt_us);
  Firebase.RTDB.setFloat(&fbdo, "/BTS/Set-Point", th_des);
}

//Funcion jalar datos de Firebase
void updateLocalFromFirebase() {
  if (Firebase.RTDB.getFloat(&fbdo, "/BTS/kp")) kp = fbdo.floatData();
  if (Firebase.RTDB.getFloat(&fbdo, "/BTS/ki")) ki = fbdo.floatData();
  if (Firebase.RTDB.getFloat(&fbdo, "/BTS/kd")) kd = fbdo.floatData();
  if (Firebase.RTDB.getInt(&fbdo, "/BTS/dt")) dt_us = fbdo.intData();
  if (Firebase.RTDB.getFloat(&fbdo, "/BTS/Set-Point")) th_des = fbdo.floatData();
}

  TaskHandle_t firebaseTaskHandle = NULL;

void firebaseTask(void * parameter) {
  while (true) {
    updateLocalFromFirebase();
    //updateFirebaseFromLocal();
    Firebase.RTDB.setFloat(&fbdo, "/BTS/Sensor", th);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  analogWrite(sen1,0);
  analogWrite(sen2,0);
  // Comunicación Serial
  //Serial.begin(115200);
  //Comunicación RTDB
  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("WIFI AWAKENING");
  lcd.setCursor(2, 1);
  lcd.print("SEQEUENCE...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
  
    delay(500);
  }

  lcd.clear();

  //Configuración Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  while (auth.token.uid == "") {delay(100);}
  lcd.setCursor(1, 0);
  lcd.print("FIREBASE READY");
  updateFirebaseFromLocal();        //Manda vaores de arranque a Firebase
  delay(1000);
  lcd.clear();
  delay(500);

  pinMode(chA_pin, INPUT_PULLUP);
  pinMode(chB_pin, INPUT_PULLUP);

  // Salidas de control PWM
  pinMode(sen1,OUTPUT);
  pinMode(sen2,OUTPUT);

  //Pulsadores
  pinMode(Btn1, INPUT_PULLUP);
  pinMode(Btn2, INPUT_PULLUP);
  pinMode(Btn3, INPUT_PULLUP);
  
  //Awakening sequence
/*
  lcd.setCursor(3, 0);
  lcd.print("PRAISE THE");
  lcd.setCursor(3, 1);
  lcd.print("OMNISSIAH!");
  delay(3000);
  lcd.clear();
  */
  Np = 0;
  th = 0;

  //Interrupciones sensor cuadratura
  attachInterrupt(digitalPinToInterrupt(chA_pin), CH_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chB_pin), CH_B, CHANGE);

  xTaskCreatePinnedToCore(firebaseTask, "FirebaseTask", 8192, NULL, 1, &firebaseTaskHandle, 1);
}

void loop() {
  //Lectura pulsadores para cambio de pantalla
  if (digitalRead(Btn1) == LOW) {
    LCDp++;
    if (LCDp == 5) LCDp = 0;
    delay(75);
  }
  else if (digitalRead(Btn3) == LOW) {
    LCDp--;
    if (LCDp == -1) LCDp = 4;
    delay(75);
  }

  //Cambio de pantalla segun seleccion:
    if (LCDp == 0) {          //Set-Point
      lcd.setCursor(0,0);
      lcd.print("POS->");
      lcd.print(th);
      lcd.setCursor(0,1);
      lcd.print("SET-POINT->");
      lcd.print(th_des);
      if (digitalRead(Btn2) == LOW) {
        vTaskSuspend(firebaseTaskHandle);
        delay(75);
        do {
          lcd.setCursor(11,1);
          lcd.print("     ");
          lcd.setCursor(11,1);
          lcd.print(th_dest);
          if (digitalRead(Btn1) == LOW) {
            th_dest += 5;
            //if (th_dest > 365) th_dest = 0;
            delay(75);
          }
          else if (digitalRead(Btn3) == LOW) {
            th_dest -= 5;
            //if (th_dest < 0) th_dest = 0;
            delay(75);
          } 
        } while (digitalRead(Btn2) == HIGH);
        //<-------------------------------------------------------------------Actualizar Set-Point RTDB
        th_des = th_dest;
        LCDp = 0;
        th_dest = 0;
        //updateFirebaseFromLocal();
        Firebase.RTDB.setFloat(&fbdo, "/BTS/Set-Point", th_des);
        delay(100);
        vTaskResume(firebaseTaskHandle);
      }
    } else if (LCDp == 1) {   //Set KP
      lcd.setCursor(0,0);
      lcd.print("SET KP->");
      lcd.print(kp, 4);
      lcd.setCursor(5,1);
      lcd.print(tkp, 4);
      if (digitalRead(Btn2) == LOW) {
        vTaskSuspend(firebaseTaskHandle);
        delay(75);
        do {
          lcd.setCursor(5,1);
          lcd.print("      ");
          lcd.setCursor(5,1);
          lcd.print(tkp, 4);
          if (digitalRead(Btn1) == LOW) {
            tkp += 0.01;
            delay(75);
          }
          else if (digitalRead(Btn3) == LOW) {
            tkp -= 0.01;
            delay(75);
          } 
        } while (digitalRead(Btn2) == HIGH);
        //<-------------------------------------------------------------------Actualizar kp RTDB
        kp = tkp;
        LCDp = 0;
        tkp = 0;
        //updateFirebaseFromLocal();
        Firebase.RTDB.setFloat(&fbdo, "/BTS/kp", kp);
        delay(100);
        vTaskResume(firebaseTaskHandle);
      }
    } else if (LCDp == 2) {   //Set KI
      lcd.setCursor(0,0);
      lcd.print("SET KI->");
      lcd.print(ki, 4);
      lcd.setCursor(5,1);
      lcd.print(tki, 4);
      if (digitalRead(Btn2) == LOW) {
        vTaskSuspend(firebaseTaskHandle);
        delay(75);
        do {
          lcd.setCursor(5,1);
          lcd.print("      ");
          lcd.setCursor(5,1);
          lcd.print(tki, 4);
          if (digitalRead(Btn1) == LOW) {
            tki += 0.01;
            delay(75);
          }
          else if (digitalRead(Btn3) == LOW) {
            tki -= 0.01;
            delay(75);
          } 
        } while (digitalRead(Btn2) == HIGH);
        //<-------------------------------------------------------------------Actualizar ki RTDB
        ki = tki;
        LCDp = 0;
        tki = 0;
        //updateFirebaseFromLocal();
        Firebase.RTDB.setFloat(&fbdo, "/BTS/ki", ki);
        delay(100);
        vTaskResume(firebaseTaskHandle);
      }
    } else if (LCDp == 3) {   //Set KD
      lcd.setCursor(0,0);
      lcd.print("SET KD->");
      lcd.print(kd, 4);
      lcd.setCursor(5,1);
      lcd.print(tkd, 4);
      if (digitalRead(Btn2) == LOW) {
        vTaskSuspend(firebaseTaskHandle);
        delay(75);
        do {
          lcd.setCursor(5,1);
          lcd.print("      ");
          lcd.setCursor(5,1);
          lcd.print(tkd, 4);
          if (digitalRead(Btn1) == LOW) {
            tkd += 0.001;
            delay(75);
          }
          else if (digitalRead(Btn3) == LOW) {
            tkd -= 0.001;
            delay(75);
          } 
        } while (digitalRead(Btn2) == HIGH);
        //<-------------------------------------------------------------------Actualizar kd RTDB
        kd = tkd;
        LCDp = 0;
        tkd = 0;
        //updateFirebaseFromLocal();
        Firebase.RTDB.setFloat(&fbdo, "/BTS/kd", kd);
        delay(100);
        vTaskResume(firebaseTaskHandle);
      }
    } else if (LCDp == 4) {   //Set Dt
      lcd.setCursor(0,0);
      lcd.print("SET Dt->");
      lcd.print(dt_us);
      lcd.setCursor(6,1);
      lcd.print(tdt_us);
      if (digitalRead(Btn2) == LOW) {
        vTaskSuspend(firebaseTaskHandle);
        delay(75);
        do {
          lcd.setCursor(6,1);
          lcd.print("    ");
          lcd.setCursor(6,1);
          lcd.print(tdt_us);
          if (digitalRead(Btn1) == LOW) {
            tdt_us += 100;
            delay(75);
          }
          else if (digitalRead(Btn3) == LOW) {
            tdt_us -= 100;
            delay(75);
          } 
        } while (digitalRead(Btn2) == HIGH);
        //<-------------------------------------------------------------------Actualizar dt RTDB
        dt_us = tdt_us;
        LCDp = 0;
        tdt_us = 0;
        //updateFirebaseFromLocal();
        Firebase.RTDB.setFloat(&fbdo, "/BTS/dt", dt_us);
        delay(100);
        vTaskResume(firebaseTaskHandle);
      }
    }
  // El código se debe escribir entre la toma de los tiempo
  t1 = micros();  //Muestra de tiempo 1
    // ****************************************
    // LEER CONSIGNA DE CONTROL
/*      if(Serial.available()> 0){
      consigna = Serial.readStringUntil('\n');
      th_des = consigna.toFloat();
      
      }
*/      
      th = R*Np;            // Toma de muestra
 
      dth_d = (th-thp)/dt;  // Derivada discreta

      // Filtro de la velocidad 
      dth_f = alpha*dth_d + (1-alpha)*dth_f;
      

      /* Calcular elemetos del control */
      ////////////////////////////////////
      e = th_des - th;             // ERROR 
      de = - dth_f;                // DERIVADA DEL ERROR
      inte = inte + e*dt;          // INTEGRAL DEL ERROR
      // CONTROL PID
      u = kp*e + kd*de + ki*inte;  // VOLTAJE

      usat = constrain(u,-6,6); // SATURACION
      // PWM 0 a 255
      PWM = usat*21.25; // 255/6V = 42.5
      
             // MANDAR SEÑAL DE CONTROL COMO PWM
      if(PWM>0){
      analogWrite(sen1,PWM);
      analogWrite(sen2,0);
      }
      if(PWM<0){
      analogWrite(sen1,0);
      analogWrite(sen2,-1*PWM);
      }

      ////////////////////////////////////

      k=k+1;                // Número de muestra
      thp = th;             // Guardar valor de th
      
      // Imprimir los valores 
      
      //Serial.print(',');
      //Serial.print(th);
      //Serial.print(',');
      //Serial.println(PWM);
    // ****************************************
  t2 = micros();  //Muestra de tiempo 2
  while((t2-t1)<dt_us){
    t2 = micros();
  }
  lcd.clear();            //Limpieza de LCD
}