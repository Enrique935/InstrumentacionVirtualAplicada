int dt_us = 2000;             // muestreo en micro-segundos
float dt = dt_us*0.000001;    // muestro en segudos
unsigned long t1 = 0, t2 = 0; // Tiempos para intervalos
unsigned long k = 0;          // Contador de muestras

int Np = 0;                   // Número de pulsos
const float R = 0.642857;       // Resolución de salida
float th = 0, thp =0;         // Posición angular y valor pasado
float dth_d = 0, dth_f =0;    // derivada discreta y s. filtrada
float alpha = 0.05;            // Coeficiente de filtro

//float kp = 0.48222, kd = 0.20961, ki = 0.23562;
float kp = 0.12, kd = 0.001   , ki = 0.001; // P
//float kp = 0.23, kd = 0.0470, ki = 0; // PD
 //float kp = 0.23, kd = 0.023, ki = 0.1;   // PID
float e = 0, de = 0, inte = 0; 
float u = 0, usat = 0;        // SEÑAL DE CONTROL
float PWM = 0;                  // SEÑAL PWM
float th_des = 0;             // CONSIGAN DEL CONTROL
// REVISAR CAUTELOSAMENTE EL SENTIDO DE GIRO
const int sen1 = 5; 
const int sen2 = 6; 

String data;
String setpoint;
String P;
String D;
String I;

void setup() {
  // Comunicación Serial
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), CH_A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), CH_B, RISING);

  // Salidas de control PWM
  pinMode(sen1,OUTPUT);
  pinMode(sen2,OUTPUT);
}

void loop() {
  //Serial.print(Np);
  //Serial.print(", ");
  //Serial.println(th);
  //Serial.print(", ");
  //Serial.println(e);
  // El código se debe escribir entre la toma de los tiempo
  t1 = micros();  //Muestra de tiempo 1
    // ****************************************
    // LEER CONSIGNA DE CONTROL
      if(Serial.available()> 0){
      setpoint = Serial.readStringUntil(',');
      P = Serial.readStringUntil(',');
      D = Serial.readStringUntil(',');
      I = Serial.readStringUntil('\n');

      th_des = setpoint.toFloat();
      kp = P.toFloat();
      kd = D.toFloat();
      ki = I.toFloat();
      }
      
      th = -1*R*Np;            // Toma de muestra
 
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
      //Serial.print(th_des);
      //Serial.print(",");
      //Serial.print(kp);
      //Serial.print(",");
      //Serial.print(ki);
      //Serial.print(",");
      //Serial.println(kd);
      //Serial.print(',');
      //Serial.println(PWM);
      Serial.println(e);
    // ****************************************
  t2 = micros();  //Muestra de tiempo 2
  while((t2-t1)<dt_us){
    t2 = micros();
  }
  //Serial.println(t2-t1);
  //delay(50);
}

void CH_A(){
  if (digitalRead(3)==LOW)
  Np=Np+1;
  if (digitalRead(3)==HIGH)
  Np=Np-1;
}

void CH_B(){
  if (digitalRead(2)==HIGH)
  Np=Np+1;
  if (digitalRead(2)==LOW)
  Np=Np-1;
}
