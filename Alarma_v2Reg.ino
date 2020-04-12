// Verificar que xa,ya,za sean diferentes de cero, si no cambiar los registros

//Revisar sobre link IMU https://blog.mide.com/accelerometer-specifications-decoding-a-datasheet

#include <LiquidCrystal_I2C.h>
//#include<MPU6050.h>
#include "Wire.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int btn = 9;
const int piezo = 11;

const double var = 0.07;

int btn_st = 0;

long acc_x = 0, acc_y = 0, acc_z = 0; //prim
int gyro_x = 0, gyro_y = 0, gyro_z = 0; //prim
int temperatura = 0; //prim
int aux1 = 0, aux2 = 0, aux3 = 0, aux4 = 0; //prim
int count;
//int x=0,y=0,z=0;
long xa = 0, ya = 0, za = 0, xg = 0, yg = 0, zg = 0; //prim

double acc_x_cal, acc_y_cal, acc_z_cal;       //Almacenaran calibración
double acc_x_calp, acc_y_calp, acc_z_calp;    //Almacena estado anterior

double delta1 = 0, delta2 = 0, delta3 = 0;

double acc_x_ste = 0, acc_y_ste = 0, acc_z_ste = 0; //almacena salida STR habilitada
double acc_x_std = 0, acc_y_std = 0, acc_z_std = 0; //Almacena salida STR desabilitda
double acc_x_str = 0, acc_y_str = 0, acc_z_str = 0; //Almacenaran valores de auto test
double acc_x_ft = 0, acc_y_ft = 0, acc_z_ft = 0;    //Almacenaran ajustes de fabrica
long loop_timer;
bool flag;
bool f1 = false, f2 = false, f3 = false, fbtn = false;

void setup()
{
  Wire.begin();
  pinMode(piezo, OUTPUT); //inicializa pin de salida para el piezo-electrico
  pinMode(btn, INPUT);    //inicializa pin de salida para el boton

  setup_mpu_6050_registers();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Alerta");
  lcd.setCursor(0, 1);
  lcd.print("V2.0");

  delay(1500);
  lcd.clear();
  calibracion_acc();
  setup_mpu_6050_registers();
  loop_timer = micros();

  read_mpu_6050_data();     //Evita activacion del piezo al inicio
  acc_x_cal = ((double)acc_x - acc_x_str) / 4096;
  acc_y_cal = ((double)acc_y - acc_y_str) / 4096;
  acc_z_cal = ((double)acc_z - acc_z_str) / 4096;
  acc_x_calp = acc_x_cal;
  acc_y_calp = acc_y_cal;
  acc_z_calp = acc_z_cal;

}

void loop()
{
  btn_st = 0;
  acc_x_calp = acc_x_cal;   //Almacena estado anterior
  acc_y_calp = acc_y_cal;
  acc_z_calp = acc_z_cal;


  read_mpu_6050_data();     //Lee nuevos datos;
  acc_x_cal = ((double)acc_x - acc_x_str) / 4096;
  acc_y_cal = ((double)acc_y - acc_y_str) / 4096;
  acc_z_cal = ((double)acc_z - acc_z_str) / 4096;
  //  acc_x_cal = (double)acc_x/4096;
  //  acc_y_cal = (double)acc_x/4096;
  //  acc_z_cal = (double)acc_x/4096;

  //  x=map(acc_x, -16000,16000, -125,125);
  //  y=map(acc_y, -16000,16000, -125,125);
  //  z=map(acc_z, -16000,16000, -125,125);

  acc_x_calp = abs(acc_x_cal - acc_x_calp);
  acc_y_calp = abs(acc_y_cal - acc_y_calp);
  acc_z_calp = abs(acc_z_cal - acc_z_calp);

  lcd.setCursor(0, 0);
  lcd.print("x: ");
  lcd.print(acc_x_cal);
  //  lcd.print("  ");
  //  lcd.print(temperatura);
  //  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("y: ");
  lcd.print(acc_y_cal);
  //  lcd.print("    z: ");
  //  lcd.print(acc_z);
  delay(100);
  lcd.clear();

  if ((acc_x_calp >= var) | (acc_y_calp >= var) | (acc_z_calp >= var))
  {
    lcd.setCursor(0, 0);
    lcd.print("Activado");
    digitalWrite(piezo,HIGH);
    //digitalWrite(piezo, LOW);
    evento_btn();
    delay(7000);
    digitalWrite(piezo,LOW);
    //digitalWrite(piezo, HIGH);
    lcd.clear();
  }
  else {
    digitalWrite(piezo,LOW);
    //digitalWrite(piezo, HIGH);
  }

  //  while(micros() - loop_timer <4000);
  //  loop_timer=micros;
}


void evento_btn()
{
  btn_st = digitalRead(btn);
  if (btn_st == HIGH) {
    // turn LED on:
    digitalWrite(piezo,LOW);
    //digitalWrite(piezo, HIGH);
    lcd.clear();
    //    break;
  }
}

void calibracion_acc()         //Asigna valores de STR a partir del residuo de entre SelfTest activado y ST desactivado
{
  flag = true;
  setup_st_mpu_6050_data();
  factorytrim_mpu_6050_registers();                           //Obtiene ajustes de fabrica Acc_r_ft

  while (f1 == false) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    if (flag == true)                                       //Transfiere datos SelfTest activado
    {
      setup_st_mpu_6050_data();
      read_mpu_6050_data();
      acc_x_ste = (double)acc_x;
      flag = false;                                       //Cambia estado de bandera
    }
    else                                                 //Transfiere datos a SelfTest desactivado
    {
      setup_st_mpu_6050_data();
      read_mpu_6050_data();
      acc_x_std = (double)acc_x;
      flag = true;
    }
    acc_x_str = acc_x_ste - acc_x_std;
    delta1 = (acc_x_str - acc_x_ft) / acc_x_ft;

    if (abs(delta1) * 100 >= 1)
      f1 = true;

    while (f2 == false) //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    {
      if (flag == true)                                       //Transfiere datos SelfTest activado
      {
        setup_st_mpu_6050_data();
        read_mpu_6050_data();
        acc_y_ste = (double)acc_y;
        flag = false;                                       //Cambia estado de bandera
      }
      else                                                 //Transfiere datos a SelfTest desactivado
      {
        setup_st_mpu_6050_data();
        read_mpu_6050_data();
        acc_y_std = (double)acc_y;
        flag = true;
      }
      acc_y_str = acc_y_ste - acc_y_std;
      delta2 = (acc_y_str - acc_y_ft) / acc_y_ft;

      if (abs(delta2) * 100 >= 1)
        f2 = true;

      while (f3 == false) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      {
        if (flag == true)                                       //Transfiere datos SelfTest activado
        {
          setup_st_mpu_6050_data();
          read_mpu_6050_data();
          acc_z_ste = (double)acc_z;
          flag = false;                                       //Cambia estado de bandera
        }
        else                                                 //Transfiere datos a SelfTest desactivado
        {
          setup_st_mpu_6050_data();
          read_mpu_6050_data();
          acc_z_std = (double)acc_z;
          flag = true;
        }
        acc_z_str = acc_z_ste - acc_z_std;
        delta3 = (acc_z_str - acc_z_ft) / acc_z_ft;
        //El test solo será valido si se cumple la condicion de cambio en el ajuste de fabrica
        if (abs(delta3) * 100 >= 1)
          f3 = true;
      }
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("Test completado");
  delay(1500);
  lcd.clear();
}

void read_mpu_6050_data()
{ //Subrutina para leer los datos crudos del acelerometro
  Wire.beginTransmission(0x68);                                        //Inicia comunicaciones con MPU-6050
  Wire.write(0x3B);                                                    //Envia el registro de inicio solicitado
  Wire.endTransmission();                                              //Finaliza transmision
  Wire.requestFrom(0x68, 14);                                          //Solicita 14 bytes al MPU-6050
  while (Wire.available() < 14);                                       //Espera mientras todos los bytes son recibidos
  acc_x = Wire.read() << 8 | Wire.read();                              //Une el byte bajo y alto acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //
  acc_z = Wire.read() << 8 | Wire.read();                              //
  temperatura = Wire.read() << 8 | Wire.read();                        //
  gyro_x = Wire.read() << 8 | Wire.read();                             //
  gyro_y = Wire.read() << 8 | Wire.read();                             //
  gyro_z = Wire.read() << 8 | Wire.read();                             //
}

void setup_st_mpu_6050_data()
{
  //Activa MPU-6050
  Wire.beginTransmission(0x68);                                        //Inicia comunicaciones con MPU-6050
  Wire.write(0x6B);                                                    //Envía el registro de inicio solicitado
  Wire.write(0x00);                                                    //Fija el registro de inicio
  Wire.endTransmission();                                              //Finaliza transmision
  //Configura el acelerometro a (+/-Xg)
  Wire.beginTransmission(0x68);                                        //
  Wire.write(0x1C);                                                    //Configuración de acelerometro
  if (flag == true)
  {
    Wire.write(0xF0);                                                    //F0(8g) para autotest es forzoso configurar a 8g
    Wire.endTransmission();                                              //
    //Configura el gyro (500dps escala completa)
    Wire.beginTransmission(0x68);                                        //
    Wire.write(0x1B);                                                    //
    Wire.write(0x08);                                                    //
    Wire.endTransmission();                                              //
  }
  else
  {
    Wire.write(0x10);                                                    //10(8g) ajuste necesariamente a 8g
    Wire.endTransmission();                                              //
    //Configura el gyro (500dps escala completa)
    Wire.beginTransmission(0x68);                                        //
    Wire.write(0x1B);                                                    //
    Wire.write(0x08);                                                    //
    Wire.endTransmission();                                              //
  }
}

void setup_mpu_6050_registers()
{
  //Activa MPU-6050
  Wire.beginTransmission(0x68);                                        //Inicia comunicaciones con MPU-6050
  Wire.write(0x6B);                                                    //Envía el registro de inicio solicitado
  Wire.write(0x00);                                                    //Fija el registro de inicio
  Wire.endTransmission();                                              //Finaliza transmision
  //Configura el acelerometro a (+/-Xg)
  Wire.beginTransmission(0x68);                                        //
  Wire.write(0x1C);                                                    //Configuración de acelerometro
  Wire.write(0x10);                                                    //00(2g), 08(4g), 10(8g),18(16g)
  Wire.endTransmission();                                              //
  //Configura el gyro (500dps escala completa)
  Wire.beginTransmission(0x68);                                        //
  Wire.write(0x1B);                                                    //
  Wire.write(0x08);                                                    //
  Wire.endTransmission();                                              //
}

void factorytrim_mpu_6050_registers()
{
  Wire.beginTransmission(0x68);                                       //Inicia comunicaciones con MPU-6050
  Wire.write(0x0D);                                                   //Envia el registro de inicio solicitado
  Wire.endTransmission();                                             //Finaliza transmision
  Wire.requestFrom(0x68, 4);                                          //Solicita 4 bytes al MPU-6050
  while (Wire.available() < 4);                                       //Espera mientras todos los bytes son recibidos
  aux1 = Wire.read();                                                 //xa[4-2], xg[4-0]
  aux2 = Wire.read();                                                 //ya[4-2], yg[4-0]
  aux3 = Wire.read();                                                 //za[4-2], zg[4-0]
  aux4 = Wire.read();                                                 //reservado[1-0], xa[1-0], ya[1-0], za[1-0],
  xa = (aux1 >> 3 & 0x1C) | (aux4 >> 4 & 0x03);
  ya = (aux2 >> 3 & 0x1C) | (aux4 >> 2 & 0x03);
  za = (aux3 >> 3 & 0x1C) | (aux4 & 0x03);

  if (xa == 0) {
    acc_x_ft = 0;
    lcd.setCursor(0, 1);
    lcd.print("x: ");
    lcd.print(acc_x_ft);
    delay(500);
    lcd.clear();
  }
  else
    acc_x_ft = 4096 * 0.34 * pow(0.92 / 0.34, (((double)xa - 1) / 30));

  if (ya == 0) {
    acc_y_ft = 0;
    lcd.setCursor(0, 1);
    lcd.print("y: ");
    lcd.print(acc_y_ft);
    delay(500);
    lcd.clear();
  }
  else
    acc_y_ft = 4096 * 0.34 * pow(0.92 / 0.34, (((double)ya - 1) / 30));

  if (za == 0) {
    acc_z_ft = 0;
    lcd.setCursor(0, 1);
    lcd.print("z: ");
    lcd.print(acc_z_ft);
    delay(500);
    lcd.clear();
  }
  else
    acc_z_ft = 4096 * 0.34 * pow(0.92 / 0.34, (((double)za - 1) / 30));
}
