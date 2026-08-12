//CONTIENE LAS LIBRERIAS BASICAS PARA TRABAJAR CON EL DRIVER DE LEDS, ACCEDIENDO DE FORMA DIRECTA A LOS REGISTROS DEL DRIVER
//PARA SU CORRECTO FUNCIONAMIENTO. 
//EL PROGRAMA AQUI CONSISITE EN UN DEMO QUE SIRVE PARA BLINKEAR LOS LEDS DE LA TARJETA. 
#include <Wire.h>
#include <stdint.h>
#include "LP55231_Direct.h"

// ====== Demo ======
void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  Begin1();
  Enable1();
  for (uint8_t i = 0; i < 9; i++) {
      SetDriveCurrent1(i,0x4F);
  }
  Serial.println("BlINKEANDO LEDS RGB CON DRIVER LP55231 ");
}

void loop() {
  // Rojo: D2 D4 D6
  Serial.println("BLINK RED");
  const uint8_t red[] ={D2, D4, D6};
  for (int i=0;i<3;i++) SetChannelPWM1(red[i], 255);
  const uint8_t grn1[] = {D1, D3, D5};
  for (int i=0;i<3;i++) SetChannelPWM1(grn1[i], 165);
  delay(500);
  for (int i=0;i<3;i++) SetChannelPWM1(red[i], 0);
    for (int i=0;i<3;i++) SetChannelPWM1(grn1[i], 0);
  delay(500);

  // Verde: D1 D3 D5
  Serial.println("BLINK GREEN");
  const uint8_t grn[] = {D1, D3, D5};
  for (int i=0;i<3;i++) SetChannelPWM1(grn[i], 255);
  delay(500);
  for (int i=0;i<3;i++) SetChannelPWM1(grn[i], 0);
  delay(500);

  // Azul: D7 D8 D9
  Serial.println("BLINK BLUE");
  const uint8_t blu[] = {D7, D8, D9};
  for (int i=0;i<3;i++) SetChannelPWM1(blu[i], 128);
    const uint8_t blu1[] = {D2, D4, D6};
  for (int i=0;i<3;i++) SetChannelPWM1(blu1[i], 128);
  delay(500);
  for (int i=0;i<3;i++) SetChannelPWM1(blu[i], 0);
  for (int i=0;i<3;i++) SetChannelPWM1(blu1[i], 0);
  delay(500);
}
