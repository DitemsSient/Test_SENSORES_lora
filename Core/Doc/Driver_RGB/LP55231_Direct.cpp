#include <Wire.h>
#include <stdint.h>
#include "LP55231_Direct.h"


/* ============================================================
                            Funciones 
   ============================================================ */

void Begin1(void) {
  Wire.begin();
  Reset1();                 // REG_RESET = 0xFF
}

void Enable1(void) {
  // CNTRL1: set enable bit (bit6) => 0x40
  LP_WriteReg(REG_CNTRL1, 0x40);

  // MISC: internal clock + charge pump + auto increment (según lib) => 0x53
  LP_WriteReg(REG_MISC, 0x53);
}

void Disable1(void) {
  uint8_t val = LP_ReadReg(REG_CNTRL1);
  val &= ~0x40;            // limpia bit6
  LP_WriteReg(REG_CNTRL1, val);
}

void Reset1(void) {
  LP_WriteReg(REG_RESET, 0xFF);
}

bool SetChannelPWM1(uint8_t channel, uint8_t value) {
  if(channel >= NumChannels) return false;
  LP_WriteReg(REG_D1_PWM + channel, value);
  return true;
}
bool SetMasterFader1(uint8_t fader, uint8_t value) {
  if(fader >= NumFaders) return false;
  LP_WriteReg(REG_MASTER_FADE_1 + fader, value);
  return true;
}

bool SetLogBrightness1(uint8_t channel, bool enable) {
  if(channel >= NumChannels) return false;

  uint8_t regVal = LP_ReadReg(REG_D1_CTRL + channel);
  regVal &= ~0x20;                 // bit5
  if(enable) regVal |= 0x20;
  LP_WriteReg(REG_D1_CTRL + channel, regVal);
  return true;
}

bool SetDriveCurrent1(uint8_t channel, uint8_t value) {
  if(channel >= NumChannels) return false;
  LP_WriteReg(REG_D1_I_CTL + channel, value);
  return true;
}
    
bool AssignChannelToMasterFader1(uint8_t channel, uint8_t fader) {
  if(channel >= NumChannels) return false;
  if(fader >= NumFaders) return false;

  uint8_t regVal = LP_ReadReg(REG_D1_CTRL + channel);

  // bits7:6 seleccionan master fader: (fader+1) en 2 bits, shift 6
  uint8_t bitVal = ((fader + 1) & 0x03) << 6;

  regVal &= ~0xC0;
  regVal |= bitVal;

  LP_WriteReg(REG_D1_CTRL + channel, regVal);
  return true;
}

/* ============================================================
   I2C: WriteReg / ReadReg con Wire (lo que buscabas)
   ============================================================ */

void LP_WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LP55231_ADDR);
  Wire.write(reg);     // dirección de registro
  Wire.write(val);     // dato
  Wire.endTransmission();
}

uint8_t LP_ReadReg(uint8_t reg) {
  // set pointer
  Wire.beginTransmission(LP55231_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start

  Wire.requestFrom((int)LP55231_ADDR, 1);
  if(Wire.available()) return (uint8_t)Wire.read();
  return 0;
}
