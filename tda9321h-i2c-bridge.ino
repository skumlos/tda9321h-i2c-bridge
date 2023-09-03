/* TDA9321H I2C Bridge
 * (2023) Martin Hejnfelt (martin@hejnfelt.com)
 *
 * For use with monitors that use the TDA9321H jungle IC.
 * This changes the I2C communication from monitor MCUs
 * that disables IE1 and IE2 bits.
 *
 * Connect the main I2C line of the monitors PCB to the 
 * hardware I2C (TWI) pins and then specify the desired
 * secondary I2C pins for the communication to the jungle.
 * Remember pull-up resistors on the seondary I2C bus (D4 and D5 below)
 * 2.2-3.3K ones should work fine, just solder between the pin
 * and the Arduinos VCC pin.
 *
 * On the Barco ADVM14, SCL is R45 and SDA is R44
 *
 * A speedy Arduino is recommended, for now only tested on:
 * - 16MHz Mini Pro clone thingy
 * 
 * Requires the SoftI2CMaster library
 * https://github.com/felias-fogg/SoftI2CMaster (or install through Library Manager)
 * 
 * Monitors tested:
 * BarcoNet ADVM14
 *
 * Expect your monitor to catch fire!
 */

// Version 1.0

#define I2C_TIMEOUT 10000
#define I2C_PULLUP 0
#define I2C_FASTMODE 0

#define READ_DELTA_MS (100)

// Only have one active of these, depending on what you want.
#define RGB_SWITCH_REG_VAL (0x3) /* IE1 and IE2 enabled */
//#define RGB_SWITCH_REG_VAL (0xB) /* IE1, IE2 and YUV mode for RGB1 enabled, untested... */

// These work for Nano 3.0 / Mini Pro
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5

#include <SoftI2CMaster.h>
#include <Wire.h>

// Address is 10001X1Y, X=<AS pin>=0/GND on ADVM14, Y=R/W
#define TDA9321H_ADDR (69)

#define REG_RGB_SWITCH (0x0A)

void setup() {
  bool iicinit = i2c_init();
  Wire.begin(TDA9321H_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
}

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TDA9321H_ADDR<<1)|I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop(); 
}

enum WriteState {
  WR_REG,
  WR_DATA
};

WriteState writeState = WR_REG;

int currentReg = 0x0;
uint8_t w[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void writeRequest(int byteCount) {
  for(int i=0; i<byteCount; ++i) {
    switch(writeState) {
      case WR_REG:
        currentReg = Wire.read();
        i2c_start((TDA9321H_ADDR<<1)|I2C_WRITE);
        i2c_write(currentReg);
        writeState = WR_DATA;
      break;
      case WR_DATA:
        w[currentReg] = Wire.read();
        if(REG_RGB_SWITCH == currentReg)
          w[currentReg] |= RGB_SWITCH_REG_VAL;
        i2c_write(w[currentReg++]);
      break;
    }
  }
  writeState = WR_REG;
  i2c_stop();
}

uint8_t r[4] = {0, 0, 0, 0};

void readRequest() {
  Wire.write(r,4);
}

void read() {
  i2c_start((TDA9321H_ADDR<<1)|I2C_READ);
  r[0] = i2c_read(false); // read one byte
  r[1] = i2c_read(false); // read one byte
  r[2] = i2c_read(false); // read one byte
  r[3] = i2c_read(true); // read one byte and send NAK to terminate
  i2c_stop(); // send stop condition
}

unsigned int last = 0;
// Constantly read the status regs to be able to serve them back upon request
void loop() {
  if((millis() - last) > READ_DELTA_MS) {
    noInterrupts();
    read();
    interrupts();
    last = millis();
  }
}