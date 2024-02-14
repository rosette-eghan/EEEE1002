//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot 2023                                     *//
//*                                                      *//
//*  Skeleton Master Code for Use with the               *//
//*  EEEBot_MainboardESP32_Firmware Code                 *//
//*                                                      *//
//*  Nat Dacombe                                         *//
//********************************************************//

// the following code acts as a 'bare bones' template for your own custom master code that works with the firmware code provided
// therefore, the variable names are non-descriptive - you should rename these variables appropriately
// you can either modify this code to be suitable for the project week task, or use the functions as inspiration for your own code

#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define Sensor1 26  // Orange
#define Sensor2 27  // Green
#define Sensor3 2  // Yellow
#define Sensor4 4  // Blue

float refPoint = 0;
float weightedAv = 0;
float XiSi = 0;
float error = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
}

// three integer values are sent to the slave device
int x = 200;
int y = 200;
int z = 90;

void loop()
{
  int value1 = analogRead(Sensor1);
  int value2 = analogRead(Sensor2);
  int value3 = analogRead(Sensor3);
  int value4 = analogRead(Sensor4);

  value1 = map(value1, 468, 4095, 0, 9);
  value2 = map(value2, 312, 4095, 0, 9);
  value3 = map(value3, 343, 4095, 0, 9);
  value4 = map(value4, 727, 4095, 0, 9);
 
  value1 = constrain(value1, 0, 9); //white = 9 & black = 0
  value2 = constrain(value2, 0, 9);
  value3 = constrain(value3, 0, 9);
  value4 = constrain(value4, 0, 9);
        
  Serial.print("Sensor 1 = ");
  Serial.println(value1);
  Serial.print("Sensor 2 = ");
  Serial.println(value2);
  Serial.print("Sensor 3 = ");
  Serial.println(value3);
  Serial.print("Sensor 4 = ");
  Serial.println(value4);

  delay(300);

  float totalValue = 0;
  totalValue += value1 + value2 + value3 + value4;

  XiSi = (3*value1)+(1*value2)+(-1*value3)+(-3*value4);

  weightedAv = XiSi/totalValue;
  error = refPoint-weightedAv;

  Serial.print("Weighted Average = ");
  Serial.println(weightedAv);
  Serial.print("Error = ");
  Serial.println(error);


  // two 16-bit integer values are requested from the slave
  int16_t a = 0;
  int16_t b = 0;
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
  uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  a = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
  b = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number

  Serial.print(a);
  Serial.print("\t");
  Serial.println(b);
  
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(x & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(y & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(z & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting
  delay(100);
}





