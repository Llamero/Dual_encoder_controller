#include <Wire.h>
#pragma pack(1) //Remove alignment padding bytes in structs - https://forum.pjrc.com/threads/50536-problem-with-union-in-Teensy-3-5
struct encoderStruct{
  uint16_t encoder_pos[2]; //Encoder PWM
  uint8_t command; //Instruction from controller
  uint8_t checksum; //Checksum
};

const struct defaultEncoderStruct{
  uint16_t encoder_pos[2] = {1,1}; //Encoder PWM
  uint8_t command = 0; //Instruction from controller
  uint8_t checksum = 0; //Checksum
} defaultEncoder;

union BUFFERUNION //Convert binary buffer <-> config setup
{
   encoderStruct e;
   byte byte_buffer[sizeof(defaultEncoderStruct)];
} encoder;

const uint8_t controller_id = 2;
uint8_t index = 0;
uint8_t checksum = 0;

void setup() {
  Wire.begin(1);        // join I2C bus (address optional for master)
  Wire.setClock(400000);
  Serial.begin(250000);  // start serial for output
}

void loop() {
  Wire.requestFrom(controller_id, sizeof(defaultEncoderStruct));    // request 6 bytes from slave device #8
  index = 0;
  checksum = 0;
  Wire.readBytes(encoder.byte_buffer, sizeof(defaultEncoderStruct)); //Readbytes needed for correct parsing for struct
  index = sizeof(defaultEncoderStruct);
  while(index--) checksum += encoder.byte_buffer[index];
  if(checksum) Serial.println("ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  else{
    // for(index=0; index<sizeof(defaultEncoderStruct); index++){
    //   Serial.print(encoder.byte_buffer[index]); 
    //   Serial.print(" "); 
    // }
    // Serial.println();
    Serial.print(encoder.e.encoder_pos[0]); 
    Serial.print(" "); 
    Serial.print(encoder.e.encoder_pos[1]); 
    Serial.print(" "); 
    Serial.println(encoder.e.command);
  } 
  delay(100);
}
