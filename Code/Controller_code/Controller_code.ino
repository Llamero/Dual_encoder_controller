#include <digitalWriteFast.h>
#include <Wire.h>
#pragma pack(1) //Remove alignment padding bytes in structs - https://forum.pjrc.com/threads/50536-problem-with-union-in-Teensy-3-5
struct encoderStruct{
  uint16_t encoder_pos[2]; //Encoder PWM
  uint8_t command; //Instruction from controller
  uint8_t checksum; //Checksum
};
//Command:
//LSB: left_push, right_push, left_enc_button, right_enc_button, X, X, X, X
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

const float scale[] = {1.001,1.01}; //Scales by which the intensity changes per unit knob turn - pushing encoder switches scales
const uint8_t pinf_mask = B11110011; //Mask for pins used on portf
const uint8_t pinb_mask = B00010010; //Mask for pins used on portb
const uint8_t en_raw_mask = B11010010; //Mask for encoder pins on portf
const uint8_t sw_raw_mask[] = {B00000001, B00100000}; //Mask for encoder switch pins on portf
const uint8_t button_mask[] = {B00010000, B00000010}; //Mask for button pins on portb
const uint8_t en_pin_mask[][2] = {{B00000010, B00010000}, {B01000000, B10000000}}; //Masks for individial encoder quadrature pins on portf
const uint8_t en_order[] = {0, 1, 3, 2}; //Order of encoder quadrature values goign CW
const uint16_t DEBOUNCE = 40; //Switch debounce time (ms)
const uint8_t pin_sw[] = {15, 8}; //button pin #
const uint8_t pin_led[] = {9, 10}; //button led pin #
const uint8_t device_id = 2;
uint8_t i; //index
uint8_t j; //index #2
uint8_t j_f; //Forward index #2
uint8_t j_r; //Reverse index #2
uint8_t cur_pinf; //Current state of portf
uint8_t prev_pinf; //Previous state of portf
uint8_t cur_pinb; //Current state of portb
uint8_t prev_pinb; //Previous state of portb
uint8_t cur_en_raw; //Raw bit state of quadrature pins on portf
uint8_t prev_en_raw; //Previous raw bit state of quadrature pins on portf
uint8_t en_order_index[2]; //index of encoders in encoder_order array - tracks encoder direction
uint8_t cur_en[2]; //Current value of encoder - bits shifted so that A is b0 and B is b1
float float_en_pos[] = {1,1}; //Floating point value of encoder - allows for gamma curve to LED control
uint8_t scale_index; //index of current scale to be used
uint8_t command_mask; //Mask for flipping individual bits in the command byte

void setup() {
  Wire.begin(device_id);                // join I2C bus with address #8
  Wire.onRequest(requestEvent); // register event
  
  //Set encoder pins
  for(i=18; i<24; i++) pinMode(i, INPUT_PULLUP);

  //Set switch pins
  for(i=0; i<2; i++){
    pinMode(pin_sw[i], INPUT_PULLUP);
    pinMode(pin_led[i], OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  cur_pinf = PINF & pinf_mask;
  prev_pinf = cur_pinf;
  cur_pinb = PINB & pinb_mask;
  prev_pinb = cur_pinb;

 //Serial.begin(250000);
}

void loop() {
  while(true){
    cur_pinf = PINF & pinf_mask; //check encoder
    cur_pinb = PINB & pinb_mask; //check pushbuttons
    if(cur_pinf != prev_pinf){ //if encoder changed
      prev_pinf = cur_pinf;
      checkEncoder();
      checkSwitch();
    }
    if(cur_pinb != prev_pinb){ //if pushbuttons changed
      prev_pinb = cur_pinb;
      checkButton();
    }
  }
}

void requestEvent() {
  encoder.e.checksum = 0;
  for(i=0; i<sizeof(defaultEncoderStruct) - 1; i++) encoder.e.checksum += encoder.byte_buffer[i];
  encoder.e.checksum = 0-encoder.e.checksum; 
  Wire.write(encoder.byte_buffer, sizeof(defaultEncoderStruct)); // respond with message of 6 bytes
  // as expected by master
}

void checkButton(){
  for(i=0; i<2; i++){
    command_mask = B00000001 << i;
    if(!(cur_pinb & button_mask[i]) && !(encoder.e.command & command_mask)){ //if button was just pressed
      digitalWriteFast(pin_led[i], HIGH);
      encoder.e.command |= command_mask;
      delay(DEBOUNCE);
    }
    else if((cur_pinb & button_mask[i]) && (encoder.e.command & command_mask)){ //If button was just released
      digitalWriteFast(pin_led[i], LOW);
      encoder.e.command &= ~command_mask;
      delay(DEBOUNCE);
    }
  }
  if(!(cur_pinb & pinb_mask)){ //If both buttons are pressed
    digitalWriteFast(LED_BUILTIN, HIGH);
  }
  else digitalWriteFast(LED_BUILTIN, LOW);
}

void checkSwitch(){
  for(i=0; i<2; i++){
    command_mask = B00000100 << i;
    if(!(sw_raw_mask[i] & cur_pinf) && !(encoder.e.command & command_mask)){ //if button was just pressed
        encoder.e.command |= command_mask;
        scale_index++;
        scale_index %= (sizeof(scale)/sizeof(scale[0]));
        digitalWriteFast(LED_BUILTIN, HIGH);
        delay(DEBOUNCE);
    }
    else if((sw_raw_mask[i] & cur_pinf) && (encoder.e.command & command_mask)){ //If button was just released
      encoder.e.command &= ~command_mask;
      digitalWriteFast(LED_BUILTIN, LOW);
      delay(DEBOUNCE);
    }
  }
}

void checkEncoder(){
  cur_en_raw = cur_pinf & en_raw_mask; //Extrace encoder a and b pin states
  if(cur_en_raw != prev_en_raw){
    prev_en_raw = cur_en_raw;
    
    //Decode the encoder
    cur_en[0] = (cur_en_raw & en_pin_mask[0][0])>>1;
    cur_en[0] += (cur_en_raw & en_pin_mask[0][1])>>3;
    cur_en[1] = cur_en_raw >> 6;
    
    //Find direction of rotation
    for(i=0; i<2; i++){
      j = en_order_index[i]%4;
      j_f = j+1; //Forward and reverse indeces must be calculated step wise to prevent sign problems
      j_f %= 4;
      j_r = j-1;
      j_r %= 4;
      if(cur_en[i] == en_order[j]);
      else if(cur_en[i] == en_order[j_f]){ //Encoder turned CW
        en_order_index[i]++;
        if(encoder.e.encoder_pos[i] < 65535){ //If encoder isn't at max value
          float_en_pos[i] *= scale[scale_index];
          if(float_en_pos[i] > 65535) float_en_pos[i] = 65535;
          encoder.e.encoder_pos[i] = round(float_en_pos[i]);
        }
        else{
          encoder.e.encoder_pos[i] = 65535;
          float_en_pos[i] = 65535;
        } 
      }
      else if(cur_en[i] == en_order[j_r]){ //Encoder turned CCW
        en_order_index[i]--;
        if(encoder.e.encoder_pos[i] > 1){
          float_en_pos[i] /= scale[scale_index];
          if(float_en_pos[i] < 1) float_en_pos[i] = 1;
          encoder.e.encoder_pos[i] = round(float_en_pos[i]);
        }
        else{
          encoder.e.encoder_pos[i] = 1;
          float_en_pos[i] = 1;
        } 
      }
      else{ //Encoder skipped step
        en_order_index[i] += 2; 
      }
    }
  }
}







