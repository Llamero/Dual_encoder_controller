#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "PacketSerial.h"

#pragma pack(1) //Remove alignment padding bytes in structs - https://forum.pjrc.com/threads/50536-problem-with-union-in-Teensy-3-5
struct encoderStruct{
  uint8_t command; //button presses (bit 0 = left push, bit 1 = right push, bit 2 = left enc, bit 3 = right enc.)
  int16_t encoder_pos[2]; //Encoder PWM
};
//Command:
//LSB: left_push, right_push, left_enc_button, right_enc_button, X, X, X, X
const struct defaultEncoderStruct{
  uint8_t command = 0; //button presses (bit 0 = right push, bit 1 = left push, bit 2 = right enc, bit 3 = left enc.)
  int16_t encoder_pos[2] = {0,0}; //Encoder PWM or command data
} defaultEncoder;

const struct prefixStruct{
  uint8_t heartbeat = 0; //Empty command confirming connection is still good
  uint8_t magic_number = 1; //Recv magic number at connection start and confirm with magic reply
  uint8_t update_interval = 2; //Rate to send position updates to computer
  uint8_t set_led = 3; //Change LED intensity
  uint8_t disconnect = 4; //Computer has disconnected from controller
} prefix;

union BUFFERUNION //Convert binary buffer <-> config setup
{
   encoderStruct e;
   byte byte_buffer[sizeof(defaultEncoderStruct)];
} encoder;

const static uint32_t COBS_BUFFER_SIZE = 110; //Size of the COBS buffer
const static uint16_t HEARTBEAT_TIMEOUT = 10000;
const static char MAGIC_RECEIVE[] = "p6hGvGAKtyRehDZMM0VO"; //Magic number received from GUI to verify this is an LED driver
char MAGIC_SEND[] = "-1UltmSfFUudnRfC1Y923"; //Magic number received from GUI to verify this is an LED driver
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
char temp_buffer[COBS_BUFFER_SIZE]; //Temporary buffer for preparing packets immediately before transmission
uint8_t update_interval = 10; //Time in ms between position packets sent to computer
bool serial_connection_active = false; //Whether there is an active serial connection to the computer
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
uint8_t led_intensity[] = {255, 255}; //LED on intensities
uint8_t command_mask; //Mask for flipping individual bits in the command byte

PacketSerial_<COBS, 0, COBS_BUFFER_SIZE> usb; //Sets Encoder, framing character, buffer size
elapsedMillis update_timer;
elapsedMillis heartbeat;

void setup() {
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

  usb.begin(115200);
  usb.setPacketHandler(&onPacketReceived);

}

void loop() {
  while(serial_connection_active){
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
    if(heartbeat >= HEARTBEAT_TIMEOUT) disconnect();
    if(update_timer >= update_interval){
      update_timer = 0; //Reset update interval timer
      memcpy(temp_buffer, encoder.byte_buffer, sizeof(defaultEncoderStruct)); //Copy controller info to temp buffer
      usb.send((const unsigned char*) temp_buffer, sizeof(defaultEncoderStruct)); //Send controller info
      encoder.e.encoder_pos[0] = 0; //Zero encoder positions
      encoder.e.encoder_pos[1] = 0;
      usb.update(); //Check if a command was received
    }
  }
  usb.update(); //If serial is not active, monitor the usb connection
  delay(10);
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
}

void checkSwitch(){
  for(i=0; i<2; i++){
    command_mask = B00000100 << i;
    if(!(sw_raw_mask[i] & cur_pinf) && !(encoder.e.command & command_mask)){ //if button was just pressed
        encoder.e.command |= command_mask;
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
  cur_en_raw = cur_pinf & en_raw_mask; //Extract encoder a and b pin states
  if(cur_en_raw != prev_en_raw){ //If encoder posistions changed
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
      if(cur_en[i] == en_order[j]); //No rotation
      else if(cur_en[i] == en_order[j_f]){ //Encoder turned CW
        encoder.e.encoder_pos[i]++;
        en_order_index[i]++;
      }
      else if(cur_en[i] == en_order[j_r]){ //Encoder turned CCW
        encoder.e.encoder_pos[i]--;
        en_order_index[i]--; 
      }
      else{ //Encoder skipped step
        en_order_index[i] += 2; 
      }
    }
  }
}

static void onPacketReceived(const uint8_t* buffer, size_t size){
  // Route decoded packet based on prefix byte
  heartbeat = 0; //Reset heartbeat timer as a serial packet has been received
  uint8_t buffer_prefix = buffer[0];
  if(buffer_prefix == prefix.heartbeat) serial_connection_active = true; //Start/continue sending status packets; 
  else if(buffer_prefix == prefix.magic_number) magicExchange(buffer, size);
  else if(buffer_prefix == prefix.update_interval) setUpdateInterval(buffer, size);
  else if(buffer_prefix == prefix.set_led) setLed(buffer, size);
  else if(buffer_prefix == prefix.disconnect) disconnect();
}

static void magicExchange(const uint8_t* buffer, size_t size){
  uint32_t a;
  if(size == sizeof(MAGIC_RECEIVE)){
    for(a=0; a<size; a++){
      if(buffer[a+1] != MAGIC_RECEIVE[a]){
        break;
      }
    }
    if(a==size-1){
      MAGIC_SEND[0] = prefix.magic_number;
      usb.send((const unsigned char*) MAGIC_SEND, size);
    }
  }
}

static void setUpdateInterval(const uint8_t* buffer, size_t size){
  update_interval = buffer[1];
}

static void setLed(const uint8_t* buffer, size_t size){
  analogWrite(pin_led[0], buffer[1]);
  analogWrite(pin_led[1], buffer[2]);
}

static void disconnect(){
  serial_connection_active = false; //Stop sending status packets
}




