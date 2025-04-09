#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include <EEPROM.h>
#include "PacketSerial.h"

#pragma pack(1) //Remove alignment padding bytes in structs - https://forum.pjrc.com/threads/50536-problem-with-union-in-Teensy-3-5
struct configurationStruct{ //65 bytes
  char controller_name[16]; //Name of LED driver: "default name"
  char encoder_name[2][16]; //Name of LED driver: "default name"
  float knob_rates[2][4]; //Different rates the knob gamma is applied as the knob turns
  uint8_t led_intensity[2]; //Indicator LED intensities when off and on
  uint8_t update_interval; //rate to send updates to GUI
  uint8_t checksum;
};

const struct defaultConfigurationStruct{ //65 bytes
  char controller_name[16] = "Unnamed driver "; //Name of LED driver: "default name"
  char encoder_name[2][16] = {"Left Encoder   ", "Right Encoder  "}; //Name of LED driver: "default name"
  float knob_rates[2][4] = {{1.01, 1.0001, 0, 0}, {1.01, 1.0001, 0, 0}}; //Different rates the knob gamma is applied as the knob turns
  uint8_t led_intensity[2] = {1, 255}; //Indicator LED intensity when off and on
  uint8_t update_interval = 10; //rate to send updates to GUI
  uint8_t checksum = 253;
} defaultConfig;

struct encoderStruct{
  uint8_t command; //button presses (bit 0 = left push, bit 1 = right push, bit 2 = left enc, bit 3 = right enc., bit 4 = left LED on, bit 5 = right LED on, bit 6 = built-in LED on.)
  int16_t encoder_pos[2]; //Encoder PWM
};

//Command:
//LSB: left_push, right_push, left_enc_button, right_enc_button, X, X, X, X
const struct defaultEncoderStruct{
  uint8_t command = 0; //button presses (bit 0 = right push, bit 1 = left push, bit 2 = right enc, bit 3 = left enc.)
  int16_t encoder_pos[2] = {0,0}; //Encoder PWM or command data
} defaultEncoder;

const struct prefixStruct{
  uint8_t message = 0; //Send error message to GUI or empty command confirming connection is still good (heartbeat)
  uint8_t magic_number = 1; //Recv magic number at connection start and confirm with magic reply
  uint8_t send_config = 2; //Get driver configuration
  uint8_t recv_config = 3; //Set driver configuration
  uint8_t send_id = 8; //Send the driver ID only
  uint8_t encoder_status = 12; //Send current encoder status
  uint8_t disconnect = 14; //Computer has disconnected from controller
  uint8_t set_led = 18; //Set LED intenssity
} prefix;

union CONFIGUNION //Convert binary buffer <-> config setup
{
   configurationStruct c;
   byte byte_buffer[sizeof(defaultConfigurationStruct)];
} conf;

union BUFFERUNION //Convert binary buffer <-> config setup
{
   encoderStruct e;
   byte byte_buffer[sizeof(defaultEncoderStruct)];
} encoder;

const static uint8_t COBS_BUFFER_SIZE = 255; //Size of the COBS buffer
const static uint16_t HEARTBEAT_TIMEOUT = 10000;
const static char MAGIC_RECEIVE[] = "p6hGvGAKtyRehDZMM0VO"; //Magic number received from GUI to verify this is an LED driver
char MAGIC_SEND[] = "-1UltmSfFUudnRfC1Y923"; //Magic number received from GUI to verify this is an LED driver
const uint8_t pinf_mask = B11110011; //Mask for pins used on portf
const uint8_t pinb_mask = B00010010; //Mask for pins used on portb
const uint8_t en_raw_mask = B11010010; //Mask for encoder pins on portf
const uint8_t sw_raw_mask[] = {B00100000, B00000001}; //Mask for encoder switch pins on portf
const uint8_t button_mask[] = {B00010000, B00000010}; //Mask for button pins on portb
const uint8_t en_pin_mask[][2] = {{B00000010, B00010000}, {B01000000, B10000000}}; //Masks for individial encoder quadrature pins on portf
const uint8_t en_order[] = {0, 1, 3, 2}; //Order of encoder quadrature values goign CW
const uint16_t DEBOUNCE = 40; //Switch debounce time (ms)
const uint8_t pin_button[] ={8, 15}; //button pin #
const uint8_t pin_led[] = {9, 10}; //button led pin #
char temp_buffer[COBS_BUFFER_SIZE]; //Temporary buffer for preparing packets immediately before transmission
uint8_t temp_size; //Size of current data packet on temp buffer
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
  Serial.begin(115200);

  //Load default structs
  memcpy(conf.byte_buffer, &defaultConfig, sizeof(conf.byte_buffer));
  memcpy(encoder.byte_buffer, &defaultEncoder, sizeof(encoder.byte_buffer));

  //Set encoder pins
  for(i=18; i<24; i++) pinMode(i, INPUT_PULLUP);

  //Set switch pins
  for(i=0; i<2; i++){
    pinMode(pin_button[i], INPUT_PULLUP);
    pinMode(pin_led[i], OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  cur_pinf = PINF & pinf_mask;
  prev_pinf = cur_pinf;
  cur_pinb = PINB & pinb_mask;
  prev_pinb = cur_pinb;
  usb.setStream(&Serial);
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
      sendUpdate();
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
      encoder.e.command |= command_mask;
      delay(DEBOUNCE);
    }
    else if((cur_pinb & button_mask[i]) && (encoder.e.command & command_mask)){ //If button was just released
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
        delay(DEBOUNCE);
    }
    else if((sw_raw_mask[i] & cur_pinf) && (encoder.e.command & command_mask)){ //If button was just released
      encoder.e.command &= ~command_mask;
      delay(DEBOUNCE);
    }
  }
}

void checkEncoder(){
  cur_en_raw = cur_pinf & en_raw_mask; //Extract encoder a and b pin states
  if(cur_en_raw != prev_en_raw){ //If encoder posistions changed
    prev_en_raw = cur_en_raw;
    
    //Decode the encoder
    cur_en[0] = cur_en_raw >> 6;
    cur_en[1] = (cur_en_raw & en_pin_mask[0][0])>>1;
    cur_en[1] += (cur_en_raw & en_pin_mask[0][1])>>3;
    
    
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
/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL/////////////////SERIAL
static void onPacketReceived(const uint8_t* buffer, size_t size){
  // Route decoded packet based on prefix byte
  heartbeat = 0; //Reset heartbeat timer as a serial packet has been received
  uint8_t buffer_prefix = buffer[0];
  if(buffer_prefix == prefix.message) serial_connection_active = true; //Start/continue sending status packets; 
  else if(buffer_prefix == prefix.magic_number) magicExchange(buffer, size);
  else if(buffer_prefix == prefix.send_config) sendConfiguration();
  else if(buffer_prefix == prefix.recv_config) recvConfiguration(buffer, size);
  else if(buffer_prefix == prefix.send_id) sendId();
  else if(buffer_prefix == prefix.encoder_status) sendUpdate();
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
      usb.send(MAGIC_SEND, sizeof(MAGIC_SEND)-1); //-1 to remove null at end of string
    }
  }
}

static void setLed(const uint8_t* buffer, size_t size){
  if(size == 4){
    analogWrite(pin_led[0], conf.c.led_intensity[buffer[1]]);
    analogWrite(pin_led[1], conf.c.led_intensity[buffer[2]]);
    digitalWriteFast(LED_BUILTIN, buffer[3]);

    uint8_t mask = B00010000;
    for(uint8_t a=0; a<size-1; a++){
      if(buffer[a+1]) encoder.e.command |= mask;
      else encoder.e.command &= ~mask;
      mask <<= 1;
    }
  }
}

static void sendConfiguration(){
  initializeConfigurations();
  memcpy(temp_buffer+1, conf.byte_buffer, sizeof(conf.byte_buffer));
  temp_buffer[0] = prefix.send_config;
  usb.send((const unsigned char*) temp_buffer, sizeof(conf.byte_buffer)+1); //Send controller info
}

static void recvConfiguration(const uint8_t* buffer, size_t size){
  uint8_t checksum = 0;
  temp_size = 0;
  if(size == sizeof(conf.byte_buffer)+1){
    for(int a = 0; a<(int) size-1; a++) checksum += buffer[a];
    if(!checksum){
      memcpy(conf.byte_buffer, buffer, sizeof(conf.byte_buffer));
      conf.byte_buffer[0] = prefix.send_config; //Switch prefix to sending prefix
      conf.byte_buffer[size-1] += (prefix.recv_config - prefix.send_config); //Fix corresponding checksum
      for(int a = 0; a<(int) size; a++) EEPROM.update(a + sizeof(MAGIC_RECEIVE), conf.byte_buffer[a]); //Copy configuration to EEPROM
      initializeConfigurations(); //Re-run the setup routine to update driver state
      temp_size = sprintf(temp_buffer, "-Configuration file was successfully uploaded.\nAlso upload \"Sync\" settings to apply changes.");
    }
    else temp_size = sprintf(temp_buffer, "-Error: Controller check sum is non-zero: %d", checksum); 
  }
  else temp_size = sprintf(temp_buffer, "-Error: Controller config packet is wrong size. Expected %d, got %d.", sizeof(conf.byte_buffer), size);
    
  if(temp_size){
    temp_buffer[0] = prefix.message;
    usb.send((const unsigned char*) temp_buffer, temp_size);
  }
}

static void sendId(){
  char controller_id[sizeof(conf.c.controller_name)+1];
  for(uint32_t a=0; a<sizeof(conf.c.controller_name); a++){
    controller_id[a+1] = conf.c.controller_name[a];
  }
  controller_id[0] = prefix.send_id;
  usb.send((const unsigned char*) controller_id, sizeof(controller_id));
}

static void sendUpdate(){
  update_timer = 0; //Reset update interval timer
  memcpy(temp_buffer+1, encoder.byte_buffer, sizeof(defaultEncoderStruct)); //Copy controller info to temp buffer
  temp_buffer[0] = prefix.encoder_status;
  usb.send((const unsigned char*) temp_buffer, sizeof(defaultEncoderStruct)+1); //Send controller info
  encoder.e.encoder_pos[0] = 0; //Zero encoder positions
  encoder.e.encoder_pos[1] = 0;
}

static void disconnect(){
  if(heartbeat > HEARTBEAT_TIMEOUT){
    temp_size = sprintf(temp_buffer, "-Error: Heartbeat timed out at %d ms, exceeding the %d ms cutoff. Controller is disconnecting", heartbeat, HEARTBEAT_TIMEOUT);
    temp_buffer[0] = prefix.message;
    usb.send((const unsigned char*) temp_buffer, temp_size);
  }
  temp_buffer[0] = prefix.disconnect; //Send disconnect command
  usb.send((const unsigned char*) temp_buffer, 1);
  serial_connection_active = false; //Stop sending status packets
}

//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM//////////////EEPROM

//Check EEPROM to see if it has a saved configuration
void initializeConfigurations(){
  int a;
  uint8_t check_sum;
  uint16_t buffer_size = sizeof(MAGIC_RECEIVE);
  uint16_t EEPROM_address;
 
  //See if EEPROM has magic number
  for(a=0; a<buffer_size; a++){
    if(EEPROM.read(a) != MAGIC_RECEIVE[a]){
      break;
    }
  }
  //Verify EEPROM checksums
  if(a==buffer_size){
    auto verifyChecksum = [&] (){
      check_sum = 0;
      while(buffer_size--){
        check_sum += EEPROM.read(EEPROM_address--);
      }
    };   
    //Verify EEPROM check sums
    EEPROM_address = sizeof(MAGIC_RECEIVE) + sizeof(conf.byte_buffer) - 1;
    buffer_size = sizeof(conf.byte_buffer);
    verifyChecksum();
    if(!check_sum){
      loadEEPROMtoStructs();
    }
    else loadDefaultsToEEPROM();
  }
  else loadDefaultsToEEPROM();
}

//https://forum.arduino.cc/index.php?topic=42850.0
void loadDefaultsToEEPROM(){
  uint8_t *buffer_ptr;
  uint16_t buffer_size;
  uint16_t EEPROM_address = 0;
  // char message[] = "-A valid driver configuration was not found on EEPROM, so default settings will be loaded.";
  // message[0] = prefix.message;
  // usb.send((const unsigned char*) message, sizeof(message));
  
  //Lambda functions in C++11 rock! https://stackoverflow.com/questions/4324763/can-we-have-functions-inside-functions-in-c
  auto loadEEPROM = [&] (){
    while(buffer_size--){
      EEPROM.update(EEPROM_address++, *buffer_ptr++);
    }
  };
  buffer_ptr = (uint8_t *)&MAGIC_RECEIVE;
  buffer_size = sizeof(MAGIC_RECEIVE);
  loadEEPROM();
  buffer_ptr = (uint8_t *)&defaultConfig;
  buffer_size = sizeof(conf.byte_buffer);
  loadEEPROM();
  loadEEPROMtoStructs();
}

void loadEEPROMtoStructs(){
    uint16_t EEPROM_address = sizeof(MAGIC_RECEIVE) + sizeof(conf.byte_buffer);
    uint16_t buffer_size = sizeof(conf.byte_buffer);

    buffer_size = sizeof(conf.byte_buffer);
    while(buffer_size) conf.byte_buffer[--buffer_size] = EEPROM.read(--EEPROM_address);
}


