#include <LPD8806.h>
#include <FastLED.h>
#include <QueueArray.h>
#include <Wire.h>
/**
 * Pinball 2000 Driver Board Controller
 *
 * (C) 2013 - Jimmy Lipham <jimmy@catster.net>
 * AKA "Compy"
 */

// PortD 0
//       1 20
//       2 19

// Strip 1: Data pin is 3
//          Clock pin is 2

// Data/Clock inputs are on male side of strip
// Clock = Green, Data = White

// Orange = Data,  Green = Clock

// Strip 2: Data pin is 5
//          Clock pin is 4

// Strip 3: Data pin is 42
//          Clock pin is 44

// Strip 4: Data pin is 48
//          Clock pin is 46

#include "p2k.h"

#define LED_PIN     2
#define NUM_LEDS    50
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define LED_UPDATES_PER_SECOND  10

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

QueueArray<P2KCommand> eventq;

unsigned long last_heartbeat;
unsigned long last_hello;
unsigned long cnt;
unsigned long switch_tick_time;
unsigned long switch_gap_time;
void(* resetFunc) (void) = 0;
void setup()
{
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(  BRIGHTNESS );

  TWBR = ((F_CPU / 850000) - 16) / 2;
  
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
  Serial.begin(57600);
  
  while (!Serial) {
  
  }
  
  /*
  Serial.println(F("P2K controller running in debug mode..."));
   
   Serial.print(F("Struct action size: "));
   Serial.println(sizeof(action));
   
   Serial.print(F("Struct switchrule size: "));
   Serial.println(sizeof(switchrule));
   */

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  /*

  DDRA = B11111111;
  DDRC = B11111111;
  
  strip1.begin();
  strip2.begin();
  strip3.begin();
  strip4.begin();

  strip1.show();
  strip2.show();
  strip3.show();
  strip4.show();

  for(int i=0; i<strip2.numPixels(); i++) strip2.setPixelColor(i, strip2.Color(80,80,80));
  for(int i=0; i<strip3.numPixels(); i++) strip3.setPixelColor(i, strip3.Color(80,80,80));
  */
  
  
  hello_processed = false;

  //delay(2000);

  //Serial.println("P2K Controller");
  
  // Send hello packet to host
  #ifndef DEBUG
  
  #endif

}

void setLEDState(byte chain, byte ledNum, byte state) {
  unsigned long begin_time = micros();
  Wire.begin();
  Wire.beginTransmission(8);
  Wire.write(1);
  Wire.write(chain);
  Wire.write(ledNum);
  Wire.write(state);
  Wire.endTransmission();
  Wire.end();
  stats_led_exec_length = micros() - begin_time;
}

void disableALLLEDs() {
  Wire.begin();
  Wire.beginTransmission(8);
  Wire.write(0);
  Wire.write(0);
  Wire.write(0);
  Wire.write(0);
  Wire.endTransmission();
  Wire.end();
}

void setLEDMode(byte mode) {
  Wire.begin();
  Wire.beginTransmission(8);
  Wire.write(1);
  Wire.write(mode);
  Wire.write(0);
  Wire.write(mode);
  Wire.endTransmission();
  Wire.end();
}

void init_driverboard()
{
  for (sw_col = 0; sw_col < 8; sw_col++) switch_matrix[sw_col] = 0;
  for (int i = 0; i < NUM_SOLENOIDS; i++) solenoid_action[i].mode = ACTION_OFF;
  for (int i = 0; i < NUM_SWITCHRULES; i++) switch_rules[i].switch_num = 0;
  for (int i = 0; i < 16; i++) bank_states[i] = 0;
  for (int i = 0; i < 8; i++) { 
    matrix_a_states[i] = 0; 
    matrix_b_states[i] = 0; 
  }
  p = new Parallel();
  p->init_parport();

  //pinMode(13, OUTPUT);

  p->send_data(REGISTER_SOLENOID_D, 0x0);
  p->send_data(REGISTER_SOLENOID_A, 0x0);
  p->send_data(REGISTER_SOLENOID_B, 0x0);
  p->send_data(REGISTER_SOLENOID_C, 0x0);
  p->send_data(REGISTER_SOLENOID_FLIPPER, 0x0);
  p->send_data(REGISTER_SOLENOID_LOGIC, 0x0);

  last_heartbeat = millis();

  sw_col = 0;
  sw_bank_cnt = 0;
  cnt = 0;
  switch_tick_time = 0;
  last_switch_tick = 0;
  last_lamp_tick = 0;
  last_solenoid_tick = 0;

  lamp_strength_cnt = 0;


  sol_strength_cnt = 0;
}

byte get_lamp_idx(int matrix, int col, int row)
{
  /*
  Serial.print("Matrix: ");
   Serial.print(matrix);
   Serial.print(" col ");
   Serial.print(col);
   Serial.print(" row ");
   Serial.print(row);
   Serial.print(" ");
   Serial.println(((col * 8) + row) + (matrix * 64));
   */
  return ((col * 8) + row) + (matrix * 64);
}

byte lamp2driveridx(byte driver)
{
  int col = driver / 16;
  int matrix = 0;
  if (col > 7)
  {
    matrix = 1;
    col -= 8;
  }

  int row = driver - ((driver / 16) * 16);
  /*
  Serial.print("lamp2driveridx(");
   Serial.print(driver);
   Serial.print(") col");
   Serial.print(col);
   Serial.print(" row ");
   Serial.print(row);
   Serial.print(" Matrix ");
   Serial.println(matrix);
   */
  return get_lamp_idx(matrix, col, row);
}
int current_led_state = 0;
unsigned long i_tick_lamps = TICK_LAMPS;
void loop()
{
  //Serial.println("HI");

  time = millis();
  uTime = micros();

  if (uTime - last_led_tick >= TICK_LEDS) {
    /*
    ChangePalettePeriodically();
      
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1;
    
    FillLEDsFromPaletteColors( startIndex);
    
    
    //FastLED.show();
    //FastLED.delay(1000);
    */
    //setLEDState(1, 1, current_led_state);

    //current_led_state = (current_led_state == 1) ? 0 : 1;
    
    last_led_tick = uTime;
  }
  
  struct P2KCommand c;
  if (Serial.available() > 0)
  {
    if (StreamSend::receiveObject(Serial, &c, sizeof(struct P2KCommand)) == GOOD_PACKET)
    {
      
      if (c.opcode == CMD_FIRE_LAMP)
      {
        fire_lamp(c.int1, c.int2, c.int3);
      }
      
      else if (c.opcode == CMD_FIRE_SOLENOID)
      {
        fire_solenoid(c.int1, c.int2);
      }
      else if (c.opcode == CMD_ENABLE_LAMP)
      {
        enable_lamp(c.int1, c.int2);
      }
      else if (c.opcode == CMD_ENABLE_SOLENOID)
      {
        enable_solenoid(c.int1);
      }
      else if (c.opcode == CMD_DISABLE_LAMP)
      {
        disable_lamp(c.int1);
      }
      else if (c.opcode == CMD_DISABLE_SOLENOID)
      {
        disable_solenoid(c.int1);
      }
      else if (c.opcode == CMD_SCHEDULE_LAMP)
      {
        schedule_driver((byte)DRIVER_LAMP, (byte)c.int1, c.int2, (byte)c.int3);
      }
      else if (c.opcode == CMD_SCHEDULE_SOLENOID)
      {
        schedule_driver((byte)DRIVER_SOLENOID, (byte)c.int1, c.int2, (byte)c.int3);
      }
      else if (c.opcode == CMD_SET_TICK_RATE)
      {
        i_tick_lamps = c.int1;
      }
      else if (c.opcode == CMD_ENABLE_FLIPPERS)
      {
        enable_flippers();
      }
      
      else if (c.opcode == CMD_GET_SWITCH_STATES)
      {
        // Loop through all switches and send back the state
        for (int rw = 0; rw < 8; rw++)
        {
          outbound_command.int1 = rw;
          outbound_command.int2 = switch_matrix[rw];
          outbound_command.opcode = CMD_GET_SWITCH_STATES;
          outbound_command.int3 = 0;
          outbound_command.int4 = 0;
          send_msg(outbound_command);
        }
      }

      else if (c.opcode == CMD_RESET_COIL_SCHEDULES)
      {
        // Loop through all coils and disable their schedules and disable the coil as well
        for (int cidx = 0; cidx < NUM_SOLENOIDS; cidx++) {
          solenoid_action[cidx].mode = ACTION_OFF;
        }
      }
      
      else if (c.opcode == CMD_DISABLE_FLIPPERS)
      {
        disable_flippers();
      }
      else if (c.opcode == CMD_SET_LED_MODE) {
        setLEDMode(c.int1);
      }
      
      if (c.opcode == CMD_HELLO)
      {
        last_heartbeat = time;
        if (!hello_processed) {
          init_driverboard();
          hello_processed = true;
          schedule_driver((byte)DRIVER_SOLENOID, (byte)0xd4, 0xFF00FF00, (byte)0);
        }
        

        outbound_command.int1 = 0;
        outbound_command.int2 = 0;
        outbound_command.opcode = CMD_HELLO;
        outbound_command.int3 = 0;
        outbound_command.int4 = 0;
        send_msg(outbound_command);

        /*
        last_heartbeat = millis();
        outbound_command.int1 = 0;
        outbound_command.int2 = 0;
        outbound_command.opcode = CMD_HEARTBEAT;
        outbound_command.int3 = 0;
        outbound_command.int4 = 0;
        send_msg(outbound_command); 
        */
      }
      else if (c.opcode == CMD_HEARTBEAT)
      {
        last_heartbeat = time;
        outbound_command.int1 = stats_lamp_exec_length;
        outbound_command.int2 = stats_switch_exec_length;
        outbound_command.opcode = CMD_HEARTBEAT;
        outbound_command.int3 = stats_solenoid_exec_length;
        outbound_command.int4 = stats_led_exec_length;
        send_msg(outbound_command);
              
      }
      /*
      else if (c.opcode == CMD_ADD_SWITCH_RULE)
      {
        //void add_switch_rule(int sw, int sw_type, int state, int driver_type, int driver_num, byte driver_pulse_type, byte strength, byte duration)

        add_switch_rule(c.int1, c.int2, c.int3, c.int4

        add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
        add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);
        add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
        add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);
      
        add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);
        add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);

        
      }
      */
    }
  }
  
  if (!hello_processed) {
    // Send hello up the chain
    if (time - last_hello >= 2000) {
      last_hello = time;
      outbound_command.int1 = 0;
      outbound_command.int2 = 0;
      outbound_command.opcode = CMD_HELLO;
      outbound_command.int3 = 0;
      outbound_command.int4 = 0;
      send_msg(outbound_command);
    }
    return;
  }
  
  

  
  unsigned long task_start_time = 0;
  if (uTime - last_lamp_tick >= TICK_LAMPS)
  {
    task_start_time = uTime;
    lamps_handler();
    stats_lamp_exec_length = micros() - task_start_time;
    last_lamp_tick = uTime;
  }
  else if (uTime - last_switch_tick >= TICK_SWITCHES)
  {
    switch_gap_time = uTime - last_switch_tick;
    task_start_time = uTime;
    switches_handler();
    stats_switch_exec_length = micros() - task_start_time;
    last_switch_tick = uTime;

  }
  else if (uTime - last_solenoid_tick >= TICK_SOLENOIDS)
  {
    task_start_time = uTime;
    solenoids_handler();
    last_solenoid_tick = uTime;
    stats_solenoid_exec_length = micros() - task_start_time;
  }
  
  
  // Has the watchdog timed out?
  
  if (time - last_heartbeat >= WATCHDOG_TIMEOUT)
  {
    
    p->send_data(REGISTER_SOLENOID_D, 0x0);
    p->send_data(REGISTER_SOLENOID_A, 0x0);
    p->send_data(REGISTER_SOLENOID_B, 0x0);
    p->send_data(REGISTER_SOLENOID_C, 0x0);
    p->send_data(REGISTER_SOLENOID_FLIPPER, 0x0);
    p->send_data(REGISTER_SOLENOID_LOGIC, 0x0);
    setLEDMode(0);
    if (hello_processed)
      resetFunc();
    
  }
  
  if (time - last_usb_send >= USB_SEND_INTERVAL_MS) {
    if (!eventq.isEmpty()) {
      P2KCommand c = eventq.dequeue();
      send_msg(c);
    }
    last_usb_send = time;
  }
  
}

// * * * Functions for getting the state of switches, directswitches, lamps and solenoids

byte get_switch(byte sw)
{
  return (switch_matrix[sw >> 4] & (1 << (sw & 0xf))) > 0;
}

byte get_direct_switch(byte sw)
{
  return (direct_switches[sw >> 4] & (1 << (sw & 0xf))) > 0;
}

byte get_lamp(byte lamp)
{
  return lamp_matrix_action[lamp].mode > OFF;
}

byte get_solenoid(byte solenoid)
{
  return solenoid_action[solenoid].mode > OFF;
}

// * * * The main function for starting an action. Usually called from the short cut functions below...

void set_action(byte type, byte num, byte mode, byte strength, byte on_time,
byte off_time, int pulse_max)
{
  struct action *action_ptr;
  RTIME pulse_time;

  if (type == ACTION_LAMP) {
    action_ptr = (struct action*) &lamp_matrix_action[lamp2driveridx(num)];
  }
  else {
    action_ptr = (struct action*) &solenoid_action[bank2idx(num)];
  }

  action_ptr->strength = strength;


  if (action_ptr->strength == 0) {
    action_ptr->strength = 1;
  }

  pulse_time = on_time;
  action_ptr->stop_time = millis() + pulse_time;
  if (mode == ACTION_PULSE) {
    action_ptr->pulse_on_time = pulse_time;
    action_ptr->pulse_off_time = off_time;
    action_ptr->pulse_high = 1;
    action_ptr->pulse_max = pulse_max;
    action_ptr->pulse_cnt = 0;
  }
  else {
    action_ptr->pulse_high = 0;
  }
  action_ptr->mode = mode;
}

// * * * Lamp handling functions

void enable_lamp(byte lamp, byte strength)
{
  set_action(ACTION_LAMP, lamp, ACTION_CONTINOUS, strength, 0, 0, 0);
}

void fire_lamp(byte lamp, byte strength, byte duration)
{
  set_action(ACTION_LAMP, lamp, ACTION_ONE_SHOT, strength, duration, 0, 0);
}

void pulse_lamp(byte lamp, byte strength, byte on_time, byte off_time, int pulse_max)
{
  set_action(ACTION_LAMP, lamp, ACTION_PULSE, strength, on_time, off_time, pulse_max);
}

void disable_lamp(byte lamp)
{
  lamp_matrix_action[lamp2driveridx(lamp)].mode = ACTION_OFF;
}

void disable_lamps()
{
  int i;
  for (i = 0; i < NUM_LAMPS; i++)
  {
    lamp_matrix_action[i].mode = ACTION_OFF;
  }
}

// * * * Solenoid (and flash lamps) handling functions

void enable_solenoid(byte solenoid)
{
  set_action(ACTION_SOLENOID, solenoid, ACTION_CONTINOUS, 1, 0, 0, 0);
}

void fire_solenoid(byte solenoid, byte duration)
{
  set_action(ACTION_SOLENOID, solenoid, ACTION_ONE_SHOT, 1, duration, 0, 0);
}

void pulse_solenoid(byte solenoid, byte on_time, byte off_time, int pulse_max)
{
  set_action(ACTION_SOLENOID, solenoid, ACTION_PULSE, 1, on_time, off_time, pulse_max);
}

void disable_solenoid(byte solenoid)
{
  solenoid_action[bank2idx(solenoid)].mode = ACTION_OFF;
}

void broadcast_event(byte type, byte sw, byte state, byte debounced)
{
  int i;
  // Process direct switch rules
  for (i = 0; i < NUM_SWITCHRULES; i++)
  {
    if (switch_rules[i].switch_num == sw && switch_rules[i].switch_type == type)
    {

      if (switch_rules[i].state == state)
      {
#ifdef DEBUG
        Serial.println("MATCHED SWITCH RULE");
#endif
        // Pulse driver based on type
        if (switch_rules[i].driver_type == DRIVER_LAMP)
        {

          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_ENABLE)
            enable_lamp((byte)switch_rules[i].driver_num, (byte)switch_rules[i].strength);

          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_PULSE)
            fire_lamp((byte)switch_rules[i].driver_num, (byte)switch_rules[i].strength, (byte)switch_rules[i].duration);

          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_DISABLE)
          {
#ifdef DEBUG
            Serial.println("DISABLE LAMP");
#endif
            disable_lamp((byte)switch_rules[i].driver_num);
          }
        }
        if (switch_rules[i].driver_type == DRIVER_SOLENOID)
        {
          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_ENABLE)
            enable_solenoid((byte)switch_rules[i].driver_num);

          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_PULSE)
            fire_solenoid((byte)switch_rules[i].driver_num, (byte)switch_rules[i].duration);

          if (switch_rules[i].driver_pulse_type == RULE_DRIVER_DISABLE)
          {
#ifdef DEBUG
            Serial.println("DISABLE SOLENOID");
#endif
            disable_solenoid((byte)switch_rules[i].driver_num);
          }
        }
      }
    }
  }

  outbound_command.int1 = 0;
  outbound_command.int2 = 0;
  outbound_command.opcode = 0;
  outbound_command.int3 = 0;
  outbound_command.int4 = 0;
  
  if (state == 1)
  {
    if (debounced == 1) outbound_command.opcode = EVT_SWITCHCLOSEDDEBOUNCED;
    else outbound_command.opcode = EVT_SWITCHCLOSEDNONDEBOUNCED;
  }
  else
  {
    if (debounced == 1) outbound_command.opcode = EVT_SWITCHOPENDEBOUNCED;
    else outbound_command.opcode = EVT_SWITCHOPENNONDEBOUNCED;
  }
    
  outbound_command.int1 = sw;
  outbound_command.int2 = type;

/*
  if (type == SWITCH_MATRIX)
  {
    outbound_command.opcode = CMD_OUTBOUND_MATRIX_SWITCH;
    outbound_command.int1 = sw;
    outbound_command.int2 = state;
    outbound_command.int3 = debounced;
    //rt_printk("Matrix switch event sw: %d state %d\n", sw, state);
  }
  else if (type == SWITCH_DIRECT)
  {
    outbound_command.opcode = CMD_OUTBOUND_DIRECT_SWITCH;
    outbound_command.int1 = sw;
    outbound_command.int2 = state;
    outbound_command.int3 = debounced;
    //rt_printk("Direct switch event sw: %d state %d\n", sw, state);
  }
  else if (type == SWITCH_COIN)
  {
    outbound_command.opcode = CMD_OUTBOUND_COIN_SWITCH;
    outbound_command.int1 = sw;
    outbound_command.int2 = state;
    outbound_command.int3 = debounced;
  }
  */
#ifndef DEBUG
  eventq.enqueue(outbound_command);
  //send_msg(outbound_command);
#else
  Serial.print(F("OPCODE: "));
  Serial.print(outbound_command.opcode);
  Serial.print(F(" INT1: "));
  Serial.print(outbound_command.int1);
  Serial.print(F(" INT2: "));
  Serial.print(outbound_command.int2);
  Serial.print(F(" DEBOUNCED: "));
  Serial.print(outbound_command.int3);
  Serial.print(F(" SCHEDULE: "));
  Serial.println(outbound_command.schedule);
#endif
}

void send_msg(struct P2KCommand cmd)
{
  StreamSend::sendObject(Serial, &cmd, sizeof(cmd));
}

int receive_msg(struct P2KCommand *cmd)
{
  return StreamSend::receiveObject(Serial, cmd, sizeof(struct P2KCommand));
}



void lamps_handler()
{
  struct action *action_ptr;
  // * * * Lamp matrix A & B * * *
  lamp_data[0] = lamp_data[1] = 0;
  for (lamp_row = 0; lamp_row < 8; lamp_row++)
  {
    for (lamp_lamp = (lamp_col << 4 | lamp_row); lamp_lamp < 0xFF; lamp_lamp += 0x80)
    {
      action_ptr = (struct action*) &lamp_matrix_action[get_lamp_idx(lamp_lamp >> 7, lamp_col, lamp_row)];
      if (action_ptr->mode == ACTION_SCHEDULE && action_ptr->trash == TRUE)
      {
        action_ptr->trash = FALSE;
        action_ptr->mode = ACTION_OFF;
        continue;
      }

      if (action_ptr->mode == ACTION_SCHEDULE)
      {
        if (CHECK_BIT(action_ptr->schedule, action_ptr->current_bit))
        {
          lamp_data[lamp_lamp >> 7] |= (1 << lamp_row);
        }
        if (time - action_ptr->stop_time >= UPDATE_SCHEDULE_US)
        {
          if (action_ptr->current_bit == 31)
          {
            if (action_ptr->pulse_max > 0)
            {
              if (action_ptr->pulse_cnt == action_ptr->pulse_max)
              {
                action_ptr->trash = TRUE;
              }
              else
                action_ptr->pulse_cnt++;
            }
            action_ptr->current_bit = 0;
          }
          else
            action_ptr->current_bit++;
          action_ptr->stop_time = time;
        }
        continue;
      }

      if (action_ptr->mode == ACTION_OFF) continue;

      if ((action_ptr->mode != ACTION_PULSE || action_ptr->pulse_high) 
        && (lamp_strength_cnt % action_ptr->strength) == 0
        )
      {
        //if ((1 << col) == 1) rt_printk("LAMP %d ON1\n", lamp);
        lamp_data[lamp_lamp >> 7] |= (1 << lamp_row);
        if (action_ptr->mode == ACTION_CONTINOUS) continue;
      }
      else
      {
        //rt_printk("LAMP %d OFF1\n", lamp);
      }
      if (time > action_ptr->stop_time)
      {
        if (action_ptr->mode == ACTION_PULSE)
        {
          if (action_ptr->pulse_high == 0) { 
            action_ptr->pulse_cnt++; 
          }
          action_ptr->pulse_high = !action_ptr->pulse_high;
          action_ptr->stop_time = time +
            (action_ptr->pulse_high ? action_ptr->pulse_on_time : action_ptr->pulse_off_time);
        }
        if (action_ptr->mode != ACTION_PULSE || (action_ptr->mode == ACTION_PULSE &&
          action_ptr->pulse_cnt >= action_ptr->pulse_max && action_ptr->pulse_max > 0))
        {
          if (action_ptr->mode != ACTION_SCHEDULE) action_ptr->mode = ACTION_OFF;
        }
      }
    }
  }
  // Send lamp data to the power driver board
  noInterrupts();
  p->send_data(REGISTER_LAMP_COLUMN,1 << lamp_col);
  p->send_data(REGISTER_LAMP_ROW_A,lamp_data[0]);
  p->send_data(REGISTER_LAMP_ROW_B,lamp_data[1]);
  interrupts();
  lamp_col++;
  if (lamp_col >= 8) lamp_col = 0;

  lamp_strength_cnt = ++lamp_strength_cnt % 7;
}

void switches_handler()
{
  byte direct_switch_banks[] = {
    REGISTER_SWITCH_FLIPPER, REGISTER_SWITCH_MISC, REGISTER_SWITCH_COIN, REGISTER_SWITCH_ZERO,
    REGISTER_SWITCH_DIP, REGISTER_LAMP_A_DIAGNOSTIC, REGISTER_LAMP_B_DIAGNOSTIC,
    REGISTER_FUSE_A_DIAGNOSTIC, REGISTER_FUST_B_DIAGNOSTIC        };

  sw_col_val = (1 << sw_col);
  p->send_data(REGISTER_SWITCH_COLUMN, sw_col_val);
  sw_data = p->receive_data(REGISTER_SWITCH_ROW);
  /*
  Serial.print("sw_col_val: ");
   Serial.print(sw_col_val);
   Serial.print(" sw_data: ");
   Serial.print(sw_data);
   Serial.print(" matrix: ");
   Serial.print(switch_matrix[sw_col]);
   Serial.println("");
   */

  // The column has changed, loop through to find out which switches have changed
  //if (sw_data != switch_matrix[sw_col])
  //{
    for (sw_row = 0; sw_row < 8; sw_row++)
    {
      sw_bit = 1 << sw_row;
      sw_sw = (sw_col << 4) + sw_row;
      
      // Is the change bit set for this and is the switch state the same as the last time, then fire a debounce event
      // and unset the change bit
      if ((sw_bit & switch_matrix_changing[sw_col]) && (sw_bit & sw_data) == (sw_bit & switch_matrix[sw_col]))
      {
        switch_matrix_changing[sw_col] -= sw_bit;
        broadcast_event(SWITCH_MATRIX, sw_sw, (sw_bit & sw_data) != 0, 1);
      }
    
      
      // The state has changed
      // If the change bit is set for this, and the state has changed, unset the change bit
      // If the change bit is set for this, and the state has NOT changed, fire a debounce event
      // If the change bit is not set for this, set the change bit
      if ((sw_bit & sw_data) != (sw_bit & switch_matrix[sw_col]))
      {
        
        
        // Is the change bit set? The state has also changed
        if ((sw_bit & switch_matrix_changing[sw_col]))
        {
          // Unset the change bit
          switch_matrix_changing[sw_col] -= sw_bit;
        }
        else
        {
          // Set the change bit
          switch_matrix_changing[sw_col] += sw_bit;
        }

        if ((sw_bit & sw_data) == 0) switch_matrix[sw_col] -= sw_bit;
        else switch_matrix[sw_col] += sw_bit;
        broadcast_event(SWITCH_MATRIX, sw_sw, (sw_bit & sw_data) != 0, 0);
      }
    }
  //}

  // Direct switches -- Only check every few columns
  if (sw_col % 2 == 0)
  {
    sw_bank = direct_switch_banks[sw_bank_cnt];
    sw_data = p->receive_data(sw_bank);
    // Check if any switches have changed since last time
    //if (sw_data != direct_switches[sw_bank])
    //{
      for (sw_switches = 0; sw_switches < 8; sw_switches++)
      {
        sw_bit = 1 << sw_switches;
        
        sw_sw = (sw_bank << 4) + sw_switches;
        
        // Is the change bit set for this and is the switch state the same as the last time, then fire a debounce event
        // and unset the change bit
        if ((sw_bit & direct_switches_changing[sw_bank]) && (sw_bit & sw_data) == (sw_bit & direct_switches[sw_bank]))
        {
          direct_switches_changing[sw_bank] -= sw_bit;
          broadcast_event(SWITCH_DIRECT, sw_sw, (sw_bit & sw_data) != 0, 1);
        }

        if ((sw_bit & sw_data) != (sw_bit & direct_switches[sw_bank]))
        {          
          
          // Is the change bit set? The state has also changed
          if ((sw_bit & direct_switches_changing[sw_bank]))
          {
            // Unset the change bit
            direct_switches_changing[sw_bank] -= sw_bit;
          }
          else
          {
            // Set the change bit
            direct_switches_changing[sw_bank] += sw_bit;
          }
          
          if ((sw_bit & sw_data) == 0) direct_switches[sw_bank] -= sw_bit;
          else direct_switches[sw_bank] += sw_bit;

          broadcast_event(SWITCH_DIRECT, sw_sw, (sw_bit & sw_data) != 0, 0);
        }
      }
    //}
    sw_bank_cnt = ++sw_bank_cnt % 2;
  }
  sw_col = ++sw_col % 8;
}

byte bank2idx(byte oldidx)
{
  return (oldidx - ((oldidx / 16) * 16)) + (8 * ((oldidx / 16) - 9));
}

void solenoids_handler()
{
  struct action *action_ptr;
  boolean print_sol_data = false;
  unsigned long t = millis();
  for (sol_bank = REGISTER_SOLENOID_C ; sol_bank <= REGISTER_SOLENOID_LOGIC; sol_bank++) {
    sol_sol_data = 0;
    print_sol_data = false;
    for (sol_sol = 0; sol_sol < 8; sol_sol++) {
      action_ptr = (struct action*) &solenoid_action[bank2idx(sol_bank << 4 | sol_sol)];
      //rt_printk("bank: %d sol: %d\n", bank, sol);
      if (action_ptr->mode == ACTION_SCHEDULE && action_ptr->trash == TRUE)
      {
        //Serial.println(F("Trashing schedule."));
        action_ptr->trash = FALSE;
        action_ptr->mode = ACTION_OFF;
        continue;
      }

      if (action_ptr->mode == ACTION_SCHEDULE)
      {
        if (CHECK_BIT(action_ptr->schedule, action_ptr->current_bit))
        {
          sol_sol_data |= (1 << sol_sol);
        }

        if (t - action_ptr->stop_time >= UPDATE_SCHEDULE_US)
        {
          if (action_ptr->current_bit == 31)
          {
            if (action_ptr->pulse_max > 0)
            {
              if (action_ptr->pulse_cnt == action_ptr->pulse_max)
              {
                action_ptr->trash = TRUE;
              }
              else
                action_ptr->pulse_cnt++;
            }
            action_ptr->current_bit = 0;
          }
          else
            action_ptr->current_bit++;

          action_ptr->stop_time = t;
        }

        continue;
      }

      if (action_ptr->mode == ACTION_OFF) continue;

      if ((action_ptr->mode != ACTION_PULSE || action_ptr->pulse_high) &&
        (sol_strength_cnt % action_ptr->strength) == 0) {
        sol_sol_data |= (1 << sol_sol);
        if (action_ptr->mode == ACTION_CONTINOUS) continue;
      }

      if (t > action_ptr->stop_time) {

        if (action_ptr->mode == ACTION_PULSE) {
          if (action_ptr->pulse_high == 0) { 
            action_ptr->pulse_cnt++; 
          }
          action_ptr->pulse_high = !action_ptr->pulse_high;
          action_ptr->stop_time = t +
            (action_ptr->pulse_high ? action_ptr->pulse_on_time : action_ptr->pulse_off_time);
        }
        if (action_ptr->mode != ACTION_PULSE || (action_ptr->mode == ACTION_PULSE &&
          action_ptr->pulse_cnt >= action_ptr->pulse_max && action_ptr->pulse_max > 0)) {

          if (action_ptr->mode != ACTION_SCHEDULE) action_ptr->mode = ACTION_OFF;
        }
      }


    }
    /*
    if (print_sol_data || sol_bank == 13)
     {
     Serial.print("Bank: ");
     Serial.print(sol_bank);
     Serial.print(" Data: ");
     Serial.println(sol_sol_data);
     }
     */
    if (bank_states[sol_bank] != sol_sol_data)
    {
      bank_states[sol_bank] = sol_sol_data;
      p->send_data(sol_bank,sol_sol_data);
    }
  }
}

void add_switch_rule(int sw, int sw_type, int state, int driver_type, int driver_num, byte driver_pulse_type, byte strength, byte duration)
{
  int i;

#ifdef DEBUG
  Serial.print("add_switch_rule(");
  Serial.print(sw);
  Serial.print(",");
  Serial.print(sw_type);
  Serial.print(",");
  Serial.print(state);
  Serial.print(",");
  Serial.print(driver_type);
  Serial.print(",");
  Serial.print(driver_num);
  Serial.print(",");
  Serial.print(driver_pulse_type);
  Serial.print(",");
  Serial.print(strength);
  Serial.print(",");
  Serial.println(duration);
#endif

  if (sw >= 0xff || driver_num >= 0xff) return;
  if (state != SW_OPEN && state != SW_CLOSE) return;
  if (driver_type != DRIVER_LAMP && driver_type != DRIVER_SOLENOID) return;
  if (sw_type != SWITCH_MATRIX && sw_type != SWITCH_DIRECT) return;

  // Find a blank switch rule
  for (i = 0; i < 0xff; i++)
  {
    if (switch_rules[i].switch_num == 0)
    {
      switch_rules[i].switch_num = sw;
      switch_rules[i].switch_type = sw_type;
      switch_rules[i].state = state;
      switch_rules[i].driver_type = driver_type;
      switch_rules[i].driver_num = driver_num;
      switch_rules[i].strength = strength;
      switch_rules[i].duration = duration;
      switch_rules[i].driver_pulse_type = driver_pulse_type;


#ifdef DEBUG
      Serial.print("Added sw rule sw=");
      Serial.print(sw);
      Serial.print(" type=");
      Serial.print(sw_type);
      Serial.print(" driver_type=");
      Serial.print(driver_type);
      Serial.print(" driver=");
      Serial.print(driver_num);
      Serial.print(" pulse_type=");
      Serial.print(driver_pulse_type);
      Serial.print(" RULE=");
      Serial.println(i);
#endif

      return;
    }
  }
}

void rem_switch_rule(int sw, int sw_type, int state, int driver_type, int driver_num)
{
  int i;
  for (i = 0; i < NUM_SWITCHRULES; i++)
  {
    if (switch_rules[i].switch_num == sw && switch_rules[i].state == state && switch_rules[i].switch_type == sw_type
      && switch_rules[i].driver_type == driver_type && switch_rules[i].driver_num == driver_num)
    {
      switch_rules[i].switch_num = 0;
      switch_rules[i].switch_type = 0;
      switch_rules[i].state = 0;
      switch_rules[i].driver_type = 0;
      switch_rules[i].driver_num = 0;
      switch_rules[i].driver_pulse_type = 0;
    }
  }
}

void schedule_driver(byte driver_type, byte driver_num, uint32_t schedule, byte loops)
{
  struct action *action_ptr;
  if (driver_type == DRIVER_SOLENOID) action_ptr = (struct action *) &solenoid_action[bank2idx(driver_num)];
  else if (driver_type == DRIVER_LAMP)  action_ptr = (struct action *) &lamp_matrix_action[lamp2driveridx(driver_num)];

  action_ptr->mode = ACTION_SCHEDULE;
  action_ptr->trash = FALSE;
  action_ptr->pulse_cnt = 1;
  action_ptr->pulse_max = loops;
  action_ptr->current_bit = 0;
  action_ptr->schedule = schedule;
  action_ptr->stop_time = 0;
  action_ptr->strength = 1;
}

void enable_flippers()
{
  add_switch_rule(SWITCH_JET_1, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_1, RULE_DRIVER_PULSE, 1, 32);
  add_switch_rule(SWITCH_JET_2, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_2, RULE_DRIVER_PULSE, 1, 32);
  add_switch_rule(SWITCH_JET_3, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_3, RULE_DRIVER_PULSE, 1, 32);

  add_switch_rule(SWITCH_LEFT_SLING, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_LEFT_SLING, RULE_DRIVER_PULSE, 1, 32);
  add_switch_rule(SWITCH_RIGHT_SLING, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_RIGHT_SLING, RULE_DRIVER_PULSE, 1, 32);

  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);
  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);

  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);
  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);

  // Upper flippers
  /*
  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);
  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_POWER, RULE_DRIVER_PULSE, 1, 50);
  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_ENABLE, 1, 32);

  add_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_UPPER_RIGHT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);
  add_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_UPPER_LEFT_FLIPPER_HOLD, RULE_DRIVER_DISABLE, 1, 32);
  */
}

void disable_flippers()
{
  rem_switch_rule(SWITCH_JET_1, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_1);
  rem_switch_rule(SWITCH_JET_2, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_2);
  rem_switch_rule(SWITCH_JET_3, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_JET_3);

  rem_switch_rule(SWITCH_LEFT_SLING, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_LEFT_SLING);
  rem_switch_rule(SWITCH_RIGHT_SLING, SWITCH_MATRIX, SW_CLOSE, DRIVER_SOLENOID, COIL_RIGHT_SLING);

  rem_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_POWER);
  rem_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_HOLD);
  rem_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_POWER);
  rem_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_CLOSE, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_HOLD);

  rem_switch_rule(SWITCH_RIGHT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_LOWER_RIGHT_FLIPPER_HOLD);
  rem_switch_rule(SWITCH_LEFT_FLIPPER_BUTTON, SWITCH_DIRECT, SW_OPEN, DRIVER_SOLENOID, COIL_LOWER_LEFT_FLIPPER_HOLD);
  
  disable_solenoid(COIL_JET_1);
  disable_solenoid(COIL_JET_2);
  disable_solenoid(COIL_JET_3);
  
  disable_solenoid(COIL_LOWER_LEFT_FLIPPER_POWER);
  disable_solenoid(COIL_LOWER_LEFT_FLIPPER_HOLD);
  disable_solenoid(COIL_LOWER_RIGHT_FLIPPER_POWER);
  disable_solenoid(COIL_LOWER_RIGHT_FLIPPER_HOLD);
}



