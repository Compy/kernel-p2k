/**
 * This is the common Pinball 2000 header file.
 *
 * Incuded here are the constants for accessing data registers on the driver board. 
 * CTRL_ elements are control register numbers and IO_ are IO register numbers.
 *
 * (C) 2013 - Jimmy Lipham <jimmy@catster.net>
 * AKA "Compy"
 */

#ifndef P2K_H
#define P2K_H


//#define DEBUG 1

#define WATCHDOG_TIMEOUT           3000

#define NUM_SOLENOIDS              48
#define NUM_LAMPS                  128
#define NUM_SWITCHRULES            32

#define TICK_SWITCHES              500  // 400 us
#define TICK_LAMPS                 610
//#define TICK_LAMPS                 2430 // 2500 us (2.5ms)
#define TICK_SOLENOIDS             2500 // 2500 us (2.5ms)
#define UPDATE_SCHEDULE_US         31
#define TICK_LEDS                  250000
#define USB_SEND_INTERVAL_MS       8

// Generic Pinball 2000 definitions
#define REGISTER_SWITCH_COIN        0x00    // 00000000 [0x00]  U2-y0   Input   Coin switches
#define REGISTER_SWITCH_FLIPPER     0x01    // 00000001 [0x01]  U2-y1   Input   Flipper switches
#define REGISTER_SWITCH_DIP         0x02    // 00000010 [0x02]  U2-y2   Input   DIP Switches on driver board
#define REGISTER_SWITCH_MISC        0x03    // 00000011 [0x03]  U2-y3   Input   Flipper EOS and diagnostics
#define REGISTER_SWITCH_ROW         0x04    // 00000100 [0x04]  U2-y4   Input   Switch row receivers
#define REGISTER_SWITCH_COLUMN      0x05    // 00000101 [0x05]  U2-y5   Output  Switch column drivers
#define REGISTER_LAMP_ROW_A         0x06    // 00000110 [0x06]  U2-y6   Output  Lamp set A row drivers
#define REGISTER_LAMP_ROW_B         0x07    // 00000111 [0x07]  U2-y7   Output  Lamp set B row drivers
#define REGISTER_LAMP_COLUMN        0x08    // 00001000 [0x08]  U4-y0   Output  Lamp column drivers
#define REGISTER_SOLENOID_C         0x09    // 00001001 [0x09]  U4-y1   Output  Solenoid set C
#define REGISTER_SOLENOID_B         0x0a    // 00001010 [0x0A]  U4-y2   Output  Solenoid set B
#define REGISTER_SOLENOID_A         0x0b    // 00001011 [0x0B]  U4-y3   Output  Solenoid set A
#define REGISTER_SOLENOID_FLIPPER   0x0c    // 00001100 [0x0C]  U4-y4   Output  Flipper solenoids
#define REGISTER_SOLENOID_D         0x0d    // 00001101 [0x0D]  U4-y5   Output  Solenoid set D
#define REGISTER_SOLENOID_LOGIC     0x0e    // 00001110 [0x0E]  U4-y6   Output  Logic-level drivers
#define REGISTER_SWITCH_ZERO        0x0f    // 00001111 [0x0F]  U4-y7   Input   System info on driver board
#define REGISTER_LAMP_A_DIAGNOSTIC  0x10    // 00010000 [0x10]  U6-y0   Input   Lamp set A diagnostic inputs
#define REGISTER_LAMP_B_DIAGNOSTIC  0x11    // 00010001 [0x11]  U6-y1   Input   Lamp set B diagnostic inputs
#define REGISTER_FUSE_A_DIAGNOSTIC  0x12    // 00010010 [0x12]  U6-y2   Input   Fuse set A diagnostic inputs
#define REGISTER_FUST_B_DIAGNOSTIC  0x13    // 00010011 [0x13]  U6-y3   Input   Fuse set B diagnostic inputs

#define SW_COIN1            0x01
#define SW_COIN2            0x02
#define SW_COIN3            0x04
#define SW_COIN4            0x08
#define SW_COIN5            0x10
#define SW_COIN6            0x20
#define SW_COIN7            0x40
#define SW_COIN8            0x80

#define SW_FLIP1            0x01
#define SW_FLIP2            0x02
#define SW_FLIP3            0x04
#define SW_FLIP4            0x08
#define SW_FLIP5            0x10
#define SW_FLIP6            0x20
#define SW_FLIP7            0x40
#define SW_FLIP8            0x80

#define SW_DIP1             0x01
#define SW_DIP2             0x02
#define SW_DIP3             0x04
#define SW_DIP4             0x08

#define SW_DIA1             0x01
#define SW_DIA2             0x02
#define SW_DIA3             0x04
#define SW_DIA4             0x08
#define SW_EOS1             0x10
#define SW_EOS2             0x20
#define SW_EOS3             0x40
#define SW_EOS4             0x80

#define NUM_SW_ROWS         8
#define SW_ROW1             0x01
#define SW_ROW2             0x02
#define SW_ROW3             0x04
#define SW_ROW4             0x08
#define SW_ROW5             0x10
#define SW_ROW6             0x20
#define SW_ROW7             0x40
#define SW_ROW8             0x80

#define NUM_SW_COLS         8
#define SW_COL1             0x01
#define SW_COL2             0x02
#define SW_COL3             0x04
#define SW_COL4             0x08
#define SW_COL5             0x10
#define SW_COL6             0x20
#define SW_COL7             0x40
#define SW_COL8             0x80

#define SWITCH_SLAM_TILT                0x10
#define SWITCH_COIN_DOOR		        0x11
#define SWITCH_PLUMB_BOB_TILT           0x12
#define SWITCH_RIGHT_FLIPPER_BUTTON     0x14
#define SWITCH_LEFT_FLIPPER_BUTTON      0x15
#define SWITCH_RIGHT_ACTION_BUTTON      0x16
#define SWITCH_LEFT_ACTION_BUTTON       0x17

#define SWITCH_LOWER_RIGHT_FLIPPER_EOS  0x34
#define SWITCH_LOWER_LEFT_FLIPPER_EOS   0x35
#define SWITCH_UPPER_RIGHT_FLIPPER_EOS  0x36
#define SWITCH_UPPER_LEFT_FLIPPER_EOS   0x37

#define SWITCH_START_BUTTON             0x02
#define SWITCH_LAUNCH_BUTTON            0x12
#define COIL_BALL_EJECT                 0xa0
#define COIL_BALL_LAUNCH                0xa6

#define SWITCH_LEFT_SLING               0x50
#define SWITCH_RIGHT_SLING              0x51
#define SWITCH_JET_1                    0x52
#define SWITCH_JET_2                    0x53
#define SWITCH_JET_3                    0x54

#define COIL_LEFT_SLING                 0xa1
#define COIL_RIGHT_SLING                0xa2
#define COIL_JET_1                      0xa3
#define COIL_JET_2                      0xa4
#define COIL_JET_3                      0xa5

#define COIL_LOWER_RIGHT_FLIPPER_POWER  0xc0
#define COIL_LOWER_RIGHT_FLIPPER_HOLD   0xc1
#define COIL_LOWER_LEFT_FLIPPER_POWER   0xc2
#define COIL_LOWER_LEFT_FLIPPER_HOLD    0xc3

#define COIL_UPPER_RIGHT_FLIPPER_POWER  0xc4
#define COIL_UPPER_RIGHT_FLIPPER_HOLD   0xc5
#define COIL_UPPER_LEFT_FLIPPER_POWER   0xc6
#define COIL_UPPER_LEFT_FLIPPER_HOLD    0xc7



#define COIL_SOL_EJECT                  0xb7


#define SW_OPEN             0
#define SW_CLOSE            1
#define OFF                 0
#define ON                  1
#define TRUE                1
#define FALSE               0

#define ACTION_LAMP			0
#define ACTION_SOLENOID		1
#define ACTION_OFF			0
#define ACTION_CONTINOUS	1
#define ACTION_ONE_SHOT		2
#define ACTION_PULSE		3
#define ACTION_SCHEDULE		4

#define SWITCH_MATRIX		0
#define SWITCH_DIRECT		1
#define SWITCH_COIN			2
#define DRIVER_LAMP			0
#define DRIVER_SOLENOID		1

#define CMD_FIRE_LAMP			0
#define CMD_ENABLE_FLIPPERS		1
#define CMD_DISABLE_FLIPPERS	        2
#define CMD_FIRE_SOLENOID		3
#define CMD_PATTER_LAMP			4
#define CMD_PATTER_SOLENOID		5
#define CMD_SCHEDULE_LAMP		6
#define CMD_SCHEDULE_SOLENOID	        7
#define CMD_ENABLE_LAMP			8
#define CMD_DISABLE_LAMP		9
#define CMD_DISABLE_LAMPS		10
#define CMD_ENABLE_SOLENOID		11
#define CMD_DISABLE_SOLENOID	        12
#define CMD_ADD_SWITCH_RULE_CLOSE	13
#define CMD_DEL_SWITCH_RULE_CLOSE	14
#define CMD_GET_SWITCH_STATES	        15
#define CMD_ADD_SWITCH_RULE_OPEN	16
#define CMD_DEL_SWITCH_RULE_OPEN	17
#define CMD_HELLO                       18
#define CMD_RESET_COIL_SCHEDULES        19
#define CMD_SET_TICK_RATE               20
#define CMD_SET_LED_MODE                21
#define CMD_HEARTBEAT                   99

// OUTBOUND COMMANDS
#define CMD_OUTBOUND_MATRIX_SWITCH	0
#define CMD_OUTBOUND_DIRECT_SWITCH	1
#define CMD_OUTBOUND_COIN_SWITCH	2
#define CMD_OUTBOUND_SW_STATE           3


#define EVT_HELLO                      18
#define EVT_INVALID                    0
#define EVT_SWITCHCLOSEDDEBOUNCED      1
#define EVT_SWITCHOPENDEBOUNCED        2
#define EVT_SWITCHCLOSEDNONDEBOUNCED   3
#define EVT_SWITCHOPENNONDEBOUNCED     4

// Solenoids 25 - 32 (REGISTER_SOLENOID_D)
#define HEALTH_LED			0xd4
#define INTER_LOCK_RELAY		0xd5
#define COUNTER				0xd6
#define LAMP_TEST_CONTROL		0xd7

#define LAMP_START_BUTTON               0x02

#include "StreamSend.h"
#include "parallel.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define RULE_DRIVER_ENABLE 2
#define RULE_DRIVER_DISABLE 1
#define RULE_DRIVER_PULSE 0

//typedef void (*SWITCH_FUNCTION)(int sw, byte state);
//typedef void (*EVENT_FUNCTION)(int param);
typedef long long RTIME;

static volatile byte switch_matrix[0x08];
static volatile byte switch_matrix_changing[0x08];
static volatile byte direct_switches[0x14];
static volatile byte direct_switches_changing[0x14];

long blink_health_interval = 500;
long current_millis = 0;
long time_of_last_health_blink = 0;
unsigned long last_switch_tick = 0;
unsigned long last_solenoid_tick = 0;
unsigned long last_lamp_tick = 0;
unsigned long last_led_tick = 0;
unsigned long last_usb_send = 0;
unsigned long lamp_tick_time = 0;
unsigned long lamp_tick_interval = 0;
unsigned long time, uTime;

unsigned long stats_lamp_exec_length = 0,
              stats_switch_exec_length = 0,
              stats_solenoid_exec_length = 0,
              stats_led_exec_length = 0;

Parallel *p;

byte sw_data, sw_col, sw_row, sw_bit, sw_sw, sw_col_val, sw_bank, sw_switches, sw_bank_cnt;
byte sol_bank, sol_sol, sol_strength_cnt, sol_sol_data;

int lamp_col, lamp_row, lamp_lamp, lamp_strength_cnt, lamp_timer_cnt;
byte lamp_data[2];

byte bank_states[16];
byte matrix_a_states[8];
byte matrix_b_states[8];

boolean hello_processed;
/*
LPD8806 strip1 = LPD8806(10, 3, 2);
LPD8806 strip2 = LPD8806(20, 5, 4);
LPD8806 strip3 = LPD8806(18, 42, 44);
LPD8806 strip4 = LPD8806(10, 48, 46);
*/
struct action
{
  byte mode;
  RTIME stop_time;
  byte pulse_on_time;
  byte pulse_off_time;
  byte pulse_high;
  byte pulse_cnt;
  byte pulse_max;
  byte strength;
  uint32_t schedule;
  byte current_bit;
  boolean trash;
};

struct P2KCommand
{
  uint32_t opcode;
  uint32_t int1;
  uint32_t int2;
  uint32_t int3;
  uint32_t int4;
};

struct result
{
  int opcode;
  int int1;
  int int2;
};

struct switchrule
{
  byte switch_num;
  byte switch_type;
  byte state;
  byte driver_type;
  byte driver_num;
  byte driver_pulse_type;
  byte strength;
  byte duration;
};

static volatile struct action lamp_matrix_action[NUM_LAMPS];
static volatile struct action solenoid_action[NUM_SOLENOIDS];
static volatile struct switchrule switch_rules[NUM_SWITCHRULES];
P2KCommand outbound_command;

byte get_switch(byte sw);
byte get_direct_switch(byte sw);
byte get_lamp(byte lamp);
byte get_solenoid(byte solenoid);

void set_action(byte type, byte num, byte mode, byte strength, byte on_time, byte off_time, int pulse_max);
void enable_lamp(byte lamp, byte strength);
void fire_lamp(byte lamp, byte strength, byte duration);
void pulse_lamp(byte lamp, byte strength, byte on_time, byte off_time, int pulse_max);
void disable_lamp(byte lamp);
void disable_lamps(void);

void enable_solenoid(byte solenoid);
void fire_solenoid(byte solenoid, byte duration);
void pulse_solenoid(byte solenoid, byte on_time, byte off_time, int pulse_max);
void disable_solenoid(byte solenoid);

void broadcast_event(byte type, byte sw, byte state, byte debounced);
void schedule_driver(byte driver_type, byte driver_num, uint32_t schedule, byte loops);
void process_message(char *, int size);
byte get_lamp_idx(int matrix, int col, int row);

void send_msg(struct P2KCommand cmd);
int receive_msg(struct P2KCommand *cmd);
/**
 * Add a switch rule
 * sw - The switch number to trigger the rule
 * sw_type - Either a SWITCH_MATRIX or SWITCH_DIRECT switch
 * state - One of SW_OPEN or SW_CLOSE depending on when the event should be triggered
 * driver_type - Either a DRIVER_LAMP or DRIVER_SOLENOID depending on which type should be driven
 * strength - The strength of the coil pulse
 * duration - The duration of the pulse
 */
void add_switch_rule(byte sw, byte sw_type, byte state, byte driver_type, byte driver_num, byte driver_pulse_type, byte strength, byte duration);
void rem_switch_rule(byte sw, byte sw_type, byte state, byte driver_type, byte driver_num);

void enable_flippers();
void disable_flippers();

void schedule_driver(int driver_type, int driver_num, uint32_t schedule, int loops);

#endif


