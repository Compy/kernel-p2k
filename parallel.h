#ifndef PARALLEL_H
#define PARALLEL_H

#define CTRLPORT 1
#define DATAPORT 2

#define US_LATCH_TIME 1

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define CHECK_BIT(a,b) ((a) & (1UL<<(b)))
#define SET_BIT(a,b) ((a) |= (1UL<<(b)))

class Parallel
{
  public:
  Parallel();
  unsigned char receive_data(unsigned char dest);
  void send_data(unsigned char dest, unsigned char data);
  void init_parport(void);
  void outb(unsigned char data, unsigned char reg);
  void print_binary(unsigned char bin);
  void print_binary32(uint32_t bin);
  private:
  
  
  void set_data_mode(int mode);
  unsigned char inb(unsigned char reg);
  
  char control_reg;
  char data_reg;
  int data_port_output;
};

#endif

