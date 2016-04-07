#include "parallel.h"
//#define DEBUG

/**
 * PIN MAPPING FOR PARALLEL PORT
 * CTRL0          37    PP:  1
 * CTRL1          36    PP:  14
 * CTRL2          35    PP:  16
 * CTRL3          34    PP:  17
 ********
 *
 * DATA0          22    PP:  2
 * DATA1          23    PP:  3
 * DATA2          24    PP:  4
 * DATA3          25    PP:  5
 * DATA4          26    PP:  6
 * DATA5          27    PP:  7
 * DATA6          28    PP:  8
 * DATA7          29    PP:  9
 */
Parallel::Parallel()
{
  
  control_reg = 0;
  data_reg = 0;
  data_port_output = 1;
}

void Parallel::init_parport(void)
{
  
  DDRC = B11111111;
  set_data_mode(OUTPUT);
  
  outb(0x2A, CTRLPORT);
  
}

unsigned char Parallel::receive_data(unsigned char dest)
{
  
  unsigned char data = 0;
  
  set_data_mode(OUTPUT);
  outb(0x0a, CTRLPORT);
  outb(dest, DATAPORT);
  
  outb(0x0e, CTRLPORT);
  outb(0x2b, CTRLPORT);
  set_data_mode(INPUT);
  data = inb(DATAPORT);
  set_data_mode(OUTPUT);
  outb(0x2a, CTRLPORT);
  
  
  return data;
}

void Parallel::send_data(unsigned char dest, unsigned char data)
{
  
  set_data_mode(OUTPUT);
  outb(0x02, CTRLPORT);
  outb(dest, DATAPORT);
  outb(0x06, CTRLPORT);
  outb(data, DATAPORT);
  outb(0x03, CTRLPORT);
  outb(0x02, CTRLPORT);
  outb(0x2a, CTRLPORT);
  
}


void Parallel::outb(unsigned char data, unsigned char reg)
{
  
  unsigned char hw = B00000000;
  if (reg == CTRLPORT)
  {
    control_reg = data;
    // Invert pins 1,2, and 4 per the parallel port spec
    control_reg ^= 1 << 0;
    control_reg ^= 1 << 1;
    control_reg ^= 1 << 3;
    PORTC = control_reg;
  }
  else if (reg == DATAPORT)
  {
    data_reg = data;
    PORTA = data_reg;
  }
  
  delayMicroseconds(US_LATCH_TIME);
  
}

unsigned char Parallel::inb(unsigned char reg)
{
  
  if (reg == DATAPORT)
  {
    data_reg = PINA;
  }
  
  return data_reg;
  
}

void Parallel::print_binary(unsigned char bin)
{
  int i = 0;
  for (i = 8; i > 0; i--)
  {
    Serial.print((CHECK_BIT(bin,i-1)) ? "1" : "0");
  }
  Serial.println("");
}

void Parallel::print_binary32(uint32_t bin)
{
  int i = 0;
  for (i = 32; i > 0; i--)
  {
    Serial.print((CHECK_BIT(bin,i-1)) ? "1" : "0");
  }
  Serial.println("");
}

void Parallel::set_data_mode(int mode)
{
  //data_port_output = mode;
  
  if (mode == OUTPUT)
  {
    DDRA = B11111111;
  }
  else
  {
    DDRA = B00000000;
  }
  //delayMicroseconds(US_LATCH_TIME);
  
  asm volatile(
   // DDRC = 0010 0000, ie. PORTB5 is OUT
    "nop\n"
    "nop\n"
    "nop\n"
    "nop\n"
    "nop\n"
    "nop\n"
    "nop\n"
    "nop\n"
  );
  
  
}

