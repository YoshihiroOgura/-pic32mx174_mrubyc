#include "mrubyc.h"
#include <stdio.h>
#include <stdlib.h>
#include "delay.h"
#include "string.h"
#include <sys/attribs.h>
#include <math.h>
#include "timer.h"
#include "mrbc_firm.h"

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config AI2C1 = OFF              // Alternate I/O Select for I2C1 (I2C1 uses the SDA1/SCL1 pins)
#pragma config AI2C2 = OFF              // Alternate I/O Select for I2C2 (I2C2 uses the SDA2/SCL2 pins)
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)
#pragma config BOREN = OFF              // Brown-Out Reset (BOR) Enable (Disable BOR)
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable (Disable ZPBOR during Deep Sleep Mode)
#pragma config DSWDTPS = DSPS32         // Deep Sleep Watchdog Timer Postscaler (1:2^36)
#pragma config DSWDTOSC = LPRC          // Deep Sleep WDT Reference Clock Selection (Select LPRC as DSWDT Reference clock)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable (Disable DSWDT during Deep Sleep Mode)
#pragma config FDSEN = OFF              // Deep Sleep Enable (Disable DSEN bit in DSCON)

// DEVCFG1
#pragma config FNOSC = SPLL             // Oscillator Selection Bits (System PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock Switch Enable, FSCM Enabled)
#pragma config WDTPS = PS32             // Watchdog Timer Postscaler (1:32)
#pragma config WDTSPGM = ON             // Watchdog Timer Stop During Flash Programming (Watchdog Timer stops during Flash programming)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx3        // ICE/ICD Comm Channel Select (Communicate on PGEC3/PGED3)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define MEMORY_SIZE (1024*20)
static uint8_t memory_pool[MEMORY_SIZE];
uint8_t t_count = 0;

int hal_write(int fd, const void *buf, int nbytes) {
  int i;
  while (U1STAbits.TRMT == 0);
  for (i = nbytes; i; --i) {
    while (U1STAbits.TRMT == 0);
    U1TXREG= *(char*) buf++;
  }
  return (nbytes);
}

int hal_flush(int fd) {
  return 0;
}

void pin_init(void){
  ANSELA = 0;
  ANSELB = 0;
  PORTA = 0;
  PORTB = 0;
  TRISAbits.TRISA4 = 1;
  TRISA &= 0xFC;
  TRISB &= 0xFC;
  TRISB |= 0x4c;
  TRISBbits.TRISB6 = 1;
  CNPDA = 0x0;
  CNPUA = 0x0;
  CNPDB = 0x0;
  CNPUB |= 0x4c;
}

static void c_pin_init(mrb_vm *vm, mrb_value *v, int argc) {
  pin_init();
}

static void c_c_chenge(mrb_vm *vm, mrb_value *v, int argc) {
  U2MODEbits.ON = 0;
  I2C2CONbits.ON = 0;
  AD1CON1bits.ON = 0;
  T2CONbits.ON = 0;
  T3CONbits.ON = 0;
  __builtin_disable_interrupts();
  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  
  OSCCONbits.CLKLOCK = 0;
  OSCCONbits.NOSC = 0b101;
  OSCCONbits.OSWEN = 1;
  while(OSCCONbits.OSWEN);
  OSCCONbits.SLPEN = 1;
  SYSKEY = 0x0;
  TRISA=0;
  TRISB=0;
  PMD1=0xFFFF;
  PMD2=0xFFFF;
  PMD3=0xFFFF;
  PMD4=0xFFFE;
  PMD5=0xFFFF;
  PMD6=0xFFFF;
  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  __builtin_enable_interrupts();
  SYSKEY = 0x0;
}

int check_timeout(void)
{
  int i;
  for( i = 0; i < 50; i++ ) {
    LATAbits.LATA0 = 1;
    __delay_ms( 30 );
    LATAbits.LATA0 = 0;
    __delay_ms( 30 );
    if(U1STAbits.URXDA){
      U1RXREG = 0;
      return 1;
    }
  }
  return 0;
}

/* mruby/c writer */

void __ISR(_TIMER_1_VECTOR, IPL1AUTO) _T1Interrupt (  ){
  if(OSCCONbits.NOSC == 0b101){
    SYSKEY = 0x0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;

    OSCCONbits.CLKLOCK = 0;
    OSCCONbits.NOSC = 0b001;
    OSCCONbits.OSWEN = 1;
    while(OSCCONbits.OSWEN);
    OSCCONbits.SLPEN = 1;
    SYSKEY = 0x0;
    U2MODEbits.ON = 1;
    I2C2CONbits.ON = 1;
    AD1CON1bits.ON = 1;
    T2CONbits.ON = 1;
    T3CONbits.ON = 1;
    pin_init();
    PMD1=0;
    PMD2=0;
    PMD3=0;
    PMD4=0;
    PMD5=0;
    PMD6=0;
  }
  mrbc_tick();
  IFS0CLR= 1 << _IFS0_T1IF_POSITION;
}

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) _T2Interrupt (  ){
  t_count++;
  IFS0CLR = 1 << _IFS0_T2IF_POSITION;
}

static void c_sleep_mode(mrb_vm *vm, mrb_value *v, int argc) {
  uint8_t sl_st = GET_INT_ARG(1) & 0x01;
  if(sl_st == 1){
    OSCCONbits.CLKLOCK = 0;
    OSCCONbits.NOSC = 0b101;
  }else{
    OSCCONbits.NOSC = 0b001;
    OSCCONbits.CLKLOCK = 1;
  }
  OSCCONbits.SLPEN = sl_st;
}

int main(void){
  PB1DIVbits.PBDIV = 1;
  
  /* module init */
  pin_init();
  i2c_init();
  adc_init();
  uart_init();
  timer_init();

  /*Enable the interrupt*/
  IPC9bits.U2IP = 4;
  IPC9bits.U2IS = 3;
  IFS1bits.U2RXIF = 0;
  IFS1bits.U2TXIF = 0;
  IEC1bits.U2RXIE = 1;
  IEC1bits.U2TXIE = 0;
  IEC0bits.T1IE = 1;
  IEC0bits.T2IE = 1;
  IPC6bits.FCEIP = 1;
  IPC6bits.FCEIS = 0;
  IPC1bits.T1IP = 1;
  IPC1bits.T1IS = 0;
  IPC2bits.T2IP = 2;
  IPC2bits.T2IS = 0;
  INTCONbits.MVEC = 1;

  if (check_timeout()){
    /* IDE code */
    add_code();
  };

  /* mruby/c */
  mrbc_init(memory_pool, MEMORY_SIZE);
  mrbc_define_method(0, mrbc_class_object, "pinInit", c_pin_init);
  mrbc_define_method(0, mrbc_class_object, "sleep_mode", c_c_chenge);
  mrbc_init_class_adc(0);
  mrbc_init_class_i2c(0);
  mrbc_init_class_uart(0);
  mrbc_init_class_digital(0);
  mrbc_init_class_timer(0);
  mrbc_init_class_pwm(0);
  mrbc_init_class_onboard(0);
  int fl_addr = FLASH_SAVE_ADDR;
  uint8_t code_size_box[4];
  while(1){
    if(*((char *)fl_addr) != 'R'){
      break;
    }
    mrbc_create_task((void *)fl_addr, 0);
    memcpy(code_size_box, (void *)(fl_addr + 10), 4);
    int i = 0;
    int size = 0;
    for(i = 0;i < 4;i++){
      size += (code_size_box[i] << ((3-i)*8));
    }
    int rowCount = (size % ROW_SIZE == 0) ? size / ROW_SIZE : size / ROW_SIZE + 1;
    fl_addr = fl_addr + ROW_SIZE*rowCount;
  }
  T1CONbits.ON = 1;
  mrbc_run();
  return 1;
}
