#include <stdio.h>
#include <stdbool.h>
#include "stdint.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "tm4c123gh6pm.h"
#include "driverlib/timer.h"

int state = 0;
int can_ped_press = 1;

void update_state(void);
void gpio_handler(void);

int ticks_from_ms(int ms){
  //Change milliseconds to ticks
  return 16000000*(ms/1000) - 1; 
}

void init_portB(void){
  // Enable Clock
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
  
  // Outputs
  GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7);
}

void init_portA(void){
  // Enable Clock
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
 
  // Outputs
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
}

void init_portF(void){
  // Enable Clock
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
  
  // Unlock and CR
  HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0|GPIO_PIN_4;
  
  // Inputs
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  // Enable interrupt
  GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_0|GPIO_INT_PIN_4);
  GPIOIntRegister(GPIO_PORTF_BASE, gpio_handler);
}

void timer0_init(){
  /*Initialize Timer 0*/
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
  TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
}

void timer1_init(){
  /*Initialize Timer 1*/
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));
  TimerConfigure(TIMER1_BASE,TIMER_CFG_ONE_SHOT);
  TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
}

void timer0_set(int ms, void (*callback)(void)){
  /*Set Timer 0 for ms to call callback using an interrupt*/
  TimerDisable(TIMER0_BASE,TIMER_BOTH);
  TimerLoadSet(TIMER0_BASE,TIMER_BOTH,ticks_from_ms(ms));
  TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
  TimerIntRegister(TIMER0_BASE, TIMER_BOTH ,callback);
  TimerEnable(TIMER0_BASE,TIMER_BOTH);
}

void timer1_set(int ms, void (*callback)(void)){
  /*Set Timer 1 for ms to call callback using an interrupt*/
  TimerDisable(TIMER1_BASE,TIMER_BOTH);
  TimerLoadSet(TIMER1_BASE,TIMER_BOTH,ticks_from_ms(ms));
  TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
  TimerIntRegister(TIMER1_BASE, TIMER_BOTH ,callback);
  TimerEnable(TIMER1_BASE,TIMER_BOTH);
}


void update_state(){
  // Zero out all pins
  GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7,0);
  GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7,0);
  if(state>5){  // State when a pedestrain requested to pass
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Traffic Light 1
    GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);  // Red Traffic Light 2
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);  // Green Pedestrain Light 1
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);  // Green Pedestrain Light 2
  }else{
    switch(state){
    case 0:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_7,GPIO_PIN_7);  // Green Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);  // Red Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);  // Green Pedestrain Light 2
      break;
    case 1:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6|GPIO_PIN_7,GPIO_PIN_6|GPIO_PIN_7);  // Yellow Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);  // Red Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_PIN_3);  // Green Pedestrain Light 2
      break;
    case 2:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);  // Red Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_PIN_2);  // Red Pedestrain Light 2
      break;
    case 3:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_PIN_1);  // Green Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);  // Green Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_PIN_2);  // Red Pedestrain Light 2
      break;
    case 4:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_0|GPIO_PIN_1);  // Yellow Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);  // Green Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_PIN_2);  // Red Pedestrain Light 2
      break;
    case 5:
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Traffic Light 1
      GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);  // Red Traffic Light 2
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);  // Red Pedestrain Light 1
      GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_PIN_2);  // Red Pedestrain Light 2
      break;
    }
  }
}

void one_sec_delay(){
  // clear the flag, accept pedestrian push button request
  can_ped_press = 1;
  TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
}

void timer1_callback(){
  // update the state when interrupt happened then clear the flag and enable timer0 again
  state -= 10;
  update_state();
  TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE,TIMER_BOTH);
  timer1_set(1000,one_sec_delay); // respond to the push button request then wait for 1s before respond to another request
}

void timer0_callback(){
  //update the state when interrupt happened and clear the flag
  state = (state+1)%6;
  TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
  update_state();
  if(state==0 || state==3){
    timer0_set(5000, timer0_callback);
  }else if(state==1 || state==4){
    timer0_set(2000, timer0_callback);
  }else if(state==2 || state==5){
    timer0_set(1000, timer0_callback);
  }
}

void gpio1_handler(){
  //pedestrian push button handler 1
  if(state<3){
    can_ped_press = 0;
    state += 10;
    update_state();
    TimerDisable(TIMER0_BASE,TIMER_BOTH);
    timer1_set(2000,timer1_callback);
  }
}
void gpio2_handler(){
  //pedestrian push button handler 2
  if(state>2 && state<6){
    can_ped_press = 0;
    state += 10;
    update_state();
    TimerDisable(TIMER0_BASE,TIMER_BOTH);
    timer1_set(2000,timer1_callback);
  }
}

void gpio_handler(){
  // check which button pressed then call its function and clear the interrupt
  if(can_ped_press){
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0){
      gpio1_handler();
    }else{
      gpio2_handler();
    }
  }
  GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
}

int main()
{
  init_portA(); // pedestrian Traffic Light
  init_portB(); // cars Traffic Light
  init_portF(); // pedestrian push buttons
  timer0_init(); // cars Traffic Light timer
  timer1_init(); // pedestrian Traffic Light timer
  
  update_state(); //write on the pins for the first time according to the state which is zero
  timer0_set(5000,timer0_callback); //start timer0 for the first time
  while(1){
    __asm("    wfi\n");
  } 
  return 0;
}
