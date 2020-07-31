/******************************************************************************/
/* BLINKY.C: LED Flasher                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
                  
#include <stdio.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
#include "LCD.h"                        /* Graphic LCD function prototypes    */


/* Function that initializes LEDs                                             */
void LED_Init(void) {
  PINSEL10 = 0;                         /* Disable ETM interface, enable LEDs */
  FIO2DIR  = 0x000000FF;                /* P2.0..7 defined as Outputs         */
  FIO2MASK = 0x00000000;
}

/* Function that turns on requested LED                                       */
void LED_On (unsigned int num) {
  FIO2SET = (1 << num);
}

/* Function that turns off requested LED                                      */
void LED_Off (unsigned int num) {
  FIO2CLR = (1 << num);
}

/* Function that outputs value to LEDs                                        */
void LED_Out(unsigned int value) {
  FIO2CLR = 0xFF;                       /* Turn off all LEDs                  */
  FIO2SET = (value & 0xFF);             /* Turn on requested LEDs             */
}


/* Import external IRQ handlers from IRQ.c file                               */
extern __irq void T0_IRQHandler  (void);
extern __irq void T1_IRQHandler  (void);
extern __irq void T2_IRQHandler  (void);
extern __irq void EINT0_IRQHandler  (void);

/* Import external variables from IRQ.c file                                  */
extern double time_av;
extern double obj_cnt;



int main (void) {
  int i;

	
	//WE HAVE TO SET DAC FOR PUTTING OUT ALARMS
	//PINSEL1 = 0x10 << 20;
	PINSEL1 = 0x200000;
	
	
	PCONP        |= (1 << 22);
	T2MR0         = 999;                       /* 1msec = 12000-1 at 12.0 MHz , made it x1000 for seconds*/
  T2MCR         = 3;                           /* Interrupt and Reset on MR0  */
  //T2TCR         = 1;                           /* Timer0 Enable               */
  VICVectAddr26  = (unsigned long)T2_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl26  = 15;                          /* use it for Timer0 Interrupt */
	
	
	
  LED_Init();                           /* LED Initialization                 */

  /* Enable and setup timer interrupt, start timer                            */
  T0MR0         = 11999999;                       /* 1msec = 12000-1 at 12.0 MHz , made it x1000 for seconds*/
  T0MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T0TCR         = 1;                           /* Timer0 Enable               */
  VICVectAddr4  = (unsigned long)T0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl4  = 15;                          /* use it for Timer0 Interrupt */


	//PINSEL3 = 0x30;
	//T1CCR = 0x06;   // For capture on cap1.0, falling edge interrupt*/
	
	PINSEL3 = 0xC0;
	T1CCR = 0x30;   // For capture on cap1.1, falling edge interrupt
	
	T1CTCR = 0x00;
  T1TCR         = 1;                           /* Timer1 Enable               */
	
  VICVectAddr5  = (unsigned long)T1_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl5  = 15;                          /* use it for Timer1 Interrupt */
  VICIntEnable  = (1  << 5);                   /* Enable Timer1 Interrupt     */
  VICIntEnable  = (1  << 4);                   /* Enable Timer0 Interrupt     */
	VICIntEnable  = (1  << 26);

	EXTMODE = 0x01;
	EXTPOLAR = 0x00;
	EXTINT = 0xFF;
	
	PINSEL4 = (0x01 << 20);
  VICVectAddr14  = (unsigned long)EINT0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl14  = 15;                          /* use it for EINT0 Interrupt */
  VICIntEnable  = (1  << 14);                   /* Enable EINT0 Interrupt     */


  lcd_init();
  lcd_clear();
	
  set_cursor (0, 0);
	lcd_print("Object Counter");
  for (i = 0; i < 1000000; i++);       /* Wait for initial display           */


	lcd_clear();
	
	set_cursor (0, 0);
	lcd_print("obj_av:     0.0");
	set_cursor (0, 1);
	lcd_print("time_av:    0.0");
	i = 0;
  while (1) {                           /* Loop forever                       */	
		
		
  }
}
