/******************************************************************************/
/* IRQ.C: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <LPC23xx.H>                    /* LPC23xx definitions                */
#include <stdio.h>     
#include "LCD.h"

unsigned long int last_cap = 0;
unsigned long int last_obj_cnt = 0;
unsigned long int obj_cnt = 0;
double time_av = 0;
double obj_av = 0;

short sample = 0;
short freq = 0;
short val = 0;
/* Import function for turning LEDs on or off                                 */
extern void LED_Out (unsigned int num);
extern void LED_Off(unsigned int num);

long int sounds[40] = {0x1FF
,0x29D
,0x32C
,0x39D
,0x3E5
,0x3FF
,0x3E5
,0x39D
,0x32C
,0x29D
,0x1FF
,0x161
,0xD2
,0x61
,0x19
,0x0
,0x19
,0x61
,0xD2
,0x161
,0x1FF
,0x32C
,0x3E5
,0x3E5
,0x32C
,0x1FF
,0xD2
,0x19
,0x19
,0xD2
,0x1FF
,0x32C
,0x3E5
,0x3E5
,0x32C
,0x1FF
,0xD2
,0x19
,0x19
,0xD2
};

//extern void lcd_putchar (char c);
//extern void lcd_print   (unsigned char const *string);
//extern void set_cursor  (unsigned char column, unsigned char line);

/* Timer0 IRQ: Executed periodically, THIS IS MATCH INT                                      */
__irq void T0_IRQHandler (void) {
	

	char myStg[10];
	set_cursor (9, 0);
	sprintf(myStg, "%7li", obj_cnt);
	lcd_print(myStg);
	
	
	set_cursor (9, 1);
	time_av = 1000/(float)obj_cnt;
	sprintf(myStg, "%7.3f", time_av);
	lcd_print(myStg);
	
	
	if(obj_cnt == 0)
		val++;
	else
		val = 0;
	
	
	
	if(obj_cnt >= 100){
		freq = 0;
		T2TCR = 0x01;
		LED_Out(obj_cnt);
	}
	else if(val > 4){
		freq = 20;
		T2TCR = 0x01;
	}
	else {
		VICIntEnable  = (1  << 26);
		T2TCR = 0x0;
	}
	
	obj_cnt = 0;
	
  T0IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
	
}

//THIS IS CAPTURE INT
__irq void T1_IRQHandler (void){
	obj_cnt++;
	T1TCR         = 2;                           /* Timer1 Reset              */
	//LED_Out(obj_cnt);
  T1IR        = 0x20;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}

__irq void EINT0_IRQHandler (void) {
	VICIntEnClr = (1 << 26);
	EXTINT = 0xFF;
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}

__irq void T2_IRQHandler (void) {
	DACR = sounds[freq + sample] << 6;
	sample++;
	if(sample == 20){
		sample =0;
		LED_Out(obj_cnt);
	}
	
  T2IR        = 0xFF;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}
