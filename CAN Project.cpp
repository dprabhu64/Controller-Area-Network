/*******************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software
* and to discontinue the availability of this software. By using this software,
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer *
* Copyright (C) 2012 Renesas Electronics Corporation. All rights reserved.    
*******************************************************************************/
/*******************************************************************************
* File Name	   : can_api_demo.c
* Version      : 1.0
* Device(s)	   : RX63N
* Tool-Chain   : RX Standard Toolchain 1.0.0
* H/W Platform : YRDKRX63N
* Description  : Demonstration of CAN receive and transmit using the CAN API.
*                CAN Baudrate: 500 kbps.
*                
* Operation	   : The demo can be run in three ways:
*                1) Program two boards and connect them together over the CAN 
*                   bus.
*                2) With CANPORT_TEST_1_INT_LOOPBACK used in the R_CAN_PortSet
*                   API you can communicate internally, no external bus needed!
*                3) Use a CAN bus monitor, e.g. SysTec's low-cost monitor 
*                   3204000, to send and receive frames to/from the demo. 
*                   Remote frames can also be demonstrated if CAN interrupts 
*                   are enabled. See last paragraph below.
*
*                OPERATION:
*                The demo transmits and receives frames with CAN-ID 1 by 
*                default. The default demo CAN-ID value is set in can_api_demo.h  
*                by g_tx_id_default.
*                
*                The software starts up by immediately sending ten test frames. 
*                This has two purposes, to check the link and to demonstrate 
*                how messages are sent back-to-back as quickly as possible.
*                
*                Press SW1 to send one CAN frame. The frame will be received 
*                and displayed by the other RSK as long as that board's receive
*                ID (RxID) matches the sending boards transmit ID (TxID).
*                
*                Press SW2 to display the current demo TxID on the LCD. To inc-
*                rement the TxID hold SW2 down and press SW3. The actual send 
*                command is invoked by the Sw1Func function.
*
*                Press SW3 to display current demo RxID on the LCD. To change 
*                RxID hold SW3 down and press SW2.
*
*                By default, polled CAN is used. To use the CAN interrupts for 
*                faster processing, uncomment USE_CAN_POLL in file 
*                config_r_can_rapi.h.
*
*                REMOTE frames:
*                Besides demonstrating Tx and Rx of Standard CAN frames, the 
*                demo will also send remote frame responses for remote frame 
*                requests received by the mailbox at CAN-ID 50h (defined by
*                REMOTE_TEST_ID in can_api_demo.h).
*                Remote frames demo is only done ininterrupt mode:   
*                "#define USE_CAN_POLL = 0" set in the CAN API config file. 
*                Remote requests are not sent by this demo as it is, and so must
*                come from an outside source, e.g. the CAN monitor mentioned 
*                above. The external CAN source must be set to send remote frame
*                requests to CAN-ID 50h.
*
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 05.03.2012   1.0    First release. Ported to YRDKRX63N from various
*                               prior sources.
*******************************************************************************/

/*******************************************************************************
Includes	<System Includes> , "Project Includes"
*******************************************************************************/
#include <machine.h>
#include <stdint.h>
#include <stdio.h>
#include "file1.h"

#include "platform.h"
#include "config_r_can_rapi.h"
#include "can_api_demo.h"
#include "switches.h"



#include <string.h>
#include <stdio.h>
#include <machine.h>
#include "file1.h"
#include "platform.h"
#include "r_riic_rx600.h"
#include "r_riic_rx600_master.h"
#include "riic_master_main.h"

#include "thermal_sensor_demo.h"
/* Defines ADT7420 parameters */
#include "ADT7420.h"




/*******************************************************************************
Macro definitions
*******************************************************************************/
#define LCD_DELAY               0x00400000
#define NR_STARTUP_TEST_FRAMES	10
#define MAX_CHANNELS 3  /* RX63x */

/* Pick only ONE demo testmode below by uncommenting the macro definition. */ 
#define DEMO_NORMAL               1
//#define DEMO_TEST_1_INT_LOOPBACK    1
//#define DEMO_TEST_0_EXT_LOOPBACK  1
//#define DEMO_TEST_LISTEN_ONLY     1
/******************************************************************************
Exported global variables (to be accessed by other files)
******************************************************************************/
/* Demo data */

#define engine PORTA.PIDR.BIT.B6
#define fuel PORTA.PIDR.BIT.B7
#define traction PORTC.PIDR.BIT.B1


can_frame_t		g_tx_dataframe;
can_frame_t		g_rx_dataframe;
can_frame_t		g_remote_frame;

/* the CAN peripheral channel used in this demo */
uint32_t g_can_channel;

#if TEST_FIFO
can_frame_t	    tx_fifo_dataframe;
uint8_t         tx_fifo_flag = 0;
#endif


/* Demo flags. */
#if (USE_CAN_POLL == 0)
uint32_t			CAN0_tx_sentdata_flag = 0;
uint32_t			CAN0_tx_remote_sentdata_flag = 0;
uint32_t			CAN0_rx_newdata_flag = 0;
uint32_t			CAN0_rx_test_newdata_flag = 0;
uint32_t			CAN0_rx_g_remote_frame_flag = 0;
#endif 

enum app_err_enum	app_err_nr;

/* Functions */
void lcd_flash(void);
void RTC_display(void);
void accel(char);

/******************************************************************************
Private global variables and functions
******************************************************************************/
int temperature;
/* Errors. Peripheral and bus errors. Space for each channel. */
static uint32_t		error_bus_status[MAX_CHANNELS];
static uint32_t     error_bus_status_prev[MAX_CHANNELS];
static uint32_t		can_state[MAX_CHANNELS];
static uint32_t		nr_times_reached_busoff[MAX_CHANNELS];

typedef struct
{
    uint8_t     second;                 /* Second */
    uint8_t     minute;                 /* Minute */
    uint8_t     hour;                   /* Hour */
    uint8_t     dayweek;                /* Day of the week */
    uint8_t     day;                    /* Day */
    uint8_t     month;                  /* Month */
    uint16_t    year;                   /* Year */
} time_bcd_t;
time_bcd_t      time; 

/* TEST CAN ID */
uint32_t g_tx_id_default;
uint32_t g_rx_id_default;

char lcd_out[13],date_d[13],time_d[13],cmp[10];

uint16_t adc_result;

/* Functions */
static uint32_t init_can_app(void);
static void check_can_errors(void);
static void handle_can_bus_state(uint8_t ch_nr);

#if (USE_CAN_POLL == 1)
static void can_poll_demo(void);
#else 
static void can_int_demo(void);
#endif 


/*****************************************************************************
* Function name:    can_api_demo
* Description  : 	Main Can API demo program
* Arguments    :    none
* Return value : 	none
*****************************************************************************/
void can_api_demo(void)
{
//	printf("can api");
    uint32_t i;
    uint32_t api_status = R_CAN_OK;
    uint8_t     disp_buf[13] = {0}; /* Temporary storage for display strings. */   

    g_can_channel = CH_0; /* using CAN channel 0 for this demo */
	int a,b,c,d,e,f;
	char lcd_out[13],eng,fl,trac,ts;

    /* Set default mailbox IDs for the demo*/
    if (FRAME_ID_MODE == STD_ID_MODE)
    {
        g_tx_id_default = 0x001;
        g_rx_id_default = 0x001;    
    }
    else
    {
        g_tx_id_default = 0x000A0001;
        g_rx_id_default = 0x000A0001;    
    }

    /* Init CAN. */
    api_status = R_CAN_Create(g_can_channel);
    
    if (api_status != R_CAN_OK)
    {   /* An error at this stage is fatal to demo, so stop here. */
        sprintf((char *) disp_buf, "API err:%02X  ", api_status);
        //lcd_display(LCD_LINE8, disp_buf); 
                       
        while(1)
        {
            nop();/* Wait here and leave error displayed. */
        } 
    } 
    
    /***************************************************************************
    * Pick ONE R_CAN_PortSet call below by uncommenting the matching macro  
    * in the Macro definitions section above.
    ***************************************************************************/  
    /* Normal CAN bus usage. */
    #if DEMO_NORMAL
    R_CAN_PortSet(g_can_channel, ENABLE);
    /* Test modes. With Internal loopback mode you only need one board! */
    #elif DEMO_TEST_1_INT_LOOPBACK
    R_CAN_PortSet(g_can_channel, CANPORT_TEST_1_INT_LOOPBACK);
    #elif DEMO_TEST_0_EXT_LOOPBACK
    R_CAN_PortSet(g_can_channel, CANPORT_TEST_0_EXT_LOOPBACK);
    #elif DEMO_TEST_LISTEN_ONLY
    R_CAN_PortSet(g_can_channel, CANPORT_TEST_LISTEN_ONLY);
    #endif

    /* Initialize CAN mailboxes. */
    api_status |= init_can_app();

    /* Is all OK after all CAN initialization? */
    if (api_status != R_CAN_OK)
    {
        api_status = R_CAN_OK;
        app_err_nr = APP_ERR_CAN_INIT;
    }

    /* Interrupt Enable flag is set by default. */

    /*****************************************************************************/
    /* This is how you send multiple messages back to back. Sending 10 messages. */
    /*****************************************************************************/
    if (FRAME_ID_MODE == STD_ID_MODE )
    {
        api_status |= R_CAN_TxSet(g_can_channel, CANBOX_TX, &g_tx_dataframe, DATA_FRAME);   
    }
    else
    {
        api_status |= R_CAN_TxSetXid(g_can_channel, CANBOX_TX, &g_tx_dataframe, DATA_FRAME);
    }

    #if (USE_CAN_POLL == 1)
    while (R_CAN_TxCheck(g_can_channel, CANBOX_TX))
    {
        /* Poll loop. A real application should provide for timeout. */
    }

    for (i = 0; i < NR_STARTUP_TEST_FRAMES; i++)
    {
        api_status |= R_CAN_Tx(g_can_channel, CANBOX_TX);

        while (R_CAN_TxCheck(g_can_channel, CANBOX_TX))
        {
            /* Poll loop. A real application should provide for timeout. */
        }
    }
    
    #else	/* Using CAN interrupts. */
    while (0 == CAN0_tx_sentdata_flag)
    {
        /* Poll loop. Flag set by hardware. A real application should provide for timeout. */
    }

    CAN0_tx_sentdata_flag = 0;

    for (i = 0; i < NR_STARTUP_TEST_FRAMES; i++)
    {
        api_status |= R_CAN_Tx(g_can_channel, CANBOX_TX);

        while (0 == CAN0_tx_sentdata_flag)
        {
            /* Poll loop. Flag set by hardware. A real application should provide for timeout. */
        }

        CAN0_tx_sentdata_flag = 0; /* Clear the flag for next loop iteration. */
    }
    #endif

    /*	M A I N	L O O P	* * * * * * * * * * * * * * * * * * * * * * * * * */	
	
	
	PORTA.PDR.BIT.B6 = 0;
	PORTA.PDR.BIT.B7= 0;
	PORTC.PDR.BIT.B1= 0;
	
	engine=1;
	fuel =1;
	traction=1;
	
   // while(1)
    //{
        /* User pressing switch(es) */
		/********************
        read_switches();*/
	//	printf("engine ");
		RTC_display();	
	//	printf("engine ");
		if (engine==1)
		{
//for(a=0;a<2000;a++) {}
		//	printf("Engine high\n");
			eng='R';
		}
		else{
		//	printf("Engine low\n");
		 	eng='G';	
		}
		if (fuel==1)
		{
//for(a=0;a<2000;a++) {}
		//	printf("Fuel high\n");
			fl='R';

		}
		else 
		{
		//	printf("Fuel low\n");
		 	fl='G';
		}
		if (traction==1) {
		//	printf("Tract high\n");	
			trac='R';

		}
		else {
		//	printf("Tract low\n ");
			sprintf(lcd_out,"Tract low");
    		lcd_display(LCD_LINE6,lcd_out);
	 		trac='G';
		}
		
 

		adc_result = S12ADC_read();
	    adc_result=adc_result/455;
		g_tx_dataframe.data[0] = adc_result;
		printf("\n adc=%X",g_tx_dataframe.data[0]);
		g_tx_dataframe.data[1] = eng;
		g_tx_dataframe.data[2] = fl;
		g_tx_dataframe.data[3] = trac;
		g_tx_dataframe.data[4] = temperature;
		g_tx_dataframe.data[5] = accident;
		
		printf("\nengine transmit %c",g_tx_dataframe.data[1]);
		printf("\nfuel  transmit%c",g_tx_dataframe.data[2]);
		printf("\ntract transmit%c",g_tx_dataframe.data[3]); 
		
		/* if(g_rx_dataframe.data[0]==0 || g_rx_dataframe.data[0]<=2)
			   {
				   LED4=LED_OFF;
				   LED6=LED_ON;
				   lcd_display(LCD_LINE3, "BATTERY LOW");
			   }
			   else
			   {
	              LED4=LED_ON;
				  LED6=LED_OFF;
				  lcd_display(LCD_LINE3, "BATTERY OK");
				}
				  break;
		  			if (g_rx_dataframe.data[1]=='R')// engine chk
			{
					sprintf(lcd_out,"Engine high ");
    				lcd_display(LCD_LINE4,lcd_out);
					LED6=0;
			}
			else
			{
					sprintf(lcd_out,"Engine low ");
    				lcd_display(LCD_LINE4,lcd_out);
					LED4=0;
			}
			if (g_rx_dataframe.data[2]=='R')// fuel check
			{
				sprintf(lcd_out,"Fuel high");
    			lcd_display(LCD_LINE5,lcd_out);
				LED10=0;
			}
			else
			{
				sprintf(lcd_out,"Fuel low");
    			lcd_display(LCD_LINE5,lcd_out);
				LED8=0;
			}
			if (g_rx_dataframe.data[3]=='R')// traction check 
			{
				sprintf(lcd_out,"Tract high");
    			lcd_display(LCD_LINE6,lcd_out);
				LED14=0;
				
			}
			else
			{
				sprintf(lcd_out,"Tract low");
    			lcd_display(LCD_LINE6,lcd_out);
				LED12=0;
			}*/
		#if TEST_FIFO
	        uint8_t	i = 0;
	    #endif
    
	    if (FRAME_ID_MODE == STD_ID_MODE )
	    {
	        R_CAN_TxSet(0, CANBOX_TX, &g_tx_dataframe, DATA_FRAME);
		
	    }
	    else
	    {   /* Extended ID mode. */
	
	        R_CAN_TxSetXid(0, CANBOX_TX, &g_tx_dataframe, DATA_FRAME); 
		
	    }

	    #if TEST_FIFO
	    /* Send three more to fill FIFO. */
	    for (i == 0; i < 3; i++)
	    {
	        if (FRAME_ID_MODE == STD_ID_MODE )
	        {
	            R_CAN_TxSetFifo(g_can_channel, &tx_fifo_dataframe, DATA_FRAME);
		
			}
	        else
	        {
	            R_CAN_TxSetFifoXid(g_can_channel, &tx_fifo_dataframe, DATA_FRAME);
	        }
	    }
    
	    #ifdef USE_CAN_POLL
	    /* This flag will not be set by TX FIFO interrupt, so set it here. */
	    tx_fifo_flag = 1;
	    #endif
	    #endif*/
	

		


      check_can_errors();

        if (can_state[0] != R_CAN_STATUS_BUSOFF)
        {
            #if (USE_CAN_POLL == 1)
            can_poll_demo();
            #else
            can_int_demo();
            #endif 
        }
        else
            /* Bus Off. */
      {
            //lcd_display(LCD_LINE7, "App in	");
            lcd_display(LCD_LINE8, "Bus Off ");

            /* handle_can_bus_state() will restart app. */
    //        lcd_flash();
       }

        /* Reset receive/transmit indication. */
       // LED6 = LED_OFF;
       // LED7 = LED_OFF;
   //} // End of while loop
}/* End function main(). */


#if (USE_CAN_POLL == 1)
/*****************************************************************************
* Function name:    can_poll_demo
* Description  : 	POLLED CAN demo version
* Arguments    :    none
* Return value : 	none
*****************************************************************************/
static void can_poll_demo(void)
{
    uint32_t	api_status = R_CAN_OK;
    uint8_t     disp_buf[13] = {0}; /* Temporary storage for display strings. */    

    /*** TRANSMITTED any frames? */
    api_status = R_CAN_TxCheck(g_can_channel, CANBOX_TX);

    if (R_CAN_OK == api_status)
    {
        //LED6 = LED_ON;		
        //lcd_display(LCD_LINE7, "TxChk OK");

       /*****************************************************
	    sprintf((char *)disp_buf, "%02X%02X%02X%02X",  
            g_tx_dataframe.data[0], 
            g_tx_dataframe.data[1],
            g_tx_dataframe.data[2], 
            g_tx_dataframe.data[3] );****************************/

        //lcd_display(LCD_LINE8, disp_buf);                          
        lcd_flash();
    }

    /* Since we are always polling for transmits, api_status for R_CAN_TxCheck 
      will most often be other than R_CAN_OK, don't show that in this demo.*/

    /*** RECEIVED any frames? */
    api_status = R_CAN_RxPoll(g_can_channel, CANBOX_RX);

    if (R_CAN_OK == api_status)
    {
        //LED5 = LED_ON;
        //lcd_display(LCD_LINE7, "Rx Poll OK");        
        lcd_flash();

        /* Read CAN data and show. */
        api_status = R_CAN_RxRead(g_can_channel, CANBOX_RX, &g_rx_dataframe);

        //lcd_display(LCD_LINE7, "Rx Read: ");
	
		/************************************************
        sprintf((char *)disp_buf, "%02X%02X%02X%02X",  
            g_rx_dataframe.data[0], 
            g_rx_dataframe.data[1],
            g_rx_dataframe.data[2], 
            g_rx_dataframe.data[3] );**************************/
			
			
		//	printf(" g_rx_data = %0X02", g_rx_dataframe.data[0]);
		

        //lcd_display(LCD_LINE8, disp_buf);               
        lcd_flash();

        if (R_CAN_MSGLOST == api_status)
        {
            //lcd_display(LCD_LINE7, "MSGLOST");            
            lcd_flash();
        }

        LED5 = LED_OFF;		
    }
}/* End function can_poll_demo() */

#else

/*****************************************************************************
* Function name:    can_int_demo
* Description  : 	INTERRUPT driven CAN demo version
* Arguments    :    none
* Return value : 	none
*****************************************************************************/
static void can_int_demo(void)
{
    uint32_t	api_status = R_CAN_OK;
    uint8_t     disp_buf[13] = {0}; /* Temporary storage for display strings. */ 

    /************************************************************************
    * Using CAN INTERRUPTS.													*
    * See also r_can_api.c for the ISR example.								*
    *************************************************************************/
    /* TRAN
    if (CAN0_tx_sentdata_flag)SMITTED any frames? If flag can0_tx_sentdata_flag was set by the 
    CAN Tx ISR, the frame was sent successfully. */
    {
        CAN0_tx_sentdata_flag = 0; /* Clear the flag for next time. */
        //LED6 = LED_ON;

        /* Show CAN frame was sent. */
        //lcd_display(LCD_LINE7, "Tx OK   ");

        /*sprintf((char *)disp_buf, "%02X%02X%02X%02X",  
            g_tx_dataframe.data[0], 
            g_tx_dataframe.data[1],
            g_tx_dataframe.data[2], 
            g_tx_dataframe.data[3] );*/

        //lcd_display(LCD_LINE8, disp_buf);
       // lcd_flash();
    }

    if (CAN0_tx_remote_sentdata_flag)
    {
        CAN0_tx_remote_sentdata_flag = 0;
        lcd_display(LCD_LINE7, "TxRemote"); 
        lcd_flash();
    }					

    /*** RECEIVED any frames? Only need to check if flag is set by CAN Rx ISR.
    Will only receive own frames in CAN port test modes 0 and 1. */
    if (CAN0_rx_newdata_flag)
    {
        CAN0_rx_newdata_flag = 0;
        /*  ******
		LED4 = LED_ON;*/

       // lcd_display(LCD_LINE7, "Rx OK. Read:"); 

        /* Read CAN data. */
        api_status = R_CAN_RxRead(g_can_channel, CANBOX_RX, &g_rx_dataframe);

        /* Displaying the Recieved CAN Frame on LCD Line2 */
	   
	 
	    sprintf((char *)disp_buf, "%X",  
            g_rx_dataframe.data[0] 
           );
		   
		   
		   printf("\ng_rx_dataframe = %X",g_rx_dataframe.data[0]);
		   
// while(1)
//		   {
			  	printf("\nengine receive %c",g_tx_dataframe.data[1]);
				printf("\nfuel  receive%c",g_tx_dataframe.data[2]);
				printf("\ntract receive%c",g_tx_dataframe.data[3]); 
		
		
			  	if(g_rx_dataframe.data[0]==0 || g_rx_dataframe.data[0]<=2)
			   {
				   LED4=LED_OFF;
				   LED6=LED_ON;
				   lcd_display(LCD_LINE3, "BATTERY LOW");
			   }
			   else
			   {
	              LED4=LED_ON;
				  LED6=LED_OFF;
				  lcd_display(LCD_LINE3, "BATTERY OK");
				}
				 
		  			if (g_rx_dataframe.data[1]=='R')// engine chk
			{
					sprintf(lcd_out,"Engine high ");
    				lcd_display(LCD_LINE4,lcd_out);
					LED11=LED_ON;
					LED15=LED_OFF;
			}
			else
			{
					sprintf(lcd_out,"Engine low ");
    				lcd_display(LCD_LINE4,lcd_out);
					LED15=LED_ON;
					LED11=LED_OFF;
			}
			if (g_rx_dataframe.data[2]=='R')// fuel check
			{
				sprintf(lcd_out,"Fuel high");
    			lcd_display(LCD_LINE5,lcd_out);
				LED10=LED_ON;
				LED8=LED_OFF;
			}
			else
			{
				sprintf(lcd_out,"Fuel low");
    			lcd_display(LCD_LINE5,lcd_out);
				LED8=LED_ON;
				LED10=LED_OFF;
				
			}
			if (g_rx_dataframe.data[3]=='R')// traction check 
			{
				
				sprintf(lcd_out,"Tract high");
    			lcd_display(LCD_LINE6,lcd_out);
				LED14=LED_ON;
				LED12=LED_OFF;
			}
			else
			{
				sprintf(lcd_out,"Tract low");
    			lcd_display(LCD_LINE6,lcd_out);
				LED12=LED_ON;
				LED14=LED_OFF;
				
			}
			 if (g_rx_dataframe.data[4]>(28*28))
			  {
				sprintf(lcd_out, "  Accident ");
				lcd_display(LCD_LINE7, lcd_out);	
				LED15=LED_ON;
				LED13=LED_OFF;
				
			  }
			  else
			{
				sprintf(lcd_out, "   ");
				lcd_display(LCD_LINE7, lcd_out);
				LED13=LED_ON;
				LED15=LED_OFF;
			}
			
				 if (g_rx_dataframe.data[5]>280)
			  {
				sprintf(lcd_out, "High temp");
				lcd_display(LCD_LINE8, lcd_out);	
				LED11=LED_ON;
				LED9=LED_OFF;
				
			  }
			  else
			{
				sprintf(lcd_out, "Norm temp");
				lcd_display(LCD_LINE8, lcd_out);
				LED9=LED_ON;
				LED11=LED_OFF;
			}
			
//			   break;						
		  
//		   } // End of while 
		   
		/*
        /* Display the formatted string. */                    
       // lcd_display(LCD_LINE2, disp_buf);

        /* Clear the displayed data after a pause. */         
       // lcd_flash();

        /* Display error, if any. */
        if (R_CAN_MSGLOST == api_status)
        {
            //lcd_display(LCD_LINE7, "MSGLOST"); 
            lcd_flash();
        }
		/*  ******
        LED4 = LED_OFF; */
    }

    if (CAN0_rx_test_newdata_flag)
    {
        CAN0_rx_test_newdata_flag = 0;
        /*  ******
		LED4 = LED_ON;*/
        lcd_display(LCD_LINE6, "Rx Test"); 
        lcd_flash();
    }

    /* Set up remote reply if remote request came in. */
    if (1 == CAN0_rx_g_remote_frame_flag)
    {
        CAN0_rx_g_remote_frame_flag = 0;
        g_remote_frame.data[0]++;
        
        if (FRAME_ID_MODE == STD_ID_MODE )
        {
            R_CAN_TxSet(g_can_channel, CANBOX_REMOTE_TX, &g_remote_frame, DATA_FRAME);   
        }
        else
        {
            R_CAN_TxSetXid(g_can_channel, CANBOX_REMOTE_TX, &g_remote_frame, DATA_FRAME);             
        }    
    }
	/*  ******
    LED4 = LED_OFF;*/

}/* End function can_int_demo(). */

#endif  /* USE_CAN_POLL == */


/*****************************************************************************
* Function name:    init_can_app
* Description  : 	Initialize CAN demo application
* Arguments    :    none
* Return value : 	none
*****************************************************************************/
static uint32_t init_can_app(void)
{	
    uint32_t	api_status = R_CAN_OK;
    uint32_t    i; /* Common loop index variable. */

    can_state[0] = R_CAN_STATUS_ERROR_ACTIVE;

    for (i = 0; i < MAX_CHANNELS; i++) /* Initialize status for all channels. */
    {
        error_bus_status[i] = R_CAN_STATUS_ERROR_ACTIVE;
        error_bus_status_prev[i] = R_CAN_STATUS_ERROR_ACTIVE;            
    }
    
    /* Configure mailboxes in Halt mode. */
    api_status |= R_CAN_Control(g_can_channel, HALT_CANMODE);

    /********	Init demo to recieve data	********/	
    /* Use API to set one CAN mailbox for demo receive. */
    /* Standard id. Choose value 0-0x07FF (2047). */
    if (FRAME_ID_MODE == STD_ID_MODE)
    { 
        api_status |= R_CAN_RxSet(g_can_channel, CANBOX_RX, g_rx_id_default, DATA_FRAME);
        
        /* Mask for receive box. Write to mask only in Halt mode. */
        /* 0x7FF = no mask. 0x7FD = mask bit 1, for example; If receive ID is set to 1, both
           ID 1 and 3 should be received. */
        R_CAN_RxSetMask( g_can_channel, CANBOX_RX, 0x7FF);           
    }
    else
    {
        api_status |= R_CAN_RxSetXid(g_can_channel, CANBOX_RX, g_rx_id_default, DATA_FRAME);
        
        /* Mask for receive box. Write to mask only in Halt mode. */
        /* 0x1FFFFFFF = no mask. 0x1FFFFFFD = mask bit 1, for example; If receive ID is set to 1, both
           ID 1 and 3 should be received. */
        R_CAN_RxSetMask( g_can_channel, CANBOX_RX, 0x1FFFFFFD);                  
    }

    /********	Init. demo Tx dataframe RAM structure	********/	
    /* Standard id. Choose value 0-0x07FF (2047). */
    g_tx_dataframe.id		=	g_tx_id_default;
    g_tx_dataframe.dlc		=	8;
    g_tx_dataframe.data[0]	=	0x00;
    g_tx_dataframe.data[1]	=	0x11;
    g_tx_dataframe.data[2]	=	0x22;
    g_tx_dataframe.data[3]	=	0x33;
    g_tx_dataframe.data[4]	=	0x44;
    g_tx_dataframe.data[5]	=	0x55;
    g_tx_dataframe.data[6]	=	0x66;
    g_tx_dataframe.data[7]	=	0x77;

    /* API to send will be set up in SW1Func() in file switches.c. */
    api_status |= R_CAN_Control(g_can_channel, OPERATE_CANMODE);

    /*************** Init. remote dataframe response **********************/
    g_remote_frame.id = REMOTE_TEST_ID;

    /* Length is specified by the remote request. */
    /* Stuff with some data.. */
    for (i = 0; i < 8; i++)
    {
        g_remote_frame.data[i] = i;
    }    
    
    /* Prepare mailbox for Tx. */    	
    if (FRAME_ID_MODE == STD_ID_MODE)
    {
        R_CAN_RxSet(g_can_channel, CANBOX_REMOTE_RX, REMOTE_TEST_ID, REMOTE_FRAME);
    }
    else
    {
        R_CAN_RxSetXid(g_can_channel, CANBOX_REMOTE_RX, REMOTE_TEST_ID, REMOTE_FRAME);        
    }
    /***********************************************************************/

    /* Set frame buffer id so LCD shows correct receive ID from start. */
    g_rx_dataframe.id = g_rx_id_default;

    return api_status;

} /* End function init_can_app(). */


/*****************************************************************************
* Function name:    check_can_errors
* Description  : 	Check for all possible errors, in app and peripheral. Add 
*				    checking for your app here.
* Arguments    :    none
* Return value : 	none
*****************************************************************************/
static void check_can_errors(void)
{
    uint8_t disp_buf[13] = {0}; /* Temporary storage for display strings. */

    /* Error passive or more? */
    handle_can_bus_state(g_can_channel);

    if (app_err_nr)
    {
        /* Show error to user */
        /* RESET ERRORs with SW1. */
        LED7 = LED_ON;

        lcd_display(LCD_LINE7,"App err");      
        sprintf((char *)disp_buf, "    %02X", app_err_nr);
        lcd_display(LCD_LINE8, disp_buf);        
        lcd_flash();

        LED7 = LED_OFF;
    }
    
}/* End function check_can_errors(). */


/*****************************************************************************
* Function name:    handle_can_bus_state
* Description  : 	Check CAN peripheral bus state.
* Arguments    :    Bus number, 0 or 1.
* Return value : 	none
*****************************************************************************/
static void handle_can_bus_state(uint8_t ch_nr)
{
    can_frame_t err_tx_dataframe;
    uint8_t disp_buf[13] = {0}; /* Temporary storage for display strings. */

    /* Has the status register reached error passive or more? */
    if (ch_nr < MAX_CHANNELS)
    {
        error_bus_status[ch_nr] = R_CAN_CheckErr(ch_nr);
    }
    else
    {
        return;
    }

    /* Tell user if CAN bus status changed.
    All Status bits are read only. */
    if (error_bus_status[ch_nr] != error_bus_status_prev[ch_nr])
    {	
        switch (error_bus_status[ch_nr])
        {
            /* Error Active. */
            case R_CAN_STATUS_ERROR_ACTIVE:

                /* Only report if there was a previous error. */
                if (error_bus_status_prev[ch_nr] > R_CAN_STATUS_ERROR_ACTIVE)
                {
                    if (CH_0 == ch_nr)
                    {
                        //lcd_display(LCD_LINE6, "Bus0: OK");	 
                    }
                    else
                    {
                        //lcd_display(LCD_LINE6, "Bus1: OK");                       
                    }
                    
                    delay(0x400000);    /* Allow user time to see display. */
                    //lcd_display(LCD_LINE6, "            ");	 /* Now clear it. */                   
                }

                /* Restart if returned from Bus Off. */
                if (R_CAN_STATUS_BUSOFF == error_bus_status_prev[ch_nr])
                {
                    /* Restart CAN */
                    if (R_CAN_OK != R_CAN_Create(ch_nr))
                    {
                        app_err_nr |= APP_ERR_CAN_PERIPH;
                    }

                    /* Restart CAN demos even if only one channel failed. */
                    init_can_app();
                }
            break;	

            /* Error Passive. */
            case R_CAN_STATUS_ERROR_PASSIVE:
            /* Continue into Busoff case to display. */

            case R_CAN_STATUS_BUSOFF:
             /* Bus Off. */
             
            default:
                if (CH_0 == ch_nr)
                {
                    sprintf((char *)disp_buf, "bus0: %02X", error_bus_status[ch_nr]);                 
                    //lcd_display(LCD_LINE6, disp_buf);
                }
                else if (CH_1 == ch_nr)
                {
                    sprintf((char *)disp_buf, "bus1: %02X", error_bus_status[ch_nr]);                 
                    //lcd_display(LCD_LINE6, disp_buf);
                }
                else if (CH_2 == ch_nr)
                {
                    sprintf((char *)disp_buf, "bus2: %02X", error_bus_status[ch_nr]);                 
                    //lcd_display(LCD_LINE6, disp_buf);
                }
                else
                {
                    /* No else. */
                }

                delay(0x400000);
                nr_times_reached_busoff[ch_nr]++;
            break;
        }
        
        error_bus_status_prev[ch_nr] = error_bus_status[ch_nr];

        /* Transmit CAN bus status change */
        err_tx_dataframe.id = 0x700 + ch_nr;
        err_tx_dataframe.dlc =	1;
        err_tx_dataframe.data[0] = error_bus_status[ch_nr];

        /* Send Error state on both channels. Maybe at least one is up. 
        Warning: If CAN1 and CAN1 are connected to eachother, they will try to
        send practically simultaneously. Let this be a lesson; sending the same 
        ID from two nodes onto the same bus at the same time is very hazardous
        as the arbitration cannot take place. Only use both lines below if 
        CAN1 and CAN1 are on different buses. */
        if (FRAME_ID_MODE == STD_ID_MODE )
        {
            R_CAN_TxSet(g_can_channel, CANBOX_TX, &err_tx_dataframe, DATA_FRAME);   
        }
        else
        {
            R_CAN_TxSetXid(g_can_channel, CANBOX_TX, &err_tx_dataframe, DATA_FRAME);
        } 

    }

}/* End function handle_can_bus_state() */

/*******************************************************************************
* Function name:    reset_all_errors
* Description  : 	Reset all types of errors, application and CAN peripeheral errors.
* Arguments    :    none
* Return value : 	CAN API code
*******************************************************************************/
uint32_t reset_all_errors(void)
{		
    uint32_t status = 0;

    /* Reset errors */
    app_err_nr = APP_NO_ERR;

    error_bus_status[0] = 0;
    error_bus_status[1] = 0;

    /* You can chooose to not reset error_bus_status_prev; if there was an error, 
    keep info to signal recovery */
    error_bus_status_prev[0] = 0; 
    error_bus_status_prev[1] = 0;

    nr_times_reached_busoff[0] = 0;
    nr_times_reached_busoff[1] = 0;

    /* Reset Error Judge Factor and Error Code registers */
    CAN0.EIFR.BYTE /*= CAN0.EIFR.BYTE*/ = 0;

    /* Reset CAN0 Error Code Store Register (ECSR). */
    CAN0.ECSR.BYTE = 0;

    /* Reset CAN0 Error Counters. */
    CAN0.RECR = 0;
    CAN0.TECR = 0;

    return status;
}/* End function reset_all_errors() */


/*******************************************************************************
* Function name:    delay
* Description  :    Demo delay
* Arguments    :    32-bit delay count value
* Return value : 	none
*******************************************************************************/
#pragma noinline(delay)
void delay(uint32_t n)
{
    uint32_t i;

    for(i = 0; i < n; i++)
    {
        nop();
    }

}/* End function delay() */


/*******************************************************************************
* Function name:    lcd_flash
* Description  :    Erases the frequently updated display area after a delay.
* Arguments    :    none
* Return value :    none
*******************************************************************************/
void lcd_flash(void)
{
    delay(LCD_DELAY); 
    //lcd_display(LCD_LINE7, "            ");				 
    //lcd_display(LCD_LINE8, "            "); 

} /* End function lcd_flash(). */


/*******************************************************************************
CAN INTERRRUPTS 
Interrupts are to be duplicated for each CAN channel used except for the Error 
interrupt which handles all channels in a group. 
Vectors are set according to the channel.
*******************************************************************************/
#if (USE_CAN_POLL == 0)
/*****************************************************************************
* Function name:    CAN0_TXM0_ISR
* Description  :    CAN0 Transmit interrupt. Check which mailbox transmitted 
*                   data and process it.	
* Arguments    :    N/A
* Return value :    N/A

*****************************************************************************/
#pragma interrupt CAN0_TXM0_ISR(vect=VECT_CAN0_TXM0, enable) 
void CAN0_TXM0_ISR(void)
{
    uint32_t api_status = R_CAN_OK;

    api_status = R_CAN_TxCheck(CH_0, CANBOX_TX);

    if (R_CAN_OK == api_status)
    {
        CAN0_tx_sentdata_flag = 1;
    }

    api_status = R_CAN_TxCheck(CH_0, CANBOX_REMOTE_TX);

    if (R_CAN_OK == api_status)
    {
        CAN0_tx_remote_sentdata_flag = 1;
    }

    /* Use mailbox search reg. Should be faster than above if a lot of mailboxes to check. 
    Not verified. */
}/* end CAN0_TXM0_ISR() */


/*****************************************************************************
* Function name:    CAN0_RXM0_ISR
* Description  :    CAN0 Receive interrupt.
*   				Check which mailbox received data and process it.
* Arguments    :    N/A
* Return value :    N/A
*****************************************************************************/
#pragma interrupt CAN0_RXM0_ISR(vect=VECT_CAN0_RXM0, enable)
void CAN0_RXM0_ISR(void)
{
    /* Use CAN API. */
    uint32_t api_status = R_CAN_OK;

    api_status = R_CAN_RxPoll(CH_0, CANBOX_RX);

    if (R_CAN_OK == api_status)
    {
        CAN0_rx_newdata_flag = 1;
    }

    api_status = R_CAN_RxPoll(CH_0, CANBOX_REMOTE_RX);

    if (R_CAN_OK == api_status)
    {
        /* REMOTE_FRAME FRAME REQUEST RECEIVED */
        /* Do not set BP on the next line to check for Remote frame. By the time you 
        continue, the recsucc flag will already have changed to be a trmsucc flag in 
        the CAN status reg. */

        /* Reset of the receive/transmit flag in the MCTL register will be done by 
        set_remote_reply_std_CAN0(). */

        /* Set flag to inform application. */
        CAN0_rx_g_remote_frame_flag = 1;

        g_remote_frame.dlc = (uint8_t)(CAN0.MB[CANBOX_REMOTE_RX].DLC);		

        /* Reset NEWDATA flag since we won't be reading the mailbox. */
        CAN0.MCTL[CANBOX_REMOTE_RX].BIT.RX.NEWDATA = 0;
    }

    /* Use mailbox search reg. Should be faster if a lot of mailboxes to check. */

}/* end CAN0_RXM0_ISR() */

/*****************************************************************************
* Function name:    CAN_ERS_ISR
* Description  :    CAN Group Error interrupt.
*   				Check which CAN channel is source of interrupt
* Arguments    :    N/A
* Return value :    N/A
*****************************************************************************/
#pragma interrupt	CAN_ERS_ISR(vect=VECT_ICU_GROUPE0, enable)
void CAN_ERS_ISR(void)
{
    /* Error interrupt can have multiple sources. Check interrupt flags to id source. */
    if (IS(CAN0, ERS0))
    {
        //LED7 = LED_ON;		/*TODO: additional error handling/cause identification */
        CLR(CAN0, ERS0) = 1;	/* clear interrupts */         
    }
    if (IS(CAN1, ERS1))
    {
        //LED7 = LED_ON;		/*TODO: additional error handling/cause identification */	
        CLR(CAN1, ERS1) = 1;	/* clear interrupts */			
    }
    if (IS(CAN2, ERS2))
    {
       // LED7 = LED_ON;		/*TODO: additional error handling/cause identification */		
    }

    nop();
}/* end CAN_ERS0_ISR() */

#endif /* USE_CAN_POLL == 0 */

/* eof */

/*****************************************************************************
* Function name:    RTC_display
* Description  :    RTC display 
*   				To display the RTC 
* Arguments    :    N/A
* Return value :    N/A
*****************************************************************************/
void RTC_display(void)
{
 	volatile uint8_t dummy;
    char date_d[13],time_d[13];
    size_t size;	
    time.second = RTC.RSECCNT.BYTE;         /* Read the BCD-code second */
    time.minute = RTC.RMINCNT.BYTE;         /* Read the BCD-code minute */
    time.hour = RTC.RHRCNT.BYTE;            /* Read the BCD-coded hour */
    time.dayweek = RTC.RWKCNT.BYTE;         /* Read the day of the week */
    time.day = RTC.RDAYCNT.BYTE;            /* Read the BCD-coded day */
    time.month = RTC.RMONCNT.BYTE;          /* Read the BCD-coded month */
    time.year = 0x2000 | RTC.RYRCNT.WORD;   /* Read the BCD-coded year */
/* Sending entire string to hyperterminal*/
	sprintf(date_d,"D:%x-%0.2x-%0.2x",time.year,time.month,time.day);	
	sprintf(time_d,"T:%0.2x:%0.2x:%0.2x",time.hour,time.minute,time.second);
	lcd_display(LCD_LINE1,date_d);
	lcd_display(LCD_LINE2,time_d);
}


/******************************************************/






/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 05.01.2012     1.00        First release
*******************************************************************************/

/*******************************************************************************
Includes   <System Includes> , "Project Includes"
*******************************************************************************/
/* Defines string functions used in this file */
#include <string.h>
#include <stdio.h>
#include <machine.h>

#include "file1.h"
#include "platform.h"
#include "r_riic_rx600.h"
#include "r_riic_rx600_master.h"
#include "riic_master_main.h"
#include "accelerometer_demo.h"

/* Defines ADXL345 parameters */
#include "ADXL345.h"

/*******************************************************************************
Macro definitions
*******************************************************************************/
#define ACCELEROMETER_DEBUG
void accel(char);
/*******************************************************************************
Local global variables
*******************************************************************************/
static volatile int16_t g_accel_x_zero;
static volatile int16_t g_accel_y_zero;
static volatile int16_t g_accel_z_zero;
static volatile int16_t g_accel_x;
static volatile int16_t g_accel_y;
static volatile int16_t g_accel_z;


/*******************************************************************************
* Local Function Prototypes
*******************************************************************************/
static int16_t  accel_axis_read(uint8_t);
static bool   accel_selftest( void );
static riic_ret_t accelerometer_write (uint8_t riic_channel,
                                uint8_t slave_addr,
                                uint8_t register_number, 
                                uint8_t *source_buff, 
                                uint32_t num_bytes);
                                
static riic_ret_t accelerometer_read (uint8_t riic_channel,
                               uint8_t slave_addr,
                               uint8_t register_number, 
                               uint8_t *dest_buff, 
                               uint32_t num_bytes);
float accident;	
                               

/*******************************************************************************
* Function name: accelerometer_write
* Description  : Writes a specified number of bytes starting from the specfied 
*                accelerometer register. If more than 1 byte is requested then
*                the accelerometer will automatically increment to the next 
*                register number with each sequencial write.
* Arguments    : riic_channel - 
*                   Which IIC channel of the MCU to use.
*                slave_addr -
*                   IIC slave address of the accelerometer.
*                register_number - 
*                   Which register of the accelerometer to be written.
*                (uint8_t*)source_buff - 
*                   pointer to the buffer where data will be copied from.              
*                num_bytes -
*                   The number of bytes to be written
*
* Return value : ret : RIIC result_code
*******************************************************************************/
static riic_ret_t accelerometer_write (uint8_t riic_channel,
                                uint8_t slave_addr,
                                uint8_t register_number, 
                                uint8_t *source_buff, 
                                uint32_t num_bytes)
{
    uint8_t     addr_and_register[2]; /* Storage for the slave address and target register. */
    riic_ret_t  ret = RIIC_OK;
 
    /* To write to a specific register in the accelerometer, first transmit the 
       accelerometer I2C slave address together with the register number. */         
    addr_and_register[0] = slave_addr;    /* The 7-bit I2C address of the ADXL345 and the R/W bit. */ 
    addr_and_register[1] = register_number;
               
    ret |= R_RIIC_MasterTransmitHead(CHANNEL_0, addr_and_register, 2);
    /* Now write the data from the source buffer into the target register. */
    ret |= R_RIIC_MasterTransmit(CHANNEL_0, source_buff, num_bytes);
    
    return ret;
} /* End of function accelerometer_write(). */


/******************************************************************************
* Function name: accelerometer_read
* Description  : Reads a specified number of bytes starting from the specfied 
*                accelerometer register. If more than 1 byte is requested then
*                the accelerometer will automatically increment to the next 
*                register number with each sequencial read.
* Arguments    : riic_channel - 
*                   Which IIC channel of the MCU to use.
*                slave_addr -
*                   IIC slave address of the accelerometer.
*                register_number - 
*                   Which register of the accelerometer to be written.
*                (uint8_t*)dest_buff - 
*                   pointer to the buffer into which the read data will be stored.              
*                num_bytes -
*                   The number of bytes to be read from the accelerometer
* 
* Return value : ret : RIIC result_code
******************************************************************************/
static riic_ret_t accelerometer_read (uint8_t riic_channel,
                               uint8_t slave_addr,
                               uint8_t register_number, 
                               uint8_t *dest_buff, 
                               uint32_t num_bytes)
{ 
    uint8_t     addr_and_register[2]; /* Storage for the slave address and target register. */
    riic_ret_t  ret = RIIC_OK;
    
    /* To read from a specific register in the accelerometer, first transmit the 
       accelerometer I2C slave address together with the register number. */         
    addr_and_register[0] = slave_addr; /* The 7-bit I2C address of the ADXL345 and the R/W bit. */ 
    addr_and_register[1] = register_number;
            
    ret |= R_RIIC_MasterTransmitHead(CHANNEL_0, addr_and_register, 2);
    /* Now read the data from the target register into the destination buffer. */    
    ret |= R_RIIC_MasterReceive(CHANNEL_0, slave_addr, dest_buff, num_bytes);
    
    return ret;    
} /* End of function accelerometer_read(). */


/******************************************************************************
* Function name:  accelerometer_init
* Description  : 
* Argument     : none
* Return value : riic_ret_t : RIIC result code
*******************************************************************************/
riic_ret_t accelerometer_init(void)
{
    bool            err = true;    /* Declare error flag */
    uint8_t         target_data; 
    riic_ret_t      ret;

    /* Read the DEVID register to verify the presence of the accelerometer device. */    
    ret |= accelerometer_read(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_ID_REG, &target_data, 1);
                                                               
    if ((target_data != ADXL345_DEVICE_ID) || (RIIC_OK != ret))
    {   /* Add to error return information here if desired. */
        return ret;
    }

    /* Set up the accelerometer data format register. */
    target_data = 3; /* Set data format register range bits to +/- 16g. */
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_DATA_FORMAT_REG, &target_data, 1);

    /* Put accelerometer FIFO into bypass mode. */
    target_data = 0; /* FIFO bypass mode. */
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_FIFO_CTL_REG, &target_data, 1);

    /* Set the measure bit in the accelerometer power control register. */                                     
    target_data = 8; /* Measure bit. */
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_POWER_CTL_REG, &target_data, 1);                                   

    /* Get baseline readings to calibrate accelerometer. */
    g_accel_x_zero = 0;
    g_accel_y_zero = 0;
    g_accel_z_zero = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        g_accel_x_zero += accel_axis_read(0x32);
        g_accel_y_zero += accel_axis_read(0x34);
        g_accel_z_zero += accel_axis_read(0x36);            
    }

    /* Determine the average reading. */
    g_accel_x_zero = g_accel_x_zero / 8;
    g_accel_y_zero = g_accel_y_zero / 8;
    g_accel_z_zero = g_accel_z_zero / 8;
    /* Run self test to see if the accelerometer is working. */
    err &= accel_selftest();

    /* Activate accelerometer X, Y Z to detect activity. */
    target_data = 0x70;    
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_ACT_INACT_CTL_REG, &target_data, 1);  

    //LED4 = LED5 = LED6 = LED7 = LED8 = LED9 = LED10 = LED11 = LED12 = LED13 = LED14 = LED15 = LED_OFF;

    return ret;
} /* End of function accelerometer_init(). */


/******************************************************************************
* Function name: accelerometer_demo_update
* Description  : This function reads the accelerometer to update the current 
*                xyz position values and light the corresponding LEDs. 
*                Called by the CMT callback function, this function is
*                executed after every period of the CMT timer. 
* Argument     : none
* Return value : none
*******************************************************************************/
void accelerometer_demo_update(void)
{

#ifdef ACCELEROMETER_DEBUG    
    /* Declare display buffer */
    uint8_t  lcd_buffer[13];
    int16_t adjusted_z;     
#endif    

    int16_t adjusted_x;
    int16_t adjusted_y;
  
    int16_t slope = 0;

    g_accel_x = accel_axis_read(ADXL345_DATAX0_REG);
    g_accel_y = accel_axis_read(ADXL345_DATAY0_REG);
    g_accel_z = accel_axis_read(ADXL345_DATAZ0_REG);        

    adjusted_x = g_accel_x - g_accel_x_zero;
    adjusted_y = g_accel_y - g_accel_y_zero;


    /* calculate the slope, make sure not dividing by zero */
    if ( adjusted_x == 0 )
    {
        adjusted_x = 1;
    }
    
    slope = (100 * adjusted_y) / adjusted_x;
	
	
	accident=(adjusted_x)*(adjusted_x)+(adjusted_y)*(adjusted_y)+(adjusted_z)*(adjusted_z);
	if(accident>(28*28))
	{
		printf("\naccident");
		char acc;
		 acc='R';
		
		void accel(acc);
	}

#ifdef ACCELEROMETER_DEBUG 
    adjusted_z = g_accel_z - g_accel_z_zero;
           
    //sprintf((char *)lcd_buffer, " x = %d" , adjusted_x);
    //lcd_display(LCD_LINE5, lcd_buffer);

//    sprintf((char *)lcd_buffer, " y = %d" , adjusted_y);    
  //  lcd_display(LCD_LINE6, lcd_buffer);

    //sprintf((char *)lcd_buffer, " z = %d" , adjusted_z);    
    //lcd_display(LCD_LINE7, lcd_buffer);
#endif

    /* Turn off all LEDs. */
    //LED4 = LED5 = LED6 = LED7 = LED8 = LED9 = LED10 = LED11 = LED12 = LED13 = LED14 = LED15 = LED_OFF;
    
    /* Ignore baseline noise. */    
   /* while (accident>(28*28))
    {
        LED13 = 1;
		LED15 = 0;
    }
    LED13 = 0;
    /* Activate the appropriate LED that indicates the direction of board tilt. */
    
} /* End of function cmt_accelerometer_callback(). */


/*******************************************************************************
* Function name: accel_selftest
* Description  : This function invokes the acceleromter's self-test feature.
*                Done to check if the unit is working by testing its electrical
*                and mechanical systems.
* Argument     : none
* Return value : bool   true - 
*                           pass 
*                       false - 
*                           fail
*******************************************************************************/
static bool  accel_selftest( void )
{
    uint8_t     target_data;
    riic_ret_t  ret;           
    bool        err = true;
 
    /* Set up the accelerometer data format register. */    
    target_data = 0x83;  /* Set the data format range bits to +/- 16g, and selftest mode bit. */
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_DATA_FORMAT_REG, &target_data, 1);

    /* Wait for self-test forces to act. */
    for (volatile uint16_t i = 0; i < 1000; i++)
    {
        ; /* Spin loop delay. */
    }

    g_accel_x = 0;
    g_accel_y = 0;
    g_accel_z = 0;

    /* Take an average of 8 readings per axis to serve as basis for range check. */
    for (uint16_t i = 0; i < 8; i++)
    {
        g_accel_x += accel_axis_read(ADXL345_DATAX0_REG);
        g_accel_y += accel_axis_read(ADXL345_DATAY0_REG);
        g_accel_z += accel_axis_read(ADXL345_DATAZ0_REG);        
    }
    /* Divide the 8 readings by 8 to obtain the average value. */
    g_accel_x = g_accel_x / 8;
    g_accel_y = g_accel_y / 8;
    g_accel_z = g_accel_z / 8;

    /* Normalize the self test values. */
    g_accel_x = SCALE_X(g_accel_x) - g_accel_x_zero;
    g_accel_y = SCALE_Y(g_accel_y) - g_accel_y_zero;
    g_accel_z = SCALE_Z(g_accel_z) - g_accel_z_zero;

    /* Range check self-test values. */
    err &= ((g_accel_x >   6) && (g_accel_x < 67))  ? true : false;
    err &= ((g_accel_y > -67) && (g_accel_y < -6))  ? true : false;
    err &= ((g_accel_z >  10) && (g_accel_z < 110)) ? true : false;

    /* Turn off self test mode. */                          
    target_data = 0x03;  /* Set the data format range bits to +/- 16g, and clear selftest mode bit. */
    ret |= accelerometer_write(RIIC_CHANNEL, ADXL345_ADDR, ADXL345_DATA_FORMAT_REG, &target_data, 1);
                                 
    if(ret)
    {
        err = false; /* Fail. */   
    }                             
    return err;                                 
} /* End of function accel_selftest(). */


/******************************************************************************
* Function name: accel_axis_read
* Description  : This function reads the accelerometer's X, Y or Z axis. 
* Argument     : uint8_t    axis -
*                   Which axis register to be read.
* Return value : short value of axis
*******************************************************************************/
static int16_t  accel_axis_read (uint8_t axis)
{
    int16_t    axis_val;    /* 16-bit storage for the x, y, or z axis data. */
    uint8_t    accel_reg;   /* Storage for the accelerometer register parameter. */
    uint8_t    accel_data[2];
    riic_ret_t ret;         /* Result code from the RIIC API functions. */
    
    accel_reg = axis;

    /* Uses RIIC to read the accelerometer axis data. */ 
    ret = accelerometer_read(RIIC_CHANNEL, ADXL345_ADDR, accel_reg, accel_data, 2);
    
    while (RIIC_OK != ret)
    {
        nop(); /* Stay here for debug of IIC error. */    
    }
    
    axis_val    = accel_data[1] << 8;
    axis_val   += accel_data[0];

    return axis_val;
} /* End of function accel_axis_read(). */





/**************************************************************************************************************/


/*******************************************************************************
* File Name     : thermal_sensor.c
* Version       : 1.0
* H/W Platform  : YRDKRX63N
* Description   : Driver for the ADT7420 I2C thermal sensor.
*******************************************************************************/
/*******************************************************************************
* History : DD.MM.YYYY     Version     Description
*         : 07.02.2012     1.00        First release
*******************************************************************************/

/*******************************************************************************
Includes   <System Includes> , "Project Includes"
*******************************************************************************/
#include <string.h>
#include <stdio.h>
#include <machine.h>
#include "file1.h"
#include "platform.h"
#include "r_riic_rx600.h"
#include "r_riic_rx600_master.h"
#include "riic_master_main.h"

#include "thermal_sensor_demo.h"
/* Defines ADT7420 parameters */
#include "ADT7420.h"


/*******************************************************************************
* Function name: thermal_sensor_init
* Description  : This function configures the ADT7420 thermal device. 
* Argument     : none
* Return value : riic_ret_t -
*                   RIIC return code
*******************************************************************************/
riic_ret_t thermal_sensor_init(void)
{
    uint8_t     target_data;
    uint8_t     addr_and_register[2]; /* Storage for the slave address and target register. */
    riic_ret_t  ret = RIIC_OK;

    /* To write to a specific register in the thermal sensor, first transmit its 
       I2C slave address together with the register number. */
    /* The 7-bit I2C address of the ADT7420 and the R/W bit. */            
    addr_and_register[0] = ADT7420_ADDR; 
    
    /* The register address of the configuration register. */
    addr_and_register[1] = ADT7420_CONFIG_REG;

    ret |= R_RIIC_MasterTransmitHead(CHANNEL_0, addr_and_register, 2);

    if (RIIC_OK == ret)
    {       
	    /* Configuration data: 0x00 = 13-bit resolution. */
	    /* Sign bit + 12 bits gives a temperature resolution of 0.0625°C.  */	
        target_data = 0x00; 
    
        /* Now write the data from the source buffer into the target register. */   
        ret |= R_RIIC_MasterTransmit(CHANNEL_0, &target_data, 1);   
    }
    
    return ret;                  
} /* End of function thermal_sensor_init()  */


/*******************************************************************************
* Function name: thermal_sensor_read
* Description  : This function fetches the thermal sensor temperature data and 
*                displays it on the LCD. Called by the CMT interrupt callback 
*                function, this function is executed after every period of the 
*                CMT timer.
* Argument     : none
* Return value : int16_t -
*                   signed temperature value in nearest 10ths of °C.
*******************************************************************************/
int16_t thermal_sensor_read(void)
{
    uint8_t     target_data[2]; /* Room for 2 bytes of data read from thermal sensor */
    uint8_t     addr_and_register[2]; /* Storage for the slave address and target register. */
    riic_ret_t  ret = RIIC_OK;

    uint16_t	raw_temp; /* signed */
	int16_t		signed_temp;
    
    /* To read from a specific register in the thermal sensor, first transmit the 
       its I2C slave address together with the register number. */         
    /* The 7-bit I2C address of the ADT7420 and the R/W bit. */
    addr_and_register[0] = ADT7420_ADDR;     
    /* The register address of the first byte of temperature data. */ 
    addr_and_register[1] = ADT7420_TEMP_MSB_REG;   
    
    /* Send the I2C slave address and register number (2 bytes). */
    ret |= R_RIIC_MasterTransmitHead(CHANNEL_0, addr_and_register, 2);
    
    /* Now read 2 bytes of data from the target register into the destination buffer. */    
    ret |= R_RIIC_MasterReceive(CHANNEL_0, ADT7420_ADDR, target_data, 2);  

	/* 	
	After right-shift 3 to drop lowest 3 bits,
	Sign bit + 12 bits gives a temperature resolution of 0.0625°C.  	
	Positive Temperature = ADC Code (dec)/16
	Negative Temperature = (ADC Code (dec) - 8192)/16
	where Bit 13 (sign bit) is included.
	*/

    /* Convert the device measurement into a signed decimal value. */
    raw_temp = ((uint16_t)(target_data[0]<<8)) + target_data[1];
    raw_temp = raw_temp >> 3;				/* drop 3 lowest bits of status data */       

    signed_temp = raw_temp;
	
	if (signed_temp & 0x1000) /* 13th bit is 1, so temperature is negative */
	{
		/* all this is round about way of dealing with odd 2-s complement value */
		signed_temp -= 8192;				/* becomes negative int value */
		signed_temp *= -1; 					/* work on it as positive for now */
		signed_temp *= 100;					/* value as 100ths */
		signed_temp = (uint16_t)signed_temp/16;		/* convert from 0.0625 units to ones */
		signed_temp = (signed_temp + 5)/10;	/* convert to tenths rounded to nearest */
		signed_temp *= -1; 					/* convert back to negative */	
	}
	else /* positive temperature */
	{
		signed_temp *= 100;						/* value as 100ths */
		signed_temp = (uint16_t)signed_temp/16;	/* convert from 0.0625 units to ones */
		signed_temp = (signed_temp + 5)/10;	/* convert to tenths rounded to nearest */
	}
    
	return signed_temp;  
} /* End of function thermal_sensor_read()  */

/*******************************************************************************
* Function name: temperature_display()
* Description  : Gets thermal sensor temperature data, rounds to nearest tenth,
*              : then converts it to a string and displays it on LCD.
* Argument     : none
* Return value : none
*******************************************************************************/
void temperature_display(void)
{
	int temperature;
	uint8_t	i;
    
	/* The output display string buffer. */
 	uint8_t lcd_buffer[13] = {0}; /* Space for 12 characters + null. */

    /* Read the temperature */
	temperature = thermal_sensor_read();
	//printf("temp= %d", temperature);
 	/*if(temperature>280)
	{
		//LED11 = 0;
		LED9 = 1;
	}
	else
	{
		LED9 = 0;
	}*/
    /* Convert temperature value into a string. */
   // sprintf((char *)lcd_buffer, "TEMP:%+d", temperature);

	/* Temperature value is in 10ths C. */
    /* Fit the decimal point character in before last character */
    //i = strlen((char *)lcd_buffer);
    //lcd_buffer[i] = lcd_buffer[i-1];    /* Move last character over by 1. */ 
	//lcd_buffer[i-1] = '.';	 
	//lcd_buffer[i+1] = 'C';              /* Celsius char. */ 

    /* Display the contents of lcd_buffer onto the debug LCD */
   // lcd_display(LCD_LINE8, lcd_buffer);    
    		      
}


void static cmt_callback(void)
{
    accelerometer_demo_update();
    if (g_thermal_sensor_good) /* Only run thermal sensor demo if it is present. */
    {
        temperature_display();
    }
}








