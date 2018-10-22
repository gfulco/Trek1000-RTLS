/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"

#include "deca_types.h"

#include "deca_spi.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

uint32 inittestapplication(uint8 s1switch);

#define SWS1_SHF_MODE 0x02	//short frame mode (6.81M)
#define SWS1_CH5_MODE 0x04	//channel 5 mode
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_A1A_MODE 0x10  //anchor/tag address A1
#define SWS1_A2A_MODE 0x20  //anchor/tag address A2
#define SWS1_A3A_MODE 0x40  //anchor/tag address A3
#define SWS1_USB2SPI_MODE 0x78  //USB to SPI mode
#define SWS1_TXSPECT_MODE 0x38  //Continuous TX spectrum mode
#define SWS1_RESERVED_MODE1 0x18 //Reserved mode - not used
#define SWS1_RESERVED_MODE2 0x58 //Reserved mode - not used


                             //"1234567812345678"
#define SOFTWARE_VER_STRING    "Ver.2.25mx TREK" //16 bytes!

uint8 s1switch = 0;
int instance_anchaddr = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;
int instance_mode = ANCHOR;

#define LCD_BUFF_LEN (80)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];
uint32_t pauseTWRReports  = 0;
uint32_t printLCDTWRReports  = 0;
uint8_t sendTWRRawReports = 1;

//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
instanceConfig_t chConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
                    {
                        .channelNumber = 2,             // channel
                        .preambleCode = 4,              // preambleCode
                        .pulseRepFreq = DWT_PRF_16M,    // prf
                        .dataRate = DWT_BR_110K,        // datarate
                        .preambleLen = DWT_PLEN_1024,   // preambleLength
                        .pacSize = DWT_PAC32,           // pacSize
                        .nsSFD = 1,                     // non-standard SFD
                        .sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 2 - S1: 2 on, 3 off
                    {
						.channelNumber = 2,            // channel
						.preambleCode = 4,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    },
                    //mode 3 - S1: 2 off, 3 on
                    {
						.channelNumber = 5,             // channel
						.preambleCode = 3,              // preambleCode
						.pulseRepFreq = DWT_PRF_16M,    // prf
						.dataRate = DWT_BR_110K,        // datarate
						.preambleLen = DWT_PLEN_1024,   // preambleLength
						.pacSize = DWT_PAC32,           // pacSize
						.nsSFD = 1,                     // non-standard SFD
						.sfdTO = (1025 + 64 - 32)       // SFD timeout
                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						.channelNumber = 5,            // channel
						.preambleCode = 3,             // preambleCode
						.pulseRepFreq = DWT_PRF_16M,   // prf
						.dataRate = DWT_BR_6M8,        // datarate
						.preambleLen = DWT_PLEN_128,   // preambleLength
						.pacSize = DWT_PAC8,           // pacSize
						.nsSFD = 0,                    // non-standard SFD
						.sfdTO = (129 + 8 - 8)        // SFD timeout
                    }
};

//Slot and Superframe Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
sfConfig_t sfConfig[4] ={
                    //mode 1 - S1: 2 off, 3 off
					{
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
					},
#if (DISCOVERY == 1)
                    //mode 2 - S1: 2 on, 3 off
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (100),        //number of slots in the superframe (98 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*100),  //in ms => 1000 ms frame means 1 Hz location rate
						.tagPeriod_ms = (10*100), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
#else
                    //mode 2 - S1: 2 on, 3 off
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
#endif
                    //mode 3 - S1: 2 off, 3 on
                    {
						.slotDuration_ms = (28), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*28),  //in ms => 280ms frame means 3.57 Hz location rate
						.tagPeriod_ms = (10*28), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (20000) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)

                    },
                    //mode 4 - S1: 2 on, 3 on
                    {
						.slotDuration_ms = (10), //slot duration in milliseconds (NOTE: the ranging exchange must be able to complete in this time
												 //e.g. tag sends a poll, 4 anchors send responses and tag sends the final + processing time
						.numSlots = (10),        //number of slots in the superframe (8 tag slots and 2 used for anchor to anchor ranging),
						.sfPeriod_ms = (10*10),  //in ms => 100 ms frame means 10 Hz location rate
						.tagPeriod_ms = (10*10), //tag period in ms (sleep time + ranging time)
						.pollTxToFinalTxDly_us = (2500) //poll to final delay in microseconds (needs to be adjusted according to lengths of ranging frames)
                    }
};
// ======================================================
//
//  Configure instance tag/anchor/etc... addresses
//
void addressconfigure(uint8 s1switch, uint8 mode)
{
    uint16 instAddress ;

    instance_anchaddr = (((s1switch & SWS1_A1A_MODE) << 2) + (s1switch & SWS1_A2A_MODE) + ((s1switch & SWS1_A3A_MODE) >> 2)) >> 4;

    if(mode == ANCHOR)
    {
    	if(instance_anchaddr > 3)
		{
			instAddress = GATEWAY_ANCHOR_ADDR | 0x4 ; //listener
		}
		else
		{
			instAddress = GATEWAY_ANCHOR_ADDR | instance_anchaddr;
		}
	}
    else
    {
    	instAddress = instance_anchaddr;
    }

    instance_set_16bit_address(instAddress);
}


//returns the use case / operational mode
int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 2;
    }

    return mode;
}

uint32 inittestapplication(uint8 s1switch)
{
    uint32 devID ;
    int result;

    port_set_dw1000_slowrate();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instance_readdeviceid() ;
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
    	port_wakeup_dw1000();

        devID = instance_readdeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();

    if((s1switch & SWS1_ANC_MODE) == 0)
    {
        instance_mode = TAG;
    }
    else
    {
        instance_mode = ANCHOR;
    }

    result = instance_init(instance_mode) ; // Set this instance mode (tag/anchor)
    if (0 > result) return(-1) ; // Some failure has occurred

    port_set_dw1000_fastrate();
    devID = instance_readdeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    addressconfigure(s1switch, instance_mode) ; // set up default 16-bit address

    if((instance_mode == ANCHOR) && (instance_anchaddr > 0x3))
    {
    	//invalid configuration
    	//display "Reserved" on the LCD
    }
    else
    {
		// get mode selection (index) this has 4 values see chConfig struct initialiser for details.
		dr_mode = decarangingmode(s1switch);

		chan = chConfig[dr_mode].channelNumber ;

		instance_config(&chConfig[dr_mode], &sfConfig[dr_mode]) ;                  // Set operating channel etc
    }
    return devID;
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

void setLCDline1(uint8 s1switch)
{
	int role = instance_get_role();

	sprintf((char*)&dataseq[0], "DecaRangeRTLS %s%d", (s1switch & SWS1_SHF_MODE) ? "S" : "L", chan);
	writetoLCD( 40, 1, dataseq); //send some data

	tagaddr = instance_anchaddr;
	ancaddr = instance_anchaddr;

	if(role == TAG)
	{
		sprintf((char*)&dataseq1[0], "Tag:%d    ", tagaddr);
		writetoLCD( 16, 1, dataseq1); //send some data

	}
	else if((role == ANCHOR) && (ancaddr < 4))
	{
		sprintf((char*)&dataseq1[0], "Anchor:%d ", ancaddr);
		writetoLCD( 16, 1, dataseq1); //send some data
	}
	else
	{
		sprintf((char*)&dataseq1[0], "Reserved ");
		writetoLCD( 16, 1, dataseq1); //send some data
	}
}

void configure_continuous_txspectrum_mode(uint8 s1switch)
{
    uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
	sprintf((char*)&dataseq[0], "Continuous TX %s%d", (s1switch & SWS1_SHF_MODE) ? "S" : "L", chan);
	writetoLCD( 40, 1, dataseq); //send some data
	memcpy(dataseq, (const uint8 *) "Spectrum Test   ", 16);
	writetoLCD( 16, 1, dataseq); //send some data

	//configure DW1000 into Continuous TX mode
	instance_starttxtest(0x1000);
	//measure the power
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	//user has to reset the board to exit mode
	while(1)
	{
		Sleep(2);
	}

}


/*
 * @fn      main()
 * @brief   main entry point
**/

int dw_main(void)
{
    int i = 0;
    int rx = 0;
    int toggle = 0;
    uint8 command = 0x0;
    uint8 usbVCOMout[LCD_BUFF_LEN*4];

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 0x0, sizeof(dataseq));
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    writetoLCD( 40, 1, dataseq);
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16);
    writetoLCD( 16, 1, dataseq);

    Sleep(1000);

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
#endif

    s1switch = port_is_boot1_low() << 1
    		| port_is_switch_on(TA_SW1_3) << 2
    		| port_is_switch_on(TA_SW1_4) << 3
    		| port_is_switch_on(TA_SW1_5) << 4
		    | port_is_switch_on(TA_SW1_6) << 5
    		| port_is_switch_on(TA_SW1_7) << 6
    		| port_is_switch_on(TA_SW1_8) << 7;


    if((s1switch & SWS1_USB2SPI_MODE) == SWS1_USB2SPI_MODE)
    {
        int j = 1000000;

        memset(dataseq, 0, LCD_BUFF_LEN);

        while(j--);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memcpy(dataseq, (const uint8 *) "DECAWAVE   ", 12);
        writetoLCD( 40, 1, dataseq); //send some data
#ifdef USB_SUPPORT //this is set in the port.h file
        memcpy(dataseq, (const uint8 *) "USB-to-SPI ", 12);
#else
#endif
        writetoLCD( 16, 1, dataseq); //send some data

        j = 1000000;

        while(j--);

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
#ifdef USB_SUPPORT //this is set in the port.h file
        // enable the USB functionality
        //usb_init();

        // Do nothing in foreground -- allow USB application to run, I guess on the basis of USB interrupts?
        while (1)       // loop forever
        {
            usb_run();
        }
#endif
        return 1;
    }
    else //run DecaRangeRTLS application for TREK
    {

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', LCD_BUFF_LEN);
#if(DISCOVERY==1)
        memcpy(dataseq, (const uint8 *) "DISCOVERY  TREK ", 16);
#else
        memcpy(dataseq, (const uint8 *) "DECAWAVE   TREK ", 16);
#endif
        writetoLCD( 16, 1, dataseq); //send some data

        led_off(LED_ALL);

        if(inittestapplication(s1switch) == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            return 0; //error
        }

#ifdef USB_SUPPORT //this is defined in the port.h file
        // Configure USB for output, (i.e. not USB to SPI)
        usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING, s1switch);
#endif
        // Is continuous spectrum test mode selected?
        if((s1switch & SWS1_TXSPECT_MODE) == SWS1_TXSPECT_MODE)
    	{
        	//this function does not return!
        	configure_continuous_txspectrum_mode(s1switch);
    	}

        //sleep for 5 seconds displaying last LCD message and flashing LEDs
        i=30;
        while(i--)
        {
            if (i & 1) led_off(LED_ALL);
            else    led_on(LED_ALL);

            Sleep(200);
        }
        i = 0;
        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', LCD_BUFF_LEN);
        memset(dataseq1, ' ', LCD_BUFF_LEN);

        setLCDline1(s1switch);

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
    }

    // Is reserved mode selected?
    if(((s1switch & SWS1_RESERVED_MODE1) == SWS1_RESERVED_MODE1)
    	||
    	((s1switch & SWS1_RESERVED_MODE2) == SWS1_RESERVED_MODE2)
    	)
	{
    	//this function does not return!
    	return 1;
	}


    port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    //memset(dataseq, ' ', LCD_BUFF_LEN);
    memset(dataseq1, ' ', LCD_BUFF_LEN);
#if (LCD_UPDATE_ON == 0)
    sprintf((char*)&dataseq1[0], "A%d LCD Updating", ancaddr);
    writetoLCD( 40, 1, dataseq1); //send some data
    sprintf((char*)&dataseq1[0], "   disabled.    ");
    writetoLCD( 16, 1, dataseq1); //send some data
#endif

    // main loop
    while(1)
    {
    	int n = 0;
    	instance_data_t* inst = instance_get_local_structure_ptr(0);

    	int monitor_local = inst->monitor ;
    	int txdiff = (portGetTickCnt() - inst->timeofTx);

        instance_mode = instance_get_role();

        if(instance_mode == TAG)
    	{
    		tag_run();
    	}
    	else
    	{
    		anch_run();
    	}

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((monitor_local == 1) && ( txdiff > inst->slotDuration_ms))
        {
        	int an = 0;
        	uint32 tdly ;
        	uint32 reg1, reg2;

        	reg1 = dwt_read32bitoffsetreg(0x0f, 0x1);
        	reg2 = dwt_read32bitoffsetreg(0x019, 0x1);
        	tdly = dwt_read32bitoffsetreg(0x0a, 0x1);
        	an = sprintf((char*)&usbVCOMout[0], "T%08x %08x time %08x %08x", (unsigned int) reg2, (unsigned int) reg1,
        			(unsigned int) dwt_read32bitoffsetreg(0x06, 0x1), (unsigned int) tdly);

            if(usb_ready())
            {
            	send_usbmessage(&usbVCOMout[0], an);
            }

            inst->wait4ack = 0;

        	if(instance_mode == TAG)
			{
        		tag_process_rx_timeout(inst);
			}
			else //if(instance_mode == ANCHOR)
			{
				dwt_forcetrxoff();	//this will clear all events
				//dwt_rxreset();
				//enable the RX
				inst->testAppState = TA_RXE_WAIT ;
			}
        	inst->monitor = 0;
        }

        rx = instance_newrange();

        //if there is a new ranging report received or a new range has been calculated, then prepare data
        //to output over USB - Virtual COM port, and update the LCD
        if(rx != TOF_REPORT_NUL)
        {
        	int l = 0, r= 0, aaddr, taddr;
        	int rangeTime, valid;
        	//int correction ;
            uint16 txa, rxa;

            //send the new range information to LCD and/or USB
            aaddr = instance_newrangeancadd() & 0xf;
#if(DISCOVERY == 1)
            taddr = instance_newrangetagadd() & 0xff;
#else
            taddr = instance_newrangetagadd() & 0xf;
#endif
            rangeTime = instance_newrangetim() & 0xffffffff;
#if (LCD_UPDATE_ON == 1)
            //if((dr_mode & 0x1) == 0) //only print for 110k
            if(printLCDTWRReports + 2000 <= portGetTickCnt())
            {
				//led_on(LED_PC9);
				command = 0x2 ;  //return cursor home
				writetoLCD( 1, 0,  &command);

				memset(dataseq1, ' ', LCD_BUFF_LEN);
				writetoLCD( 40, 1, dataseq); //send some data

				//anchors will print a range to each tag in sequence with 1 second pause
				//they will show the last rage to that tag
				if(instance_mode == ANCHOR)
				{
					int b = 0;
					double rangetotag = instance_get_tagdist(toggle) ;

					while(((int) (rangetotag*1000)) == 0) //if 0 then go to next tag
					{
						if(b > (MAX_TAG_LIST_SIZE-1))
							break;

						toggle++;
						if(toggle >= MAX_TAG_LIST_SIZE)
							toggle = 0;

						rangetotag = instance_get_tagdist(toggle) ;
						b++;
					}

					sprintf((char*)&dataseq1[0], "A%d T%d: %3.2f m", ancaddr, toggle, rangetotag);
					writetoLCD( 16, 1, dataseq1); //send some data

					sprintf((char*)&dataseq1[0], "01:%d 12:%d 02:%d 03:%d", anctoancrange[0], anctoancrange[1], anctoancrange[2],anctoancrange[3]);
										writetoLCD( 16, 1, dataseq1);

					toggle++;

					if(toggle >= MAX_TAG_LIST_SIZE)
						toggle = 0;
				}
				else if(instance_mode == TAG)
				{
#if(DISCOVERY == 1)
					sprintf((char*)&dataseq1[0], "T%d A%d: %3.2f m", taddr, toggle, instance_get_idist(toggle));
#else
					sprintf((char*)&dataseq1[0], "T%d A%d: %3.2f m", tagaddr, toggle, instance_get_idist(toggle));
#endif
					//toggle = 1;
					writetoLCD( 16, 1, dataseq1); //send some data

					toggle++;

					if(toggle >= MAX_ANCHOR_LIST_SIZE)
						toggle = 0;
				}
				//led_off(LED_PC9);

				//update the print time
				printLCDTWRReports = portGetTickCnt();
            }
#endif
#ifdef USB_SUPPORT //this is set in the port.h file
            //led_on(LED_PC9);
            l = instance_get_lcount() & 0xFFFF;
            if(instance_mode == TAG)
            {
            	r = instance_get_rnum();
            }
            else
            {
            	r = instance_get_rnuma(taddr);
            }
            txa =  instance_get_txantdly();
            rxa =  instance_get_rxantdly();
            valid = instance_validranges();

            n = 0;
            if(rx == TOF_REPORT_T2A)
            {
            	// anchorID tagID range rangeraw countofranges rangenum rangetime txantdly rxantdly address
            	n = sprintf((char*)&usbVCOMout[0], "mc %02x %08x %08x %08x %08x %04x %02x %08x %c%d:%d\r\n",
														valid, instance_get_idist_mm(0), instance_get_idist_mm(1),
														instance_get_idist_mm(2), instance_get_idist_mm(3),
														l, r, rangeTime,
														(instance_mode == TAG)?'t':'a', taddr, aaddr);
            	if(sendTWRRawReports == 1)
            	{
            		n += sprintf((char*)&usbVCOMout[n], "mr %02x %08x %08x %08x %08x %04x %02x %04x%04x %c%d:%d\r\n",
														valid, instance_get_idistraw_mm(0), instance_get_idistraw_mm(1),
														instance_get_idistraw_mm(2), instance_get_idistraw_mm(3),
														l, r, txa, rxa,
														(instance_mode == TAG)?'t':'a', taddr, aaddr);
            	}
            }
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
            else //anchor to anchor ranging (only output by A0)
            {
            	n = sprintf((char*)&usbVCOMout[0], "ma %02x %08x %08x %08x %08x %04x %02x %08x a0:0\r\n",
														valid, instance_get_idist_mm(0), instance_get_idist_mm(1),
														instance_get_idist_mm(2), instance_get_idist_mm(3),
														l, instance_get_rnumanc(0), rangeTime);
            	for(int i=0;i<4;i++)
            		anctoancranges[i]=instance_get_idist_mm(i);
            }
#endif
            //led_off(LED_PC9);
            instance_cleardisttableall();
#if (READ_EVENT_COUNTERS == 1)
            {
            	n += sprintf((char*)&usbVCOMout[n], "me %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\r\n",
            			inst->ecounters.ARFE, inst->ecounters.CRCB,
            			inst->ecounters.CRCG, inst->ecounters.HPW,
            			inst->ecounters.RSL, inst->ecounters.PHE,
            			inst->ecounters.RTO, inst->ecounters.SFDTO,
            			inst->ecounters.TXF, inst->ecounters.TXW);
            }
#endif
#endif
        } //if new range present

#ifdef USB_SUPPORT //this is set in the port.h file
       	if(n > 0)
       	{
       		if(pauseTWRReports == 0)
       		{
       		    if(usb_ready())
       		    {
       		    	led_off(LED_PC8);
       		    	send_usbmessage(&usbVCOMout[0], n - 2); //so we don't add another new line
       		    	usb_run();
       		    }
       		    else
       		    {
       		    	led_on(LED_PC8);
       		    }
       		}
       		else
       		{
       			if(pauseTWRReports + 1000 <= portGetTickCnt())
       			{
       				pauseTWRReports = 0;
       			}
       		}
       	}

        //led_on(LED_PC7);
        usb_run();
        //led_off(LED_PC7);
#endif
    }


    return 0;
}



