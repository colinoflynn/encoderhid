/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Keyboard demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "EncoderBoard.h"


void Jump_To_Bootloader(void);

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
 	{
		.Config =
			{
				.InterfaceNumber              = 0,
				.ReportINEndpoint             =
					{
						.Address              = KEYBOARD_EPADDR,
						.Size                 = KEYBOARD_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
			},
    };


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		HID_Device_USBTask(&Keyboard_HID_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware()
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
    DDRD &= ~(1<<0 | 1<<1 | 1<<2 | 1<<3);
    PORTD |= (1<<0 | 1<<1 | 1<<2 | 1<<3);
    DDRA = 0;
    PORTA = 0xff;
    DDRC = 0;
    PORTC = 0xff;
    
    DDRB = 0x00;
    PORTB = 0xff;
    
    DDRE = 0;
    PORTE = 0xff;
       
	//Joystick_Init();
	LEDs_Init();
	//Buttons_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

#define NUM_ENCODERS        12

#define ENCODER_TOPLEFT    ((PINA & (1<<0 | 1<<1)) >> 0)
#define ENCODER_TOPRIGHT   ((PINA & (1<<2 | 1<<3)) >> 2)
#define ENCODER_RIGHTMID   ((PINA & (1<<4 | 1<<5)) >> 4)
#define ENCODER_RIGHTBOT  ((PINA & (1<<6 | 1<<7)) >> 6)

#define ECNDOER_CHA_BOT     ((PINB & (1<<0 | 1<<1)) >> 0)
#define ECNDOER_CHB_BOT     ((PINB & (1<<2 | 1<<3)) >> 2)
#define ECNDOER_CHC_BOT     ((PINB & (1<<4 | 1<<5)) >> 4)
#define ECNDOER_CHD_BOT     ((PINB & (1<<6 | 1<<7)) >> 6)

#define ECNDOER_CHA_TOP     ((PINC & (1<<0 | 1<<1)) >> 0)
#define ECNDOER_CHB_TOP     ((PINC & (1<<2 | 1<<3)) >> 2)
#define ECNDOER_CHC_TOP     ((PINC & (1<<4 | 1<<5)) >> 4)
#define ECNDOER_CHD_TOP     ((PINC & (1<<6 | 1<<7)) >> 6)

uint8_t encoder_lookup(uint8_t i)
{
    switch(i){
        case 0: return ENCODER_TOPLEFT;
        case 1: return ENCODER_TOPRIGHT;
        case 2: return ENCODER_RIGHTMID;
        case 3: return ENCODER_RIGHTBOT;
        case 4: return ECNDOER_CHA_BOT;
        case 5: return ECNDOER_CHB_BOT;
        case 6: return ECNDOER_CHC_BOT;
        case 7: return ECNDOER_CHD_BOT;
        case 8: return ECNDOER_CHA_TOP;
        case 9: return ECNDOER_CHB_TOP;
        case 10: return ECNDOER_CHC_TOP;
        case 11: return ECNDOER_CHD_TOP;
        default: return 0;
    }
}

#define check_bootloader() ((PINE & 0x0F) == 0x06)

uint8_t button_lookup(uint8_t i)
{
    switch(i){
        case 0: 
        case 1: 
        case 2: 
        case 3: 
        case 4: 
        case 5:
        case 6: 
        case 7: return (PINE & (1<<i));
        case 8: return (PIND & (1<<0));
        case 9: return (PIND & (1<<1));
        case 10: return (PIND & (1<<2));
        case 11: return (PIND & (1<<3));
        default: return 0;
    }
}


/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;

    static uint8_t laststatus[NUM_ENCODERS];
    static uint8_t lastbuttonstatus[NUM_ENCODERS];
    
    const uint8_t pos_lookup[] = {0x02, 0x00, 0x03, 0x01};
    const uint8_t neg_lookup[] = {0x01, 0x03, 0x00, 0x02};
    uint8_t rptsize = 0;
       
    if (check_bootloader()){
        Jump_To_Bootloader();
    }
       
    for (uint8_t i = 0; i < NUM_ENCODERS; i++){
        if (laststatus[0] != encoder_lookup(i)){
           
            if (pos_lookup[laststatus[i]] == encoder_lookup(i)) {
                KeyboardReport->KeyCode[rptsize++] = HID_KEYBOARD_SC_A+i;
                
                //KeyboardReport->Modifier = HID_KEYBOARD_SC_LEFT_CONTROL;
            } else if (neg_lookup[laststatus[i]] == encoder_lookup(i)) {
                KeyboardReport->KeyCode[rptsize++] = HID_KEYBOARD_SC_A+i+NUM_ENCODERS;
            }
            laststatus[i] = encoder_lookup(i);   
        }
    }
    
    for (uint8_t i = 0; i < NUM_ENCODERS; i++){
        if (lastbuttonstatus[0] != button_lookup(i)){
           
            if (button_lookup(i) == 0) {
                KeyboardReport->KeyCode[rptsize++] = HID_KEYBOARD_SC_KEYPAD_SLASH+i;
            }
            lastbuttonstatus[i] = button_lookup(i);   
        }
    }
    
	*ReportSize = sizeof(USB_KeyboardReport_Data_t);
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
 /*

	uint8_t  LEDMask   = LEDS_NO_LEDS;
	uint8_t* LEDReport = (uint8_t*)ReportData;
	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	  LEDMask |= LEDS_LED1;

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	  LEDMask |= LEDS_LED3;

	if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
	  LEDMask |= LEDS_LED4;

	LEDs_SetAllLEDs(LEDMask);
    
    */
    ;
}

  uint32_t Boot_Key ATTR_NO_INIT;

  #define MAGIC_BOOT_KEY            0xDC42ACCA
  //Defined in makefile
  //#define BOOTLOADER_START_ADDRESS  (FLASH_SIZE_BYTES - BOOTLOADER_SEC_SIZE_BYTES)

  void Bootloader_Jump_Check(void) ATTR_INIT_SECTION(3);
  void Bootloader_Jump_Check(void)
  {
      // If the reset source was the bootloader and the key is correct, clear it and jump to the bootloader
      if ((MCUSR & (1 << WDRF)) && (Boot_Key == MAGIC_BOOT_KEY))
      {
          Boot_Key = 0;
          ((void (*)(void))BOOTLOADER_START_ADDRESS)();
      }
  }

  void Jump_To_Bootloader(void)
  {
      // If USB is used, detach from the bus and reset it
      USB_Disable();

      // Disable all interrupts
      cli();

      // Wait two seconds for the USB detachment to register on the host
      Delay_MS(2000);

      // Set the bootloader key to the magic value and force a reset
      Boot_Key = MAGIC_BOOT_KEY;
      wdt_enable(WDTO_250MS);
      for (;;);
  }

