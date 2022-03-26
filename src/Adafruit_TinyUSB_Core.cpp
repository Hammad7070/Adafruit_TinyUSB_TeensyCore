/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019, hathach for Adafruit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Adafruit_TinyUSB_TeensyCore.h"

#ifdef USE_TINYUSB
#ifdef USE_TINYWEBUSB

#include "Arduino.h"
//#include <Reset.h> // Needed for auto-reset with 1200bps port touch

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
extern "C"
{
#if defined(__SAMD51__)

void USB_0_Handler (void) { tud_int_handler(0); }
void USB_1_Handler (void) { tud_int_handler(0); }
void USB_2_Handler (void) { tud_int_handler(0); }
void USB_3_Handler (void) { tud_int_handler(0); }

#else

void USB_Handler(void) { tud_int_handler(0); }

#endif
} // extern C



//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
static void usb_hardware_init(void);

#if CFG_TUSB_DEBUG
extern "C" int serial1_printf(const char *__restrict format, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, format);
  vsnprintf(buf, sizeof(buf), format, ap);
//  if(USBSerial.availableForWrite()) USBSerial.write(buf);
  va_end(ap);
  return 1;
}
#endif

//--------------------------------------------------------------------+
// Core Init & Touch1200
//--------------------------------------------------------------------+
void Adafruit_TinyUSB_Core_init(void)
{
#if CFG_TUSB_DEBUG
  Serial1.begin(115200);
  serial1_printf("TinyUSB debugging with Serial1\n");
#endif

  USBSerial.setStringDescriptor("TinyUSB Serial");
  USBDevice.addInterface(USBSerial);
  USBDevice.setID(USB_VID, USB_PID);
  USBDevice.begin();

  usb_hardware_init();

  // Init tinyusb stack
  tusb_init();
}

void Adafruit_TinyUSB_Core_touch1200(void)
{
//  initiateReset(250);
}

//--------------------------------------------------------------------+
// Adafruit_USBD_Device platform dependent
//--------------------------------------------------------------------+

uint8_t Adafruit_USBD_Device::getSerialDescriptor(uint16_t* serial_str)
{
  char buf[11];
  uint32_t i, num;

  num = HW_OCOTP_MAC0 & 0xFFFFFF;
  // add extra zero to work around OS-X CDC-ACM driver bug
  if (num < 10000000) num = num * 10;
  ultoa(num, buf, 10);

  for (i=0; i<10; i++) {
    char c = buf[i];
    if (!c) break;
    serial_str[i] = c;
  }

  return i * 2 + 2;
}

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

// Init usb hardware when starting up. Softdevice is not enabled yet
static void usb_hardware_init(void)
{
  PMU_REG_3P0 = PMU_REG_3P0_OUTPUT_TRG(0x0F) | PMU_REG_3P0_BO_OFFSET(6)
    | PMU_REG_3P0_ENABLE_LINREG;
  
  CCM_CCGR6 |= CCM_CCGR6_USBOH3(CCM_CCGR_ON); // turn on clocks to USB peripheral
#if 1
  if ((USBPHY1_PWD & (USBPHY_PWD_RXPWDRX | USBPHY_PWD_RXPWDDIFF | USBPHY_PWD_RXPWD1PT1
    | USBPHY_PWD_RXPWDENV | USBPHY_PWD_TXPWDV2I | USBPHY_PWD_TXPWDIBIAS
    | USBPHY_PWD_TXPWDFS)) || (USB1_USBMODE & USB_USBMODE_CM_MASK)) {
    // USB controller is turned on from previous use
    // reset needed to turn it off & start from clean slate
    USBPHY1_CTRL_SET = USBPHY_CTRL_SFTRST; // USBPHY1_CTRL page 3292
    NVIC_CLEAR_PENDING(IRQ_USB1);
    USBPHY1_CTRL_CLR = USBPHY_CTRL_SFTRST; // reset PHY
    //USB1_USBSTS = USB1_USBSTS; // TODO: is this needed?
    delay(25);
  }
#endif

  USBPHY1_CTRL_CLR = USBPHY_CTRL_CLKGATE;
  USBPHY1_PWD = 0;
  
  attachInterruptVector(IRQ_USB1, &USB_Handler);
 NVIC_SET_PRIORITY(IRQ_USB1, 0);
//	/* Enable USB clock */
//#if defined(__SAMD51__)
//	MCLK->APBBMASK.reg |= MCLK_APBBMASK_USB;
//	MCLK->AHBMASK.reg |= MCLK_AHBMASK_USB;
//
//	// Set up the USB DP/DN pins
//	PORT->Group[0].PINCFG[PIN_PA24H_USB_DM].bit.PMUXEN = 1;
//	PORT->Group[0].PMUX[PIN_PA24H_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24H_USB_DM & 0x01u)));
//	PORT->Group[0].PMUX[PIN_PA24H_USB_DM/2].reg |= MUX_PA24H_USB_DM << (4 * (PIN_PA24H_USB_DM & 0x01u));
//	PORT->Group[0].PINCFG[PIN_PA25H_USB_DP].bit.PMUXEN = 1;
//	PORT->Group[0].PMUX[PIN_PA25H_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25H_USB_DP & 0x01u)));
//	PORT->Group[0].PMUX[PIN_PA25H_USB_DP/2].reg |= MUX_PA25H_USB_DP << (4 * (PIN_PA25H_USB_DP & 0x01u));
//
//
//	GCLK->PCHCTRL[USB_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
//
//	NVIC_SetPriority(USB_0_IRQn, 0UL);
//	NVIC_SetPriority(USB_1_IRQn, 0UL);
//	NVIC_SetPriority(USB_2_IRQn, 0UL);
//	NVIC_SetPriority(USB_3_IRQn, 0UL);
//#else
//	PM->APBBMASK.reg |= PM_APBBMASK_USB;
//
//	// Set up the USB DP/DN pins
//	PORT->Group[0].PINCFG[PIN_PA24G_USB_DM].bit.PMUXEN = 1;
//	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg &= ~(0xF << (4 * (PIN_PA24G_USB_DM & 0x01u)));
//	PORT->Group[0].PMUX[PIN_PA24G_USB_DM/2].reg |= MUX_PA24G_USB_DM << (4 * (PIN_PA24G_USB_DM & 0x01u));
//	PORT->Group[0].PINCFG[PIN_PA25G_USB_DP].bit.PMUXEN = 1;
//	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg &= ~(0xF << (4 * (PIN_PA25G_USB_DP & 0x01u)));
//	PORT->Group[0].PMUX[PIN_PA25G_USB_DP/2].reg |= MUX_PA25G_USB_DP << (4 * (PIN_PA25G_USB_DP & 0x01u));
//
//	// Put Generic Clock Generator 0 as source for Generic Clock Multiplexer 6 (USB reference)
//	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(6)     | // Generic Clock Multiplexer 6
//	GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
//	GCLK_CLKCTRL_CLKEN;
//	while (GCLK->STATUS.bit.SYNCBUSY)
//	;
//
//	NVIC_SetPriority((IRQn_Type) USB_IRQn, 0UL);
//#endif
}

#endif // USE_TINYUSB
