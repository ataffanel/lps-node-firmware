/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stm32f0xx_hal.h>
#include <string.h>
#include <stdio.h>

#include "system.h"
#include "tim.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "led.h"

#include "cfg.h"
#include "eeprom.h"

#include "usb_device.h"
#include "usbcomm.h"

#include "dwOps.h"

#include "mac.h"

#include "lps25h.h"
#include "test_support.h"
#include "production_test.h"
#include "statemachine.h"

static CfgMode mode = modeAnchor;

const uint8_t *uid = (uint8_t*)MCU_ID_ADDRESS;

int initDwm1000();

#define N_NODES 9

uint8_t address[8] = {0,0,0,0,0,0,0xcf,0xbc};
uint8_t base_address[8] = {0,0,0,0,0,0,0xcf,0xbc};

float positions[][3] = {
  {0,0,0},
  {4,0,0},
  {0,5,0},
  {0.1f,6.98f,2.60f},
  {5.30f, 1.70f, 2.20f},
  {0.2f,-3.96f,2.60f},
  {4.00,  5.00,  0.00},
};

float position[3];

// Static system configuration
#define MAX_ANCHORS 6
uint8_t anchors[MAX_ANCHORS];

const double C = 299792458.0;       // Speed of light
const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency

#define ANTENNA_OFFSET 154.6   // In meter
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

static void restConfig();
static void changeAddress(uint8_t addr);
static void handleInput(char ch);
static void changeMode(CfgMode newMode);
static void printMode();
static void help();

static bool receivedFlag = false;
static bool receiveFailedFlag = false;
static bool transmitedFlag = false;

// #define printf(...)
#define debug(...) // printf(__VA_ARGS__)

void txcallback(dwDevice_t *dev)
{
  transmitedFlag = true;
}

void rxcallback(dwDevice_t *dev) {
  receivedFlag = true;
}

void rxFailedCallback(dwDevice_t *dev) {
  receiveFailedFlag = true;
}

bool contains(int* list, int length, int value)
{
  int i;

  for (i=0; i<length; i++) {
    if (list[i] == value) {
      return true;
    }
  }

  return false;
}

dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;

int main() {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  int result;
  int i;
  char ch;
  bool selftestPasses = true;

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_TIM2_Init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  // Light up all LEDs to test
  ledOn(ledRanging);
  ledOn(ledSync);
  ledOn(ledMode);

  printf("\r\n\r\n====================\r\n");

  printf("SYSTEM\t: CPU-ID: ");
  for (i=0; i<12; i++) {
    printf("%02x", uid[i]);
  }
  printf("\r\n");

  // Initializing pressure sensor (if present ...)
  lps25hInit(&hi2c1);
  testSupportPrintStart("Initializing pressure sensor");
  if (lps25hTestConnection()) {
    printf("[OK]\r\n");
    lps25hSetEnabled(true);
  } else {
    printf("[FAIL] (%u)\r\n", (unsigned int)hi2c1.ErrorCode);
    selftestPasses = false;
  }

  testSupportPrintStart("Pressure sensor self-test");
  testSupportReport(&selftestPasses, lps25hSelfTest());

  // Initializing i2c eeprom
  eepromInit(&hi2c1);
  testSupportPrintStart("EEPROM self-test");
  testSupportReport(&selftestPasses, eepromTest());

  // Initialising radio
  testSupportPrintStart("Initialize DWM1000");
  dwInit(dwm, &dwOps);       // Init libdw
  dwOpsInit(dwm);
  result = dwConfigure(dwm); // Configure the dw1000 chip
  if (result == 0) {
    printf("[OK]\r\n");
    dwEnableAllLeds(dwm);
  } else {
    printf("[ERROR]: %s\r\n", dwStrError(result));
    selftestPasses = false;
  }

  if (!selftestPasses) {
    printf("TEST\t: One or more self-tests failed, blocking startup!\r\n");
    usbcommSetSystemStarted(true);
    while(1) {
      usbcommTick();
    }
  }

  cfgInit();

  if (cfgReadU8(cfgAddress, &address[0])) {
    printf("CONFIG\t: Address is 0x%X\r\n", address[0]);
    position[0] = positions[address[0]][0];
    position[1] = positions[address[0]][1];
    position[2] = positions[address[0]][2];
    printf("CONFIG\t: Position: x: %f, y: %f, z: %f", position[0], position[1], position[2]);
  } else {
    printf("CONFIG\t: Address not found!\r\n");
  }

  if (cfgReadU8(cfgMode, &mode)) {
    printf("CONFIG\t: Mode is ");
    switch (mode) {
      case modeAnchor: printf("Anchor\r\n"); break;
      case modeTag: printf("Tag\r\n"); break;
      case modeSniffer: printf("Sniffer\r\n"); break;
      default: printf("UNKNOWN\r\n"); break;
    }
  } else {
    printf("Device mode: Not found!\r\n");
  }

  uint8_t anchorListSize = 0;
  if (cfgFieldSize(cfgAnchorlist, &anchorListSize)) {
    if (cfgReadU8list(cfgAnchorlist, (uint8_t*)&anchors, anchorListSize)) {
      printf("CONFIG\t: Tag mode anchor list (%i): ", anchorListSize);
      for (i = 0; i < anchorListSize; i++) {
        printf("0x%02X ", anchors[i]);
      }
      printf("\r\n");
    } else {
      printf("CONFIG\t: Tag mode anchor list: Not found!\r\n");
    }
  }

  HAL_Delay(500);

  ledOff(ledRanging);
  ledOff(ledSync);
  ledOff(ledMode);

  dwAttachSentHandler(dwm, txcallback);
  dwAttachReceivedHandler(dwm, rxcallback);
  dwAttachReceiveFailedHandler(dwm, rxFailedCallback);

  dwNewConfiguration(dwm);
  dwSetDefaults(dwm);
  dwInterruptOnReceiveFailed(dwm, true);
  dwSetReceiverAutoReenable(dwm, false);
  dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
  dwSetChannel(dwm, CHANNEL_2);
  dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);


  dwCommitConfiguration(dwm);

  printf("SYSTEM\t: Node started ...\r\n");
  printf("SYSTEM\t: Press 'h' for help.\r\n");

  usbcommSetSystemStarted(true);

  bool ledState = false;
  uint32_t ledTick = 0;

  // Main loop ...
  while(1) {
    usbcommTick();

    // Accepts serial commands
#ifdef USE_FTDI_UART
    if (HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, 0) == HAL_OK) {
#else
    if(usbcommRead(&ch, 1)) {
#endif
      handleInput(ch);
    }

    StateMachineStep(receivedFlag, receiveFailedFlag, transmitedFlag);
    
    // Handling of the LEDs
    if (HAL_GetTick() > (ledTick+250)) {
      ledTick = HAL_GetTick();
      ledState = !ledState;
    }

     //ledOff(ledSync);

     switch (mode) {
       case modeTag:
         ledOff(ledMode);
         break;
       case modeAnchor:
         ledOn(ledMode);
         break;
       case modeSniffer:
         if (ledState) {
           ledOn(ledMode);
         } else {
           ledOff(ledMode);
         }
         break;
       default:
         ledOn(ledMode);
         ledOn(ledSync);
         ledOn(ledRanging);
         break;
     }
  }

  return 0;
}

/* Function required to use "printf" to print on serial console */
int _write (int fd, const void *buf, size_t count)
{
#ifdef USE_FTDI_UART
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, count, HAL_MAX_DELAY);
#else
  usbcommWrite(buf, count);
#endif
  return count;
}

static void handleInput(char ch) {
  bool configChanged = true;

  switch (ch) {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      changeAddress(ch - '0');
      break;
    case 'a': changeMode(modeAnchor); break;
    case 't': changeMode(modeTag); break;
    case 's': changeMode(modeSniffer); break;
    case 'd': restConfig(); break;
    case 'h':
      help();
      configChanged = false;
      break;
    case '#':
      productionTestsRun();
      printf("System halted, reset to continue\r\n");
      while(true){}
      break;
    default:
      configChanged = false;
      break;
  }

  if (configChanged) {
    printf("EEPROM configuration changed, restart for it to take effect!\r\n");
  }
}

static void restConfig() {
  printf("Resetting EEPROM configuration...");
  if (cfgReset()) {
    printf("OK\r\n");
  } else {
    printf("ERROR\r\n");
  }
}

static void changeAddress(uint8_t addr) {
  printf("Updating address from 0x%02X to 0x%02X\r\n", address[0], addr);
  cfgWriteU8(cfgAddress, addr);
  if (cfgReadU8(cfgAddress, &address[0])) {
    printf("Device address: 0x%X\r\n", address[0]);
  } else {
    printf("Device address: Not found!\r\n");
  }
}

static void changeMode(CfgMode newMode) {
    printf("Previous device mode: ");
    printMode();

    cfgWriteU8(cfgMode, newMode);

    printf("New device mode: ");
    printMode();
}

static void printMode() {
  CfgMode mode;

  if (cfgReadU8(cfgMode, &mode)) {
    switch (mode) {
      case modeAnchor: printf("Anchor"); break;
      case modeTag: printf("Tag"); break;
      case modeSniffer: printf("Sniffer"); break;
      default: printf("UNKNOWN"); break;
    }
  } else {
    printf("Not found!");
  }

  printf("\r\n");
}

static void help() {
  printf("Help\r\n");
  printf("-------------------\r\n");
  printf("0-9 - set address\r\n");
  printf("a   - anchor mode\r\n");
  printf("t   - tag mode\r\n");
  printf("s   - sniffer mode\r\n");
  printf("d   - reset configuration\r\n");
  printf("h   - This help\r\n");
}
