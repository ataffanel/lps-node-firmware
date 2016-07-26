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
 * along with the LPS Firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "statemachine.h"


#define SPEED_OF_LIGHT (299792458)
#define TICKS_PER_SECOND (63897600000ULL)
#define TICKS_PER_METER (213.139451293f)
#define RFDELAY (32930)

#define MASK_40 (0x000000FFFFFFFFFFULL)  // Time counter is 40 bits
#define MASK_TX (0x000000FFFFFFFE00ULL)  // TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//--------------- Application Constants ---------------//

#define APP_ANCHOR_COUNT   (6) // how many TDOA slots to allocate
#define APP_PACKET_HISTORY (3) // how many packets to store. Minimum is 2
#define APP_LISTEN_TIME_s  (0.5f) // how long to stay in the listen state before switching to transmit

#define APP_TRANSMIT_PERIOD_s     (1.5e-3f) // period of anchor transmission (time between transmissions from any anchors)
#define APP_TRANSMIT_BUFFER_MAX_s (1.25e-3f) // maximum time before transmission that an anchor can schedule an update. Should be less than the transmit period to avoid missing packets
#define APP_TRANSMIT_BUFFER_MIN_s (0.3e-3f) // minimum time before transmission that an anchor can schedule an update. Should be at least a few hundred microseconds, to allow for processing time.
#define APP_SLICE_LENGTH_s (APP_TRANSMIT_PERIOD_s * APP_ANCHOR_COUNT) // total round time of the system (time between a single anchor transmitting)

#define APP_MIN_PACKET_TX (1000) // how many packets do we need from id=0 before we start transmitting?
#define APP_MIN_PACKET_SYNC (800) // how many packets do we need from them before we start incorporating their clock into the system clock?

//--------------- Estimator Constants ---------------//

#define DISTANCE_STEP_MAX (0.0001f)
#define DISTANCE_STEP_MUL (0.1f)
#define CLOCK_PROCESS_VAR (1) // The CT variance of the clock's process noise
#define MEASUREMENT_VAR (4.216965034285822e-05) // The variance of a packet reception measurement

#define APP_MIN_PACKET_POS (1200)
#define CLOCK_VAL_SS_VAR (7.358604e-06) // The steady state (calculated using the above two variances) variance of the clock's value after measurement update (P^\inf_m[0,0])
#define CLOCK_RATE_SS_VAR (1.636265e-03) // The steady state (calculated using the above two variances) variance of the clock's rate after measurement update (P^\inf_m[1,1])

//--------------- State Machine Definitions ---------------//

typedef enum {
  STATE_ENTRY,
  STATE_RXLISTEN, STATE_RX, STATE_RXGOOD, STATE_RXERROR,
  STATE_TX, STATE_TXWAIT, STATE_TXGOOD, STATE_TXERROR, STATE_TXTIMEOUT,
  NUM_STATES
} State_t;

void StateEntry(bool rxgood, bool rxerror, bool txgood);

void StateRXListen(bool rxgood, bool rxerror, bool txgood);

void StateRX(bool rxgood, bool rxerror, bool txgood);

void StateRXGood(bool rxgood, bool rxerror, bool txgood);

void StateRXError(bool rxgood, bool rxerror, bool txgood);

void StateTX(bool rxgood, bool rxerror, bool txgood);

void StateTXWait(bool rxgood, bool rxerror, bool txgood);

void StateTXGood(bool rxgood, bool rxerror, bool txgood);

void StateTXError(bool rxgood, bool rxerror, bool txgood);

void StateTXTimeout(bool rxgood, bool rxerror, bool txgood);

//--------------- Data Defintions ---------------//

typedef struct {
  uint8_t senderID;
  uint16_t packetID;
  int64_t rxTime;
  int64_t txTime;
  int64_t sysTxTime;
} PacketLogEntry_t;

typedef struct {
  uint64_t packetCount;
  uint32_t ticksDistance;
  double systemSkew;
  int64_t systemOffset;
  double X[3];
  float x, y, z;
  bool fixedX, fixedY, fixedZ;
  bool positionInitialized;
  PacketLogEntry_t packetHistory[APP_PACKET_HISTORY];
  uint8_t packetHistoryFrontIdx;
} AnchorInformation_t;

typedef struct __attribute__((__packed__)) {
  uint8_t senderID;
  uint16_t packetID;
	int64_t rxTime;
} PacketRXData_t;

typedef struct __attribute__((__packed__)) { // note the order of fields here is to minimize the amount of data that a TDOA client needs to read
  int64_t txTime;           // ---
  float x, y, z;            //  | Needed for TDOA localization (and anchor sync & positioning)
  bool positionInitialized; // ---
  uint8_t senderID;         // ---
  uint16_t packetID;        //  |
  double systemSkew;        //  | Not needed for TDOA localization, but needed for anchor sync and anchor positioning
  int64_t systemOffset;     //  |
  PacketRXData_t rxPacket;  // ---
} Packet_t;


//--------------- SHARED DATA -----------------//

extern Packet_t txPacket;
extern Packet_t rxPacket;

extern AnchorInformation_t localData[APP_ANCHOR_COUNT];
extern AnchorInformation_t *my;

extern uint8_t boardID; //the anchor ID, read from the GPIO pins
extern uint16_t boardPacketID;
