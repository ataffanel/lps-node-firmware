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

#include "statemachine_tdoa.h"
#include "libdw1000.h"
#include "dw1000.h"
#include "libdw1000Spi.h"

extern dwDevice_t *dwm;

#define UWB_RXEnable() dwNewReceive(dwm); \
    dwSetDefaults(dwm); \
    dwInterruptOnReceiveFailed(dwm, true); \
    dwSetReceiverAutoReenable(dwm, false); \
    dwStartReceive(dwm)

#define UWB_ReceivedPacketLength() dwGetDataLength(dwm)

#define UWB_ReceivedPacketData(rxPacket, packetLength) dwGetData(dwm, rxPacket, packetLength)

static uint64_t UWB_LocalTime() {
  uint64_t timestamp = 0;
	dwSpiRead(dwm, SYS_TIME, NO_SUB, &timestamp, LEN_SYS_TIME);
  return timestamp;
}

double UWB_SecondsFromTicks(int64_t ticks) {
	return (double) ticks / 128.0 / 499.2e6;
}

int64_t UWB_TicksFromSeconds(double seconds) {
	return (int64_t) (seconds * 128.0 * 499.2e6);
}

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
	unsigned int idx;
	unsigned int shift;

	idx = bit / 8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	uint8_t* targetByte = &data[idx];
	shift = bit % 8;
	if(val) {
		*targetByte |= (1<<shift);
	} else {
	  *targetByte &= ~(1<<shift);
	}
}

static bool UWB_ScheduleMessage(uint64_t txTime, uint8_t *packet, uint8_t packetLength) {
	setBit(dwm->sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
  dwSpiWrite(dwm, DX_TIME, NO_SUB, &txTime, LEN_DX_TIME);

  dwNewTransmit(dwm);
  dwSetDefaults(dwm);
  dwSetData(dwm, packet, packetLength);
  dwStartTransmit(dwm);

  uint8_t data;
  dwSpiRead(dwm, SYS_STATUS_ID, 3, &data, 1);
  bool HPDWARN = (data & 0x08) != 0;
  if (HPDWARN) {
    dwIdle(dwm);
  }
  return !HPDWARN;
}

State_t lastState = STATE_ENTRY;
State_t currentState = STATE_ENTRY;
State_t nextState = STATE_ENTRY;

State_t RXMODE = STATE_RXLISTEN; // Holds the nextState for any states returning to receive, ie. whether we are listening only (STATE_RXLISTEN), or actively waiting to transmit (STATE_RX)

// Callback functions for the StateMachineStep
void (*StateFunction[NUM_STATES])(bool rxgood, bool rxerror, bool txgood) =
    { StateEntry,
      StateRXListen, StateRX, StateRXGood, StateRXError,
      StateTX, StateTXWait, StateTXGood, StateTXError, StateTXTimeout,
      StateFatal };

// Defines to handle entry and exit conditions
#define ENTRY if(lastState != currentState)
#define EXIT if(nextState != currentState)

sysTime_t stateMachineStart;
bool stateMachineListenOnly;

void StateEntry(bool rxgood, bool rxerror, bool txgood) {
  stateMachineStart = STOPWATCH_GetTicks();
  stateMachineListenOnly = true;

  RXMODE = STATE_RXLISTEN;
  nextState = RXMODE;
}


void StateRXListen(bool rxgood, bool rxerror, bool txgood) { // listen until the listen phase is over
  ENTRY {
    UWB_RXEnable();
  }

  if (STOPWATCH_SecondsSince(stateMachineStart) > APP_LISTEN_TIME_s && stateMachineListenOnly) {
    stateMachineListenOnly = !BUTTON_Read(); // if the button is pressed (true) then set the listen only to false
  }

  if (rxgood) {
    nextState = STATE_RXGOOD;
  } else if (rxerror) {
    nextState = STATE_RXERROR;
  } else if (STOPWATCH_SecondsSince(stateMachineStart) > APP_LISTEN_TIME_s && !stateMachineListenOnly) {
    if (boardID == 0 || localData[0].packetCount >= APP_MIN_PACKET_TX) {
      nextState = STATE_RX;
    }
  }
}


void StateRX(bool rxgood, bool rxerror, bool txgood) {
  ENTRY { // recalculate the next transmission time after every receive, since a reception will update our system time
    RXMODE = STATE_RX; // we are actively waiting to transmit, so all states should return here
    if (lastState != STATE_RXGOOD) { // if we were in RXGOOD, then RX is already enabled
      UWB_RXEnable();
    }

    double time_lt = UWB_SecondsFromTicks(UWB_LocalTime() + my->systemOffset); // current local time
    double time_st = time_lt + time_lt * my->systemSkew; // current system time
    nextTransmit_st = floor(time_st / APP_SLICE_LENGTH_s) * APP_SLICE_LENGTH_s +
                      boardID * APP_TRANSMIT_PERIOD_s; // start of next slice, plus board TDMA slot
    double timeToTransmit_st = nextTransmit_st - time_st;

    while (timeToTransmit_st < APP_TRANSMIT_BUFFER_MIN_s) { // if transmission is no longer possible in this round
      timeToTransmit_st += APP_SLICE_LENGTH_s; // try in the upcoming round
    }

    // convert the system transmission time into local time
    nextTransmit_st = time_st + timeToTransmit_st;
    nextTransmit_lt = UWB_TicksFromSeconds(nextTransmit_st / (1 + my->systemSkew)) - my->systemOffset;

    if (timeToTransmit_st < APP_TRANSMIT_BUFFER_MAX_s) { // if our transmission deadline is approaching
      nextState = STATE_TX;
      return;
    } else {
      STOPWATCH_ScheduleEventInSeconds((float)(timeToTransmit_st-APP_TRANSMIT_BUFFER_MAX_s));
    }
  }

  if (rxgood) {
    nextState = STATE_RXGOOD;
  } else if (rxerror) {
    nextState = STATE_RXERROR;
  } else if (STOPWATCH_EventHasOccurred()) { // it's time to transmit!
    nextState = STATE_TX;
  }
}


void StateRXGood(bool rxgood, bool rxerror, bool txgood)
{
  LED_Register_RX();

  // read the packet length (already compensated for additional CRC)
  uint16_t packetLength = UWB_ReceivedPacketLength();
  assert(packetLength == sizeof(Packet_t));

  // read the packet data
  UWB_ReceivedPacketData((uint8_t*)(&rxPacket), packetLength);

  // rounding is the lowest 9 bits of the transmission timestamp
  int64_t rounding = (rxPacket.txTime & (0x00000000000001FF));

  // calculate the desired receive time
  int64_t rxTime = UWB_ReceivedPacketTime() + rounding; // convert the device time into local time

  // enable the receiver such that we can receive the next packet while processing
  UWB_RXEnable();

  // process the packet
  ProcessReceivedPacket(&rxPacket, &rxTime);

  // transition states
  if (rxgood) { // if we've received a packet while processing, then come right back here!
    nextState = STATE_RXGOOD;
  } else if (rxerror) {
    nextState = STATE_RXERROR;
  } else {
    nextState = RXMODE;
  }
}


void StateTX(bool rxgood, bool rxerror, bool txgood) {
  txPacket.senderID = boardID;
  txPacket.packetID = boardPacketID;
  txPacket.systemOffset = my->systemOffset;
  txPacket.systemSkew = my->systemSkew;
  txPacket.txTime = nextTransmit_lt;
  txPacket.sysTxTime = UWB_TicksFromSeconds(nextTransmit_st);
  txPacket.x = my->x;
  txPacket.y = my->y;
  txPacket.z = my->z;
  txPacket.positionInitialized = my->positionInitialized;
  memcpy(&(txPacket.lastCommand), &lastCommand, sizeof(Command_t));

  // update the partners
  for(uint8_t i = 0; i<APP_ANCHOR_COUNT; i++) {
    if (localData[i].packetCount >= APP_MIN_PACKET_SYNC) { // we have received enough packets from this partner to be synced (hence the filteredRxTime is trustworthy)
      txPacket.rxPacket[i].packetID = localData[i].packetHistory[0].packetID;
      txPacket.rxPacket[i].filteredRxTime = localData[i].packetHistory[0].filteredRxTime;
      txPacket.rxPacket[i].ticksDistanceTimes16 = (uint16_t)UWB_TicksFromSeconds(16*localData[i].timeDistance); // this fits for distances less than 18m
      txPacket.rxPacket[i].bias = (uint16_t)(32767*localData[i].bias); // for biases less than 1m, this fits and gives a resolution of 0.03mm.
    } else {
      txPacket.rxPacket[i].packetID = 0;
      txPacket.rxPacket[i].filteredRxTime = 0;
      txPacket.rxPacket[i].ticksDistanceTimes16 = 0;
      txPacket.rxPacket[i].bias = 0;
    }
  }

  bool successfulTransmission = UWB_ScheduleMessage(nextTransmit_lt, (uint8_t *) (&txPacket), sizeof(txPacket));

  if (successfulTransmission) {
    my->packetCount += 1;
    nextState = STATE_TXWAIT; // wait for confirmation from the interrupt
  } else {
    nextState = STATE_TXERROR; // otherwise there was no time to transmit
  }
}


void StateTXWait(bool rxgood, bool rxerror, bool txgood) {
  ENTRY {
    STOPWATCH_ScheduleEventInSeconds((float)APP_TRANSMIT_PERIOD_s);
  }

  if (txgood) {
    nextState = STATE_TXGOOD;
  } else if (STOPWATCH_EventHasOccurred()) { // the timer has finished without a transmission having occurred! timeout.
    nextState = STATE_TXTIMEOUT;
  }
}


void StateTXGood(bool rxgood, bool rxerror, bool txgood) {
  // save the sent packet in the packet log
  PacketLogEntry_t packetLog;
  packetLog.senderID = txPacket.senderID;
  packetLog.packetID = txPacket.packetID;
  packetLog.txTime = txPacket.txTime;
  packetLog.rxTime = 0;
  ARRAY_PUSH(packetLog, my->packetHistory, APP_PACKET_HISTORY);
  boardPacketID += 1;

  LED_Register_TX();
  nextState = RXMODE;
}


void StateRXError(bool rxgood, bool rxerror, bool txgood) {
//	VCP_WriteString("RX Error");
  LED_Register_ALERT();
//	UWB_RXReset();
  nextState = RXMODE;
}


void StateTXError(bool rxgood, bool rxerror, bool txgood) {
//	VCP_WriteString("TX Error");
  LED_Register_ERROR();
  nextState = RXMODE;
}


void StateTXTimeout(bool rxgood, bool rxerror, bool txgood) {
//	VCP_WriteString("TX Timeout");
  LED_Register_ERROR();
  nextState = RXMODE;
}


void StateFatal(bool rxgood, bool rxerror, bool txgood) {
  PANIC();
}


void StateMachineStep(bool rxgood, bool rxerror, bool txgood) {
  lastState = currentState;
  currentState = nextState;

  StateFunction[currentState](rxgood, rxerror, txgood);
}
