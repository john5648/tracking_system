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
/* 
lpsTwr2Tag.c: Uwb two way ranging between two crazyflies
One Crazyflie should have last number of own address as A(which is 10 in decimal)
The other can be any address except A

Reference
1. TWR algorithm between multiple crazyflies: https://github.com/shushuai3/crazyflie-firmware/tree/interRanging
2. Algorithm mode change: https://github.com/Williamwenda/crazyflie-firmware/tree/dev/tdoa3_plus
 */


#include <string.h>
#include <math.h>

#include "lpsTwr2Tag.h"
#include "lpsTdma.h"

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "crtp_localization_service.h"

#include "stabilizer_types.h"
#include "estimator.h"
#include "cf_math.h"

#include "physicalConstants.h"
#include "configblock.h"
#include "lpsTdma.h"
#include "static_mem.h"

// Config
static lpsTwr2AlgoOptions_t defaultOptions = {

   .antennaDelay = LOCODECK_ANTENNA_DELAY,

};

#define basicAddr 0xbccf000000000000
// selfID = last number of own address, if URI = 'radio://0/120/2M/E7E7E7E7EB' then selfID = 11
static uint8_t selfID; 
static uint8_t droneaddress[5] = {11,12,13,14,15};
static uint8_t UAVnum=3;
static locoAddress_t selfAddress;
static float filtered_dis;

int MODE = 4;
static int strikeout;
bool twragain = false;

int switchAgentMode(){
    return MODE;
}

typedef struct {
  float distance;
} twr2State_t;

static twr2State_t state;
static lpsTwr2AlgoOptions_t* options = &defaultOptions;

static uint8_t current_receiveID; // transmit to which crazyflie

static bool checkTurn; // check if the receiving UWB turns into transmitting mode

// Outlier rejection
#define RANGING_HISTORY_LENGTH 32
#define OUTLIER_TH 4
NO_DMA_CCM_SAFE_ZERO_INIT static struct {
  float32_t history[RANGING_HISTORY_LENGTH];
  size_t ptr;
} rangingStats;

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static volatile uint8_t curr_seq = 0;

static bool rangingOk;

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (options->antennaDelay / 2);

  switch (txPacket.payload[0]) {
    case LPS_TWR_POLL:
      poll_tx = departure;
      break;
    case LPS_TWR_FINAL:
      final_tx = departure;
      break;
    case LPS_TWR_ANSWER:
      answer_tx = departure;
      break;
    case LPS_TWR_REPORT:
      break;
    case LPS_TWR_REPORT+1:
      break;
  }
}


static uint32_t rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);

  if (dataLength == 0) return 0;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  // go back to TDOA2 if next crazyflie starts twr algorithm
  if(selfID !=10 && (rxPacket.sourceAddress & 0x0f)>10){
    MODE = lpsMode_TDoA2;
    return 0;
  }else if(rxPacket.destAddress != selfAddress){
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return MAX_TIMEOUT;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  switch(rxPacket.payload[LPS_TWR_TYPE]) {
    case LPS_TWR_POLL:
      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      poll_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;

    case LPS_TWR_ANSWER:
      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return 0;
      }

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      answer_rx = arival;

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;

    case LPS_TWR_FINAL:
    {
      lpsTwr2TagReportPayload_t *report = (lpsTwr2TagReportPayload_t *)(txPacket.payload+2);

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwGetReceiveTimestamp(dev, &arival);
      arival.full -= (options->antennaDelay / 2);
      final_rx = arival;

      memcpy(&report->pollRx, &poll_rx, 5);
      memcpy(&report->answerTx, &answer_tx, 5);
      memcpy(&report->finalRx, &final_rx, 5);

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(lpsTwr2TagReportPayload_t));

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    }
    case LPS_TWR_REPORT:
    {
      if (rxPacket.payload[LPS_TWR_SEQ] != curr_seq) {
        return 0;
      }

      lpsTwr2TagReportPayload_t *report = (lpsTwr2TagReportPayload_t *)(rxPacket.payload+2);
      double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

      memcpy(&poll_rx, &report->pollRx, 5);
      memcpy(&answer_tx, &report->answerTx, 5);
      memcpy(&final_rx, &report->finalRx, 5);

      tround1 = answer_rx.low32 - poll_tx.low32;
      treply1 = answer_tx.low32 - poll_rx.low32;
      tround2 = final_rx.low32 - answer_tx.low32;
      treply2 = final_tx.low32 - answer_rx.low32;

      tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

      tprop = tprop_ctn / LOCODECK_TS_FREQ;
      state.distance = SPEED_OF_LIGHT * tprop;

      // Outliers rejection
      rangingStats.ptr = (rangingStats.ptr + 1) % RANGING_HISTORY_LENGTH;
      float32_t mean;
      float32_t stddev;

      arm_std_f32(rangingStats.history, RANGING_HISTORY_LENGTH, &stddev);
      arm_mean_f32(rangingStats.history, RANGING_HISTORY_LENGTH, &mean);
      float32_t diff = fabsf(mean - state.distance);

      rangingStats.history[rangingStats.ptr] = state.distance;

      rangingOk = true;

      if (diff < (OUTLIER_TH*stddev)) {
        filtered_dis = state.distance;
      }

      txPacket.destAddress = basicAddr + droneaddress[(selfID-10)%UAVnum];
      txPacket.sourceAddress = selfAddress;

      txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT+1;
      txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];

      dwNewTransmit(dev);
      dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

      dwWaitForResponse(dev, true);
      dwStartTransmit(dev);

      break;
    }
  }
  return MAX_TIMEOUT;
}

static void initiateRanging(dwDevice_t *dev)
{

  dwIdle(dev);

  if(selfID==10)
  {
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
  }
  else
  {
    txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
    txPacket.payload[LPS_TWR_SEQ] = ++curr_seq;

    txPacket.sourceAddress = selfAddress;
    txPacket.destAddress = basicAddr + current_receiveID;

    dwNewTransmit(dev);
    dwSetDefaults(dev);
    dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

    dwWaitForResponse(dev, true);
    dwStartTransmit(dev);
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{

  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      return MAX_TIMEOUT;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      if(checkTurn){
        strikeout = strikeout + 1;
      }
      if(strikeout >=3){
        twragain = true;
        MODE = lpsMode_TDoA2;
        return MAX_TIMEOUT;
      }
      strikeout = strikeout + 1;
      if (rangingOk == false)
      {
        initiateRanging(dev);
      }else if(rangingOk == true){
        txPacket.destAddress = basicAddr + droneaddress[(selfID-10)%UAVnum];
        txPacket.sourceAddress = selfAddress;

        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT+1;
        txPacket.payload[LPS_TWR_SEQ] = 0;

        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);      
      }  
      break;
    case eventReceiveTimeout:
    case eventReceiveFailed:
      return 0;
      break;
    default:
      configASSERT(false);
  }

  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{

  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  curr_seq = 0;
  strikeout = 0;

  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f));
  selfAddress = basicAddr + selfID;

  current_receiveID = 10;

  dwSetReceiveWaitTimeout(dev, TWR2_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}

static bool isRangingOk()
{
  return rangingOk;
}

void uwbTwr2TagSetOptions(lpsTwr2AlgoOptions_t* newOptions) {
  options = newOptions;
}

uwbAlgorithm_t uwbTwr2TagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
};

/**
 * Log group for distances (ranges) to anchors aquired by Two Way Ranging (TWR)
 */
LOG_GROUP_START(ranging)
LOG_ADD(LOG_FLOAT, distance0, &filtered_dis)
LOG_GROUP_STOP(ranging)
