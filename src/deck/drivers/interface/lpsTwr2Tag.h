#ifndef __LPS_TWR2_TAG_H__
#define __LPS_TWR2_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

#define LPS_TWR_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWR_ANSWER 0x02
#define LPS_TWR_FINAL 0x03
#define LPS_TWR_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWR_TYPE 0
#define LPS_TWR_SEQ 1

extern uwbAlgorithm_t uwbTwr2TagAlgorithm;

int switchAgentMode();
extern int MODE;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

} __attribute__((packed)) lpsTwr2TagReportPayload_t;

typedef struct {
  const uint64_t antennaDelay;

} lpsTwr2AlgoOptions_t;


void uwbTwr2TagSetOptions(lpsTwr2AlgoOptions_t* newOptions);

#define TWR2_RECEIVE_TIMEOUT 1000

#endif // __LPS_TWR2_TAG_H__
