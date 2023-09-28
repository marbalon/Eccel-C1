/*
  binary_protocol.h - this file is part of the C1 
  library for Pepper C1 modules by Eccel Technology Ltd.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

   Authors:  Marcin Baliniak

   Version: 1.0

  See file LICENSE.txt for further informations on licensing terms.
*/


#ifndef __BINARY_PROTOCOL_H__
#define __BINARY_PROTOCOL_H__

#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_BINNARY_FRAME_LEN_LEN (2048+20)
#define BINARY_STX	0xF5

typedef void (*bp_calback)(uint8_t *data, size_t len);

class BinaryProtocol
{
public:
    BinaryProtocol(bp_calback frameComplete_cb, bp_calback protocolWrite_cb);
    ~BinaryProtocol();

    void send(uint8_t *buff, size_t len);
    void writeRaw(uint8_t *buff, size_t len);
    void repeat(void);
    uint16_t parse(uint8_t *buff, size_t len);
    uint8_t protocolBuffIn[MAX_BINNARY_FRAME_LEN_LEN];    
    uint16_t protocolLenIn;    
private:
    uint16_t protocolReqLen=0;
    uint8_t protocolBuffOut[MAX_BINNARY_FRAME_LEN_LEN];
    uint16_t protocolLenOut=0;
    int error_count;
    bool header_detected;
    /*
signals:
    void frameComplete(uint8_t *buff, size_t len);
    void frameError();
    void frameTimeout();
    void protocolWrite(uint8_t *buff, size_t len);*/

    bp_calback frameComplete;
    bp_calback protocolWrite;
};
#endif

