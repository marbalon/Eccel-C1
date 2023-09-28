/*
  binary_protocol.cpp - this file is part of the C1 
  library for Pepper C1 modules by Eccel Technology Ltd.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

   Authors:  Marcin Baliniak

   Version: 1.0

  See file LICENSE.txt for further informations on licensing terms.
*/

#include <stdio.h>
#include <string.h>
#include "ccittcrc.h"
#include <time.h>
#include "binary_protocol.h"

BinaryProtocol::BinaryProtocol(bp_calback frameComplete_cb, bp_calback protocolWrite_cb)
{
    frameComplete = frameComplete_cb;
    protocolWrite = protocolWrite_cb;
    header_detected = false;
}

BinaryProtocol::~BinaryProtocol()
{

}

void BinaryProtocol::repeat(void)
{
    if (protocolLenOut > 0)
    {
        protocolWrite(protocolBuffOut, protocolLenOut);
    }
}

void BinaryProtocol::writeRaw(uint8_t *buff, size_t len)
{
    protocolWrite(buff, len);
}

void BinaryProtocol::send(uint8_t *buff, size_t len)
{
    uint16_t crc;
    protocolLenOut = 0;

    if (buff[0] == 0xFF)
    {
        error_count++;
    }

    protocolBuffOut[protocolLenOut++] = BINARY_STX;

    protocolBuffOut[protocolLenOut++] = (len + 2) & 0xff;
    protocolBuffOut[protocolLenOut++] = ((len + 2) >> 8) & 0xff;

    protocolBuffOut[protocolLenOut++] = protocolBuffOut[1] ^ 0xff;
    protocolBuffOut[protocolLenOut++] = protocolBuffOut[2] ^ 0xff;
    memcpy(&protocolBuffOut[protocolLenOut], buff, len);

    crc = GetCCITTCRC(&protocolBuffOut[protocolLenOut], len);

    protocolLenOut+=len;

    protocolBuffOut[protocolLenOut++] = (uint8_t)(crc & 0x00FF);
    protocolBuffOut[protocolLenOut++] = (uint8_t)((crc >> 8) & 0x00FF);

    protocolWrite(protocolBuffOut, protocolLenOut);
}

uint16_t BinaryProtocol::parse(uint8_t *buff, size_t len)
{
	size_t k;
    uint8_t cmd[5];
    uint16_t res = 1;
    static uint8_t header[5];
    //static uint32_t start_frame_time;

	for (k=0; k< len; k++)
	{
		header[0] = header[1];
		header[1] = header[2];
		header[2] = header[3];
		header[3] = header[4];
		header[4] = buff[k];

		if (header_detected == false)
		{
			if (header[0] == BINARY_STX && header[1] == (header[3] ^ 0xff) && header[2] == (header[4] ^ 0xff))
			{
				//header detected
				protocolReqLen = header[1] | (header[2] << 8);

				if (protocolReqLen > sizeof(protocolBuffIn))
				{
					continue;
				}
				protocolLenIn = 0;
				res = protocolReqLen;
				header_detected = true;
			}
		}
		else
		{
			protocolBuffIn[protocolLenIn++] = buff[k];
			if (protocolLenIn == protocolReqLen)
			{
				header_detected = false;
				if (GetCCITTCRC(protocolBuffIn, protocolLenIn - 2) != (uint16_t)(protocolBuffIn[protocolLenIn-2]) + (uint16_t)(protocolBuffIn[protocolLenIn-1] << 8))
				{
					cmd[0] = 0xff; //protocol error
					cmd[1] = 0xff;
					cmd[2] = 0x02; //wrong CRC
                    send(cmd, 3);
					if (buff[k] == BINARY_STX)
					{
						protocolLenIn = 0;
					}
					break;
                    //emit frameError();
				}
                protocolLenIn -=2; //remove CRC
                frameComplete(protocolBuffIn, protocolLenIn);
			}
		}
	}

	return res;
}


