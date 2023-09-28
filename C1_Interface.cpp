/*
  C1_Interface.cpp - this file is part of the C1 
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
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include "C1_Interface.h"

#include <HardwareSerial.h>
extern HardwareSerial SerialPort;

static Stream *serialPort;
static bool frame_received = false;
static uint16_t last_result;

uint16_t GetKeySize(uint16_t key_type)
{
    switch (key_type)
    {
        case KEY_TYPE_MIFARE:
            /* 6 bytes for key A + 6 bytes for key B */
            return 12;

        case KEY_TYPE_DES:
            return 16;

        case KEY_TYPE_2K3DES:
        case KEY_TYPE_AES128:
            return 16;

        case KEY_TYPE_3K3DES:
        case KEY_TYPE_AES192:
            return 24;

        case KEY_TYPE_AES256:
            return 32;

        default:
            return 0;
    }
}

void uartWrite(uint8_t *data, size_t len)
{
    serialPort->write(data,len);
    //SerialPort.printf("uartWrite:%d bytes\n", len);
}

void frameComplete(uint8_t *data, size_t len)
{
    switch (data[0])
    {
        case CMD_ACK:
            last_result = 0;
        break;
        case CMD_ERROR:
            last_result = (data[2] << 8) | (data[3]);
        break;
    }
    //SerialPort.printf("frameComplete:%d bytes, cmd:0x%02X, res: 0x%04X\n", len, data[1], last_result);

    frame_received = true;    
}

bool C1_Interface::wait4Answer(uint16_t timeout)
{
    uint8_t buff[1024];
    uint32_t tout = millis() + timeout;
    int lenght;

    frame_received = false;
    last_result = 0xff;
    
    while (millis() < tout && frame_received == false)
    {

      lenght = serialPort->available();
      if (lenght > 0)
      {
        if (lenght > sizeof(buff))
          lenght = sizeof(buff);

        lenght = serialPort->readBytes(buff, lenght);
        if (lenght > 0)
          binary_protocol->parse(buff, lenght);
      }
    }

    return frame_received;
}


C1_Interface::C1_Interface(Stream &serial)
{
    serialPort = &serial;
    binary_protocol = new BinaryProtocol(frameComplete, uartWrite);
}

/**
    @brief Function used to send dummy command packet to test connection
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::SendDummyCommand()
{
    uint8_t cmd = CMD_DUMMY_COMMAND;
    binary_protocol->send(&cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);
    return last_result;
}

/**
    @brief The command send to the module to read how many TAGS are in range of the antenna no matter which technology of tag, 
            so it returns the total amount present of all supported tag types.  
    @param count (out) number of tags
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::GetTagCount(uint8_t *count)
{
    uint8_t cmd = CMD_GET_TAG_COUNT;
    binary_protocol->send(&cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);
    *count = 0;
    if (last_result == 0)
    {
        *count = binary_protocol->protocolBuffIn[2];
    }

    return last_result;
}

/**
    @brief This command should be executed after GET_TAG_COUNT frame to read information about the tag
    @param idx tag index. Should be always less then tags detected count
    @param type (out) tag type
    @param param (out) tag parameter (SAK - byte for Mifare family tags, DSFID - byte for ICODE family tags)
    @param uid (out)  UID bytes
    @param uid_len (out) UID len
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::GetUid(uint8_t idx, uint8_t *type, uint8_t *param, uint8_t *uid, uint8_t *uid_len)
{
    uint8_t cmd[3];
    cmd[0] = CMD_GET_UID;
    cmd[1] = idx;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    if (last_result == 0)
    {
        *type = binary_protocol->protocolBuffIn[2];
        *param = binary_protocol->protocolBuffIn[3];
        memcpy(uid, &binary_protocol->protocolBuffIn[4], binary_protocol->protocolLenIn - 4);
        *uid_len = binary_protocol->protocolLenIn - 4;
    }

    return last_result;
}

/**
    @brief The command executed to activate a TAG after the discovery loop if more than one TAG is detected
    @param idx tag index
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ActivateTag(uint8_t idx)
{
    uint8_t cmd[2];

    cmd[0] = CMD_ACTIVATE_TAG;
    cmd[1] = idx;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief The Halt command takes no arguments. It halts the tag and turns off the RF field
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::Halt(void)
{
    uint8_t cmd[2];

    cmd[0] = CMD_HALT;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief The command used to enable/dissable internal polling
    @param on Internal polling state 0-off, 1-on
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::SetPolling(uint8_t on)
{
    uint8_t cmd[2];

    cmd[0] = CMD_SET_POLLING;
    cmd[1] = on;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command sets a KEY in Key Storage Memory on a selected slot. Set key can be used for all RFID functions 
            needing authorization like e.g. READ/WRITE memory on the TAG etc. This command changes a key in volatile memory, 
            so if the user wants to save it permanently and load automatically after boot-up, then the user 
            should use the  SaveKeys command.
    @param key_no Key number slot in range 0-4
    @param key_type Key type
    @param key key bytes. The lenght must be exacly as needed for key_type. Read manual for more informations

    @return 0 in case of success or error code
*/
uint16_t C1_Interface::SetKey(uint8_t key_no, uint8_t key_type, uint8_t *key)
{
    uint8_t cmd[40];
    uint8_t key_len = GetKeySize(key_type);

    cmd[0] = CMD_SET_KEY;
    cmd[1] = key_no;
    cmd[2] = key_type;
    memcpy(&cmd[3], key, key_len);

    binary_protocol->send(cmd, key_len+3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command should be called if the user wants to save keys changed using the SetKey() command 
    in the module non-volatile memory. Saved keys will be automatically loaded after power up or reboot.
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::SaveKeys(void)
{
    uint8_t cmd[2];

    cmd[0] = CMD_SAVE_KEYS;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command requests a software reboot for the Pepper C1 module. After this command the device will not accept any protocol commands for 1 second
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::Reboot(void)
{
    uint8_t cmd[2];

    cmd[0] = CMD_REBOOT;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command requests a version string from the device
    @param version (out) pointer to output buffer for version string
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::GetVersion(char *version)
{
    uint8_t cmd[2];

    cmd[0] = CMD_GET_VERSION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);
    if (last_result == 0)
    {
        memcpy(version, binary_protocol->protocolBuffIn+2, binary_protocol->protocolLenIn - 2);
        version[binary_protocol->protocolLenIn - 2] = 0;
    }
    return last_result;
}

/**
    @brief This command requests the device to enter in to sleep mode. Please read the “Sleep mode” chapter to get more information about this feature
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::Sleep(void)
{
    uint8_t cmd[2];

    cmd[0] = CMD_SLEEP;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command should be user to perform a factory reset. After this command the device will not accept any protocol commands for 1 second
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::FactoryReset(void)
{
    uint8_t cmd[] = {CMD_FACTORY_RESET, 0x01, 0x02, 0x03, 0x04};

    binary_protocol->send(cmd, 5);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command should be used to control the built-in LED. The first three bytes are the RGB value 
            of the colour and the optional two bytes are the timeout in milliseconds
    @param r,g,b RED,GREEN,BLUE value of the color in range 0-255
    @param timeout timeout value for blink event. Value 0 sets the color permanently
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::SetLed(uint8_t r, uint8_t g, uint8_t b, uint16_t timeout)
{
    uint8_t cmd[8];

    cmd[0] = CMD_LED;
    cmd[1] = r;
    cmd[2] = g;
    cmd[3] = b;
    cmd[4] = timeout & 0xff;
    cmd[5] = (timeout>>8) & 0xff;

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief The read block command should be used to read data from the tag
    @param block_no start block number
    @param block_count numer of block to read
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param buff (out) output buffer for tag data. Size of the buffer must be at least 16*block_count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_ReadBlock(uint8_t block_no, uint8_t block_count, uint8_t key_type, uint8_t key_no, uint8_t *buff)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MF_READ_BLOCK;
    cmd[1] = block_no;
    cmd[2] = block_count;
    cmd[3] = key_type;
    cmd[4] = key_no;

    binary_protocol->send(cmd, 5);
    ///wait for ACK and return status
    wait4Answer(500);
    memcpy(buff, &binary_protocol->protocolBuffIn[2], block_count*16);

    return last_result;
}

/**
    @brief The write block command should be used to write data to the tag
    @param block_no start block number
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param buff 16-bytes of data for each block to write
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_WriteBlock(uint8_t block_no, uint8_t block_count, uint8_t key_type, uint8_t key_no, uint8_t *buff)
{
    uint8_t cmd[24];

    cmd[0] = CMD_MF_WRITE_BLOCK;
    cmd[1] = block_no;
    cmd[2] = 1;
    cmd[3] = key_type;
    cmd[4] = key_no;
    memcpy(&cmd[5], buff, 16*block_count);

    binary_protocol->send(cmd, 5+(16*block_count));
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command should be used to read a value from the tag
    @param block_no block number
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param value (out) 32bit value read from the tag
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_ReadValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, uint32_t *value)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MF_READ_VALUE;
    cmd[1] = block_no;
    cmd[2] = key_type;
    cmd[3] = key_no;

    binary_protocol->send(cmd, 4);
    ///wait for ACK and return status
    wait4Answer(500);
    *value = (binary_protocol->protocolBuffIn[2] << 24) | (binary_protocol->protocolBuffIn[3] << 16) | (binary_protocol->protocolBuffIn[4] << 8) | (binary_protocol->protocolBuffIn[5]);

    return last_result;
}

/**
    @brief This command should be used to write a value from the tag
    @param block_no block number
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param value 32bit value to write
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_WriteValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, uint32_t value)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MF_READ_VALUE;
    cmd[1] = block_no;
    cmd[2] = key_type;
    cmd[3] = key_no;

    cmd[4] = (value >> 24) & 0xff;
    cmd[5] = (value >> 16) & 0xff;
    cmd[6] = (value >> 8) & 0xff;
    cmd[7] = (value) & 0xff;

    binary_protocol->send(cmd, 8);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command should be used to increment value storred in the tag
    @param block_no block number
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param value 32bit delta value to incremet
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_IncrementValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, int32_t value)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MF_INCREMENT;
    cmd[1] = block_no;
    cmd[2] = key_type;
    cmd[3] = key_no;
    cmd[4] = 0x01; //increment

    cmd[5] = (value >> 24) & 0xff;
    cmd[6] = (value >> 16) & 0xff;
    cmd[7] = (value >> 8) & 0xff;
    cmd[8] = (value) & 0xff;

    binary_protocol->send(cmd, 9);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command should be used to decrement value storred in the tag
    @param block_no block number
    @param key_type '0x0A' for Mifare Key A, or '0x0B' for key B
    @param key_no key number in key storage
    @param value 32bit delta value to decremet
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MC_DecrementValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, int32_t value)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MF_INCREMENT;
    cmd[1] = block_no;
    cmd[2] = key_type;
    cmd[3] = key_no;
    cmd[4] = 0x01; //increment

    cmd[5] = (value >> 24) & 0xff;
    cmd[6] = (value >> 16) & 0xff;
    cmd[7] = (value >> 8) & 0xff;
    cmd[8] = (value) & 0xff;

    binary_protocol->send(cmd, 9);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}





/**
    @brief The read page command should be used to read data from the tag
    @param page_no start page number
    @param page_count numer of page to read
    @param buff (out) output buffer for tag data. Size of the buffer must be at least 4*page_count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_ReadPage(uint8_t page_no, uint8_t page_count, uint8_t *buff)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFU_READ_PAGE;
    cmd[1] = page_no;
    cmd[2] = page_count;

    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);
    memcpy(buff, &binary_protocol->protocolBuffIn[2], page_count*4);

    return last_result;
}

/**
    @brief The write block command should be used to write data to the tag
    @param page_no start page number
    @param page_count numer of page to write
    @param buff input buffer for tag data. Size of the buffer must be at least 4*page_count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_WritePage(uint8_t page_no, uint8_t page_count, uint8_t *buff)
{
    uint8_t cmd[135];

    cmd[0] = CMD_MFU_WRITE_PAGE;
    cmd[1] = page_no;
    cmd[2] = page_count;
    memcpy(&cmd[3], buff, 4*page_count);

    binary_protocol->send(cmd, 3+(4*page_count));
    ///wait for ACK and return status
    wait4Answer(250*page_count);

    return last_result;
}

/**
    @brief This command requests a version string from the TAG
    @param buff (out) 8-bytes of version string
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_GetVersion(uint8_t *buff)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFU_GET_VERSION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(buff, &binary_protocol->protocolBuffIn[2], 8);

    return last_result;
}

/**
    @brief This command reads the signature information to the Mifare Ultralight Nano TAG
    @param buff (out) contains 32-bytes with ECC signature defined by the NXP standard
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_ReadSignature(uint8_t *buff)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFU_READ_SIG;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(buff, &binary_protocol->protocolBuffIn[2], 32);

    return last_result;
}

/**
    @brief This command writes the signature information to the Mifare Ultralight Nano TAG
    @param buff (in) contains 4-bytes with ECC signature defined by the NXP standard
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_WriteSignature(uint8_t *buff)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFU_WRITE_SIG;
    memcpy(&cmd[1], buff, 4);

    binary_protocol->send(cmd, 5);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command locks the signature temporarily or permanently based on the information provided in the API
    @param lock_byte lock mode byte
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_LockSignature(uint8_t lock_byte)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFU_LOCK_SIG;
    cmd[1] = lock_byte;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command should be used to read a counter from the TAG
    @param counter_no counter numer 0-2
    @param counter (out) counter 3-bytes (24bit value LSB first)
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_ReadCounter(uint8_t counter_no, uint8_t *counter)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFU_READ_COUNTER;
    cmd[1] = counter_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(counter, &binary_protocol->protocolBuffIn[2], 3);

    return last_result;
}

/**
    @brief This command should be used to read a counter from the TAG
    @param counter_no counter numer 0-2
    @param value 3-bytes (24bit value LSB first)
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_IncrementCounter(uint8_t counter_no, uint8_t *value)
{
    uint8_t cmd[6];

    cmd[0] = CMD_MFU_INCREMENT_COUNTER;
    cmd[1] = counter_no;
    memcpy(&cmd[2], value, 3);

    binary_protocol->send(cmd, 5);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief This command tries to authenticate the tag using the chosen password
    @param pass 4 bytes password
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_PasswordAuth(uint8_t *pass)
{
    uint8_t cmd[6];

    cmd[0] = CMD_MFU_PASSWD_AUTH;
    memcpy(&cmd[1], pass, 4);

    binary_protocol->send(cmd, 5);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to authenticate the Mifare Ultralight-C tag using the password stored in the key storage
    @param key_no key number in key stortage. Range 0-4
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_UlcAuth(uint8_t key_no)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFUC_AUTHENTICATE;
    cmd[1] = key_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}

/**
    @brief The Check Tearing Event command takes as arguments one byte with the counter number
    @param counter_no Counter number
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MU_CheckTearingEvent(uint8_t counter_no, uint8_t *event)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFU_CHECKEVENT;
    cmd[1] = counter_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(event, &binary_protocol->protocolBuffIn[2], 1);

    return last_result;
}


/**
    @brief This command requests version information from the tag
    @param version (out) 28-bytes of version
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_GetVersion(uint8_t *version)
{
    uint8_t cmd[2];

    cmd[0] = CMD_MFDF_GET_VERSION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(version, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}

/**
    @brief This command requests select application operation on the tag
    @param aid 3-byes containing AID
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_SelectApp(uint8_t *aid)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_SELECT_APP;
    cmd[1] = aid[0];
    cmd[2] = aid[1];
    cmd[3] = aid[2];

    binary_protocol->send(cmd, 4);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command requests list of application IDs from the TAG
    @param apps (out) bytes with application IDs. Every ID is 3-bytes long
    @param count (out) number of apps
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ListApps(uint8_t *apps, uint8_t *count)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_APP_IDS;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(apps, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);
    *count = binary_protocol->protocolLenIn/3;

    return last_result;
}

/**
    @brief This command requests list of file IDs from the TAG
    @param apps (out) bytes with file IDs. Every ID is 3-bytes long
    @param count (out) number of files
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ListFiles(uint8_t *files, uint8_t *count)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_FILE_IDS;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(files, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);
    *count = binary_protocol->protocolLenIn/3;

    return last_result;
}

/**
    @brief This command tries to authenticate the MIFARE Desfire using the password save in the key storage
    @param key_in_storage Key number in key storage
    @param key_on_card Key number on card
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_Authenticate(uint8_t key_in_storage, uint8_t key_on_card)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_AUTH;
    cmd[1] = key_in_storage;
    cmd[2] = key_on_card;


    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to authenticate the MIFARE Desfire tag in ISO CBS send mode using the key saved in the key storage
    @param key_in_storage Key number in key storage
    @param key_on_card Key number on card
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_AuthenticateIso(uint8_t key_in_storage, uint8_t key_on_card)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_AUTH_ISO;
    cmd[1] = key_in_storage;
    cmd[2] = key_on_card;


    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to authenticate the MIFARE Desfire tag in AES mode using the key saved in the key storage
    @param key_in_storage Key number in key storage
    @param key_on_card Key number on card
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_AuthenticateAes(uint8_t key_in_storage, uint8_t key_on_card)
{
    uint8_t cmd[5];

    cmd[0] = CMD_MFDF_AUTH_AES;
    cmd[1] = key_in_storage;
    cmd[2] = key_on_card;


    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to create application on the tag
    @param app_id 3-bytes application id
    @param key_settings1 Please  refer to the NXP documentation for more information
    @param key_settings2 Please  refer to the NXP documentation for more information
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CreateApp(uint8_t *app_id, uint8_t key_settings1, uint8_t key_settings2)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_CREATE_APP;
    cmd[1] = app_id[0];
    cmd[2] = app_id[1];
    cmd[3] = app_id[2];
    cmd[4] = key_settings1;
    cmd[5] = key_settings2;

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to delete an application from the tag
    @param app_id 3-bytes application id
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_DeleteApp(uint8_t *app_id)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_DELETE_APP;
    cmd[1] = app_id[0];
    cmd[2] = app_id[1];
    cmd[3] = app_id[2];

    binary_protocol->send(cmd, 4);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command tries to change the key for the selected application
    @param old_key old key number in key storage
    @param new_key new key number in key storage
    @param key_on_card key number on the card
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ChangeKey(uint8_t old_key, uint8_t new_key, uint8_t key_on_card)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_CHANGE_KEY;
    cmd[1] = old_key;
    cmd[2] = new_key;
    cmd[3] = key_on_card;
    
    binary_protocol->send(cmd, 4);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command gets the key settings bytes from the tag. This command does not require any arguments but an application must be selected and authorized.
    @param key_settings (out) 2-bytes key settings 
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_GetKeySettings(uint8_t *key_settings)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_GET_KEY_SETTINGS;
    
    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(key_settings, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command changes the key settings bytes for the selected and authorized application.
    @param key_settings 2-bytes new key settings 
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ChangeKeySettings(uint8_t *key_settings)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_CHANGE_KEY_SETTINGS;
    cmd[1] = key_settings[0];
    cmd[2] = key_settings[1];
    
    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command creates a file for the storage of plain unformatted user data within the selected application
    @param file_no File number inside the app
    @param access_rights 2 bytes. Please  refer to the NXP documentation for more information
    @param file_size max file size
    @param file_type 0-normal file, 1-backup file
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CreateDataFile(uint8_t file_no, uint8_t *access_rights, uint32_t file_size, uint8_t file_type)
{
    uint8_t cmd[8];

    cmd[0] = CMD_MFDF_CREATE_DATA_FILE;
    cmd[1] = file_no;
    cmd[2] = access_rights[0];
    cmd[3] = access_rights[1];
    
    cmd[4] = file_size & 0xff;
    cmd[5] = (file_size>> 8) & 0xff;
    cmd[6] = (file_size>> 16) & 0xff;

    cmd[7] = file_type;

    binary_protocol->send(cmd, 8);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command writes data to standard data files or backup data files
    @param file_no File number inside the app
    @param file_offset offset in the file
    @param data bytes to write
    @param len number of bytes to write (max 2048)
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_WriteData(uint8_t file_no, uint32_t file_offset, uint8_t *data, uint32_t len)
{
    uint8_t cmd[2048+20];

    if (len > 2048)
    {
        return 0x21;
    }

    cmd[0] = CMD_MFDF_WRITE_DATA;
    cmd[1] = file_no;

    cmd[2] = file_offset & 0xff;
    cmd[3] = (file_offset>> 8) & 0xff;
    cmd[4] = (file_offset>> 16) & 0xff;
    
    memcpy(&cmd[5],data, len);

    binary_protocol->send(cmd, 5+len);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command reads data from standard data files or backup data files
    @param file_no File number inside the app
    @param file_offset offset in the file
    @param data (out) bytes to read
    @param len number of bytes to read
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ReadData(uint8_t file_no, uint32_t file_offset, uint8_t *data, uint32_t len)
{
    uint8_t cmd[20];

    if (len > 2048)
    {
        return 0x21;
    }

    cmd[0] = CMD_MFDF_READ_DATA;
    cmd[1] = file_no;

    cmd[2] = file_offset & 0xff;
    cmd[3] = (file_offset>> 8) & 0xff;
    cmd[4] = (file_offset>> 16) & 0xff;
    
    cmd[5] = len & 0xff;
    cmd[6] = (len>> 8) & 0xff;
    cmd[7] = (len>> 16) & 0xff;    

    binary_protocol->send(cmd, 8);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(data, &binary_protocol->protocolBuffIn[2], len);

    return last_result;
}


/**
    @brief This command creates a file for the storage of plain unformatted user data within the selected application
    @param file_no File number inside the app
    @param access_rights 2 bytes. Please  refer to the NXP documentation for more information
    @param low_limit Low limit as 4-bytes signed value
    @param up_limit Up limit as 4-bytes signed value
    @param initial_velue Initial value as 4-bytes signed value
    @param get_free_enabled Please  refer to the NXP documentation for more information
    @param limit_credited Please  refer to the NXP documentation for more information
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CreateValueFile(uint8_t file_no, uint8_t *access_rights, int32_t low_limit, int32_t up_limit, int32_t initial_velue, uint8_t get_free_enabled, uint8_t limit_credited)
{
    uint8_t cmd[32];

    cmd[0] = CMD_MFDF_CREATE_VALUE_FILE;
    cmd[1] = file_no;
    cmd[2] = access_rights[0];
    cmd[3] = access_rights[1];
    
    cmd[4] = low_limit & 0xff;
    cmd[5] = (low_limit >> 8) & 0xff;
    cmd[6] = (low_limit >> 16) & 0xff;
    cmd[7] = (low_limit >> 24) & 0xff;

    cmd[8] = up_limit & 0xff;
    cmd[9] = (up_limit >> 8) & 0xff;
    cmd[10] = (up_limit >> 16) & 0xff;
    cmd[11] = (up_limit >> 24) & 0xff;

    cmd[12] = initial_velue & 0xff;
    cmd[13] = (initial_velue >> 8) & 0xff;
    cmd[14] = (initial_velue >> 16) & 0xff;
    cmd[15] = (initial_velue >> 24) & 0xff;

    cmd[16] = get_free_enabled;
    cmd[17] = limit_credited;

    binary_protocol->send(cmd, 18);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command returns the value stored in a value file on the TAG
    @param file_no File number inside the app
    @param value (out) pointer to output signet 32bit value
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_GetValue(uint8_t file_no, int32_t *value)
{
    uint8_t cmd[8];


    cmd[0] = CMD_MFDF_GET_VALUE;
    cmd[1] = file_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(cmd, &binary_protocol->protocolBuffIn[2], 4);

    *value = (int32_t)cmd[0];
    *value |= (int32_t)(cmd[1] << 8);
    *value |= (int32_t)(cmd[2] << 16);
    *value |= (int32_t)(cmd[3] << 24);

    return last_result;
}


/**
    @brief This command increases a value stored in a value file on the TAG. 
    @param file_no File number inside the app
    @param credit 4 bytes signed value, LSB first
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CreditFile(uint8_t file_no, int32_t credit)
{
    uint8_t cmd[8];


    cmd[0] = CMD_MFDF_CREDIT;
    cmd[1] = file_no;

    cmd[2] = credit & 0xff;
    cmd[3] = (credit >> 8) & 0xff;
    cmd[4] = (credit >> 16) & 0xff;
    cmd[5] = (credit >> 24) & 0xff;

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command decreases a value stored in a value file on the TAG. 
    @param file_no File number inside the app
    @param credit 4 bytes signed value, LSB first
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_DebitFile(uint8_t file_no, int32_t credit)
{
    uint8_t cmd[8];


    cmd[0] = CMD_MFDF_DEBIT;
    cmd[1] = file_no;

    cmd[2] = credit & 0xff;
    cmd[3] = (credit >> 8) & 0xff;
    cmd[4] = (credit >> 16) & 0xff;
    cmd[5] = (credit >> 24) & 0xff;

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command creates files for multiple storage of structurally similar data within an existing application
    @param file_no File number inside the app
    @param access_rights 2 bytes. Please  refer to the NXP documentation for more information
    @param record_size Record size, 16-bits LSB value
    @param number_of_records number of records, 16-bits LSB value
    @param cycling 0x00 - further writing is not possible unless it is cleared, 0x01 - the new record overwrites oldest record
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CreateRecordFile(uint8_t file_no, uint8_t *access_rights, uint16_t record_size, uint16_t number_of_records, uint8_t cycling)
{
    uint8_t cmd[32];

    cmd[0] = CMD_MFDF_CREATE_RECORD_FILE;
    cmd[1] = file_no;

    cmd[2] = access_rights[0];
    cmd[3] = access_rights[1];

    cmd[4] = record_size & 0xff;
    cmd[5] = (record_size >> 8) & 0xff;

    cmd[6] = number_of_records & 0xff;
    cmd[7] = (number_of_records >> 8) & 0xff;

    cmd[8] = cycling;

    binary_protocol->send(cmd, 9);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command writes data to a record file
    @param file_no File number inside the app
    @param data bytes to write
    @param data_len number of bytes to write in record
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_WriteRecord(uint8_t file_no, uint8_t *data, uint16_t data_len)
{
    uint8_t cmd[2048+20];

    cmd[0] = CMD_MFDF_WRITE_RECORD;
    cmd[1] = file_no;

    memcpy(&cmd[2], data, data_len);

    binary_protocol->send(cmd, data_len+2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command reads data from a record file
    @param file_no File number inside the app
    @param record_num record number
    @param data (out) output data pointer
    @param data_len number of bytes to write in record
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ReadRecord(uint8_t file_no, uint16_t record_num, uint16_t data_len, uint8_t *data)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_READ_RECORD;
    cmd[1] = file_no;
    cmd[2] = record_num & 0xff;
    cmd[3] = (record_num >> 8) & 0xff;

    cmd[4] = data_len & 0xff;
    cmd[5] = (data_len >> 8) & 0xff;

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(data, &binary_protocol->protocolBuffIn[2], data_len);

    return last_result;
}


/**
    @brief This command resets cyclic or lineal record files
    @param file_no File number inside the app
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_ClearRecords(uint8_t file_no)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_CLEAR_RECORDS;
    cmd[1] = file_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command permanently deactivates a file within the file directory of the currently selected application
    @param file_no File number inside the app
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_DeleteFile(uint8_t file_no)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_DELETE_FILE;
    cmd[1] = file_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}



/**
    @brief This command returns a value corresponding to the amount of free memory available on the TAG
    @param free_mem (out) free memory
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_GetFreeMem(uint32_t *free_mem)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_GET_FREEMEM;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(cmd, &binary_protocol->protocolBuffIn[2], 4);

    *free_mem = (int32_t)cmd[0];
    *free_mem |= (int32_t)(cmd[1] << 8);
    *free_mem |= (int32_t)(cmd[2] << 16);
    *free_mem |= (int32_t)(cmd[3] << 24);

    return last_result;
}


/**
    @brief This command returns a value corresponding to the amount of free memory available on the TAG
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_FormatMemory(void)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_FORMAT;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(1500);

    return last_result;
}


/**
    @brief This command validates all previous write access on backup data files
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_CommitTransaction(void)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_COMMIT_TRANSACTION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command invalidates all previous write access on backup data files
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_AbortTransaction(void)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_ABORT_TRANSACTION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command gets settings for the selected file. The format of the settings bytes depends on the file type.
    @param file_no File number inside the app
    @param file_settings (out) file settings bytes. Please refer C1 user manual for more informations about output format
    @param file_settings_len (out) file settings bytes count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_GetFileSettings(uint8_t file_no, uint8_t *file_settings, uint8_t *file_settings_len)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_GET_FILE_SETTINGS;
    cmd[1] = file_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(file_settings, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);
    
    *file_settings_len = binary_protocol->protocolLenIn-2;

    return last_result;
}



/**
    @brief This command sets new access rights for the selected file
    @param file_no File number inside the app
    @param access_rights 2 bytes. Please  refer to the NXP documentation for more information
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::MD_SetFileSettings(uint8_t file_no, uint8_t *access_rights)
{
    uint8_t cmd[10];

    cmd[0] = CMD_MFDF_SET_FILE_SETTINGS;
    cmd[1] = file_no;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(access_rights, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}

//--------------------------------------------------------------------------------------------------------


/**
    @brief This command starts the inventory procedure on ISO 15693 TAGs
    @param afi Application family identifier
    @param uid (out) Unique identifier, inverted order 8 bytes
    @param dsfid (out) Data Storage Format Identifier
    @param more_cards (out) 0 - no more cards in range of antenna, 1 – more cards in range of antenna
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_InventoryStart(uint8_t afi, uint8_t *uid, uint8_t *dsfid, uint8_t *more_cards)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_INVENTORY_START;
    cmd[1] = afi;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(uid, &binary_protocol->protocolBuffIn[2], 8);
    *dsfid = binary_protocol->protocolBuffIn[2+8];
    *more_cards = binary_protocol->protocolBuffIn[3+8];

    return last_result;
}


/**
    @brief This command should be used to continue the inventory procedure on ISO 15693 TAGs
    @param afi Application family identifier
    @param uid (out) Unique identifier, inverted order 8 bytes
    @param dsfid (out) Data Storage Format Identifier
    @param more_cards (out) 0 - no more cards in range of antenna, 1 – more cards in range of antenna
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_InventoryNext(uint8_t afi, uint8_t *uid, uint8_t *dsfid, uint8_t *more_cards)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_INVENTORY_NEXT;
    cmd[1] = afi;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(uid, &binary_protocol->protocolBuffIn[2], 8);
    *dsfid = binary_protocol->protocolBuffIn[2+8];
    *more_cards = binary_protocol->protocolBuffIn[3+8];

    return last_result;
}


/**
    @brief This command performs an ISO15693 Stay Quiet command to the  selected TAG
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_StayQuiet(void)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_STAY_QUIET;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief The read block command should be used to read data stored in TAG blocks
    @param block_numer first block number to read
    @param count Number of block to read
    @param data (out) output buffer, size must be >= 4*count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ReadBlock(uint8_t block_numer, uint8_t count, uint8_t *data)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_READ_BLOCK;
    cmd[1] = block_numer;
    cmd[2] = count;

    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(data, &binary_protocol->protocolBuffIn[2], count*4);

    return last_result;
}


/**
    @brief The write block command should be used to write data to the tag
    @param block_numer first block number to read
    @param count Number of block to read
    @param data (out) output buffer, size must be >= 4*count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_WriteBlock(uint8_t block_numer, uint8_t count, uint8_t *data)
{
    uint8_t cmd[2048+20];

    cmd[0] = CMD_ICODE_WRITE_BLOCK;
    cmd[1] = block_numer;
    cmd[2] = count;

    memcpy(&cmd[3], data, count*4);

    binary_protocol->send(cmd, 3+(count*4));
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs a lock block command
    @param block_numer block number to lock
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_LockBlock(uint8_t block_numer)
{
    uint8_t cmd[5];

    cmd[0] = CMD_ICODE_LOCK_BLOCK;
    cmd[1] = block_numer;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs a write to Application Family Identifier value inside the TAG memory. 
    @param block_numer block number to lock
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_WriteAfi(uint8_t afi)
{
    uint8_t cmd[5];

    cmd[0] = CMD_ICODE_WRITE_AFI;
    cmd[1] = afi;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs a Lock AFI command on the TAG
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_LockAfi(void)
{
    uint8_t cmd[5];

    cmd[0] = CMD_ICODE_LOCK_AFI;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs a write to Data Storage Format Identifier value inside the TAG memory
    @param dsfid block number to lock
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_WriteDsfid(uint8_t dsfid)
{
    uint8_t cmd[5];

    cmd[0] = CMD_ICODE_WRITE_DSFID;
    cmd[1] = dsfid;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs a Lock DSFID command on the TAG
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_LockDsfid(void)
{
    uint8_t cmd[5];

    cmd[0] = CMD_ICODE_LOCK_DSFID;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command performs get system information command on the TAG
    @param info (out) System information bytes
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_GetSystemInformation(uint8_t *info)
{
    uint8_t cmd[32];

    cmd[0] = CMD_ICODE_GET_SYSTEM_INFORMATION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(info, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command performs get multiple block security status command on the TAG
    @param block_numer first block number
    @param count Number of blocks
    @param bss_info (out) output buffer for bss information
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_GetMultipleBss(uint8_t block_number, uint8_t count, uint8_t *bss_info)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_GET_MULTIPLE_BSS;
    cmd[1] = block_number;
    cmd[2] = count;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(bss_info, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command enables the password protection for AFI
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_PasswordProtectAfi(void)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_PASSWORD_PROTECT_AFI;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command reads EPC data from the TAG
    @param epc_info (out) output buffer for bss information
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ReadEpc(uint8_t *epc_info)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_READ_EPC;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(epc_info, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command retrieves the NXP system information value from the TAG
    @param nxp_info (out) NXP system information bytes
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_GetNxpInformation(uint8_t *nxp_info)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_GET_NXP_SYSTEM_INFORMATION;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(nxp_info, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command requests a random number from the ICODE TAG, it is used for set password command
    @param number (out) 16bit random number
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_GetRandomNumber(uint16_t *number)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_GET_RANDOM_NUMBER;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(number, &binary_protocol->protocolBuffIn[2], 2);

    return last_result;
}


/**
    @brief This command sets the password for the selected identifier. Please read C1 manual for more infromation.
    @param id Password id. 0x01 – Read password, 0x02 – Write password, 0x04 – Privacy password, 0x08 – Destroy password
    @param xor_pwd The password is calculated as XOR  with the random number returned by the previously executed command
    @return 0 in case of success or error code

    XOR password should be calculated using this method:
    xor_pwd[0] = password[0] ^ rnd[0];
    xor_pwd[1] = password[1] ^ rnd[1];
    xor_pwd[2] = password[2] ^ rnd[0];
    xor_pwd[3] = password[3] ^ rnd[1];

*/
uint16_t C1_Interface::ICODE_SetPassword(uint8_t id, uint8_t *xor_pwd)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_SET_PASSWORD;
    cmd[1] = id;
    cmd[2] = xor_pwd[0];
    cmd[3] = xor_pwd[1];
    cmd[4] = xor_pwd[2];
    cmd[5] = xor_pwd[3];

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command writes a new password to a selected identifier. Please read C1 manual for more infromation.
    @param id Password id. 0x01 – Read password, 0x02 – Write password, 0x04 – Privacy password, 0x08 – Destroy password
    @param pwd The password is calculated as XOR  with the random number returned by the previously executed command
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_WritePassword(uint8_t id, uint8_t *pwd)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_WRITE_PASSWORD;
    cmd[1] = id;
    cmd[2] = pwd[0];
    cmd[3] = pwd[1];
    cmd[4] = pwd[2];
    cmd[5] = pwd[3];

    binary_protocol->send(cmd, 6);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command locks the addressed password
    @param id Password id. 0x01 – Read password, 0x02 – Write password, 0x04 – Privacy password, 0x08 – Destroy password
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_LockPassword(uint8_t id)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_LOCK_PASSWORD;
    cmd[1] = id;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command changes the protection status of a page
    @param id Page address to be protected in case of products that do not have pages characterized as high and Low
    @param status Protection status byte
    • Protection status options for the products that do not have pages characterized as high and Low:
            0x00: ICODE_PROTECT_PAGE_PUBLIC
            0x01: ICODE_PROTECT_PAGE_READ_WRITE_READ_PASSWORD
            0x10: ICODE_PROTECT_PAGE_WRITE_PASSWORD
            0x11: ICODE_PROTECT_PAGE_READ_WRITE_PASSWORD_SEPERATE
    • Extended Protection status options for the products that have pages characterized as high and Low:
            0x01: ICODE_PROTECT_PAGE_READ_LOW
            0x02: ICODE_PROTECT_PAGE_WRITE_LOW
            0x10: ICODE_PROTECT_PAGE_READ_HIGH
            0x20: ICODE_PROTECT_PAGE_WRITE_HIGH
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_PageProtect(uint8_t id, uint8_t status)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_PROTECT_PAGE;
    cmd[1] = id;
    cmd[2] = status;

    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command permanently locks the protection status of a page
    @param id Page address
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_LockPageProtect(uint8_t id)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_LOCK_PAGE_PROTECTION;
    cmd[1] = id;

    binary_protocol->send(cmd, 2);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This instructs the label to return the block protection status of the requested blocks
    @param block_numer first block number
    @param count Number of blocks
    @param bps_info (out) output buffer for block protection status bytes
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_GetMultipleBps(uint8_t block_number, uint8_t count, uint8_t *bps_info)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_GET_MULTIPLE_BPS;
    cmd[1] = block_number;
    cmd[2] = count;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(bps_info, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}


/**
    @brief This command permanently destroys the label (tag)
    @param xor_pwd optional XOR password, if not used please set to NULL
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_Destroy(uint8_t *xor_pwd)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_DESTROY;
    if (xor_pwd)
    {
        memcpy(&cmd[1], xor_pwd, 4);
        binary_protocol->send(cmd, 5);
    }
    else
    {
        binary_protocol->send(cmd, 1);
    }

    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command instructs the label to enter privacy mode. In privacy mode, 
    the label will only respond to ICODE_GET_RANDOM_NUMBER and ICODE_SET_PASSWORD 
    commands. To get out of the privacy mode, the Privacy password has to be transmitted 
    before with ICODE_SET_PASSWORD.
    @param xor_pwd optional XOR password, if not used please set to NULL
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_EnablePrivacy(uint8_t *xor_pwd)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_ENABLE_PRIVACY;
    if (xor_pwd)
    {
        memcpy(&cmd[1], xor_pwd, 4);
        binary_protocol->send(cmd, 5);
    }
    else
    {
        binary_protocol->send(cmd, 1);
    }

    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This instructs the label that both  Read and Write passwords are required for protected access
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_Enable64Password(void)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_ENABLE_64BIT_PASSWORD;
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command reads the signature bytes from the TAG
    @param sig (out) buffer for signature bytes
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ReadSignature(uint8_t *sig)
{
    uint8_t cmd[10];

    cmd[0] = CMD_ICODE_READ_SIGNATURE;
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(sig, &binary_protocol->protocolBuffIn[2], binary_protocol->protocolLenIn-2);

    return last_result;
}



/**
    @brief The extended read block command should be used to read data stored in TAG blocks but only if the tag supports this command – if you are not sure please use ICODE_READ_BLOCK command
    @param block_numer first block number to read
    @param count Number of block to read
    @param data (out) output buffer, size must be >= 4*count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ExtReadBlock(uint16_t block_numer, uint8_t count, uint8_t *data)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_EXT_READ_BLOCK;
    cmd[1] = block_numer & 0xff;
    cmd[2] = (block_numer >> 8) & 0xff;
    cmd[3] = count;

    binary_protocol->send(cmd, 4);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(data, &binary_protocol->protocolBuffIn[2], count*4);

    return last_result;
}


/**
    @brief The extended write block command should be used to write data to the tag but only if the tag supports this command – if you are not sure please use ICODE_WRITE_BLOCK command
    @param block_numer first block number to read
    @param count Number of block to read
    @param data (out) output buffer, size must be >= 4*count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ExtWriteBlock(uint16_t block_numer, uint8_t count, uint8_t *data)
{
    uint8_t cmd[2048+20];

    cmd[0] = CMD_ICODE_EXT_WRITE_BLOCK;
    cmd[1] = block_numer & 0xff;
    cmd[2] = (block_numer >> 8) & 0xff;
    cmd[3] = count;

    memcpy(&cmd[4], data, count*4);

    binary_protocol->send(cmd, 4+(count*4));
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command reads multiple 4-byte data chunks from the selected configuration block address
    @param block_numer first block number to read
    @param count Number of block to read
    @param data (out) output buffer, size must be >= 4*count
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_ReadConfig(uint8_t block_numer, uint8_t count, uint8_t *data)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_READ_CONFIG;
    cmd[1] = block_numer;
    cmd[2] = count;

    binary_protocol->send(cmd, 3);
    ///wait for ACK and return status
    wait4Answer(500);

    memcpy(data, &binary_protocol->protocolBuffIn[2], count*4);

    return last_result;
}


/**
    @brief This command writes configuration bytes to addressed block data from the selected configuration block address
    @param option 1 – Enable option, 0 – Disable option
    @param block_numer block number
    @param config configuration 4-bytes 
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_WriteConfig(uint8_t option, uint8_t block_numer, uint8_t *config)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_WRITE_CONFIG;
    cmd[1] = option;
    cmd[2] = block_numer;
    cmd[3] = config[0];
    cmd[4] = config[1];
    cmd[5] = config[2];
    cmd[6] = config[3];

    binary_protocol->send(cmd, 7);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}


/**
    @brief This command enables the random ID generation in the tag
    @return 0 in case of success or error code
*/
uint16_t C1_Interface::ICODE_PickRandomId(void)
{
    uint8_t cmd[16];

    cmd[0] = CMD_ICODE_PICK_RANDOM_ID;

    binary_protocol->send(cmd, 1);
    ///wait for ACK and return status
    wait4Answer(500);

    return last_result;
}