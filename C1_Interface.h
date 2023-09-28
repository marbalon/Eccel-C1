/*
  C1_Interface.h - this file is part of the Pepper C1 
  library for Pepper C1 products by Eccel Technology Ltd.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

   Authors:  Marcin Baliniak

   Version: 1.0

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef C1LIBRARY_H_INCLUDED
#define C1LIBRARY_H_INCLUDED
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "binary_protocol.h"

#define KEY_TYPE_AES128         0x00U   /**< AES 128 Key [16]. */
#define KEY_TYPE_AES192         0x01U   /**< AES 192 Key [24]. */
#define KEY_TYPE_AES256         0x02U   /**< AES 256 Key [32]. */
#define KEY_TYPE_DES            0x03U   /**< DES Single Key.   */
#define KEY_TYPE_2K3DES         0x04U   /**< 2 Key Triple Des. */
#define KEY_TYPE_3K3DES         0x05U   /**< 3 Key Triple Des. */
#define KEY_TYPE_MIFARE         0x06U   /**< MIFARE (R) Key. */

enum
{
	CMD_ACK = 0x00,
	CMD_DUMMY_COMMAND,
	CMD_GET_TAG_COUNT,
	CMD_GET_UID,
	CMD_ACTIVATE_TAG,
	CMD_HALT,
	CMD_SET_POLLING,
	CMD_SET_KEY,
	CMD_SAVE_KEYS,
	CMD_SET_NET_CFG,
	CMD_REBOOT,
	CMD_GET_VERSION,
	CMD_UART_PASSTHRU,
	CMD_SLEEP,
	CMD_GPIO,
	CMD_ANTENNA,
	CMD_BT_PIN,
	CMD_FACTORY_RESET,
	CMD_PROTO_AUTH,
	CMD_PROTO_CONFIG,
	CMD_LED,

	// mifare clasics commands
	CMD_MF_READ_BLOCK = 0x20,
	CMD_MF_WRITE_BLOCK,
	CMD_MF_READ_VALUE,
	CMD_MF_WRITE_VALUE,
	CMD_MF_INCREMENT,
	CMD_MF_TRANSFER,
	CMD_MF_RESTORE,
	CMD_MF_TRANSFER_RESTORE,

	// mifare ultralight
	CMD_MFU_READ_PAGE = 0x40,
	CMD_MFU_WRITE_PAGE,
	CMD_MFU_GET_VERSION,
	CMD_MFU_READ_SIG,
	CMD_MFU_WRITE_SIG,
	CMD_MFU_LOCK_SIG,
	CMD_MFU_READ_COUNTER,
	CMD_MFU_INCREMENT_COUNTER,
	CMD_MFU_PASSWD_AUTH,
	CMD_MFUC_AUTHENTICATE,
	CMD_MFU_CHECKEVENT,

	CMD_MFDF_GET_VERSION = 0x60,
	CMD_MFDF_SELECT_APP,
	CMD_MFDF_APP_IDS,
	CMD_MFDF_FILE_IDS,
	CMD_MFDF_AUTH,
	CMD_MFDF_AUTH_ISO,
	CMD_MFDF_AUTH_AES,
	CMD_MFDF_CREATE_APP,
	CMD_MFDF_DELETE_APP,
	CMD_MFDF_CHANGE_KEY,
	CMD_MFDF_GET_KEY_SETTINGS,
	CMD_MFDF_CHANGE_KEY_SETTINGS,
	CMD_MFDF_CREATE_DATA_FILE,
	CMD_MFDF_WRITE_DATA,
	CMD_MFDF_READ_DATA,
	CMD_MFDF_CREATE_VALUE_FILE,
	CMD_MFDF_GET_VALUE,
	CMD_MFDF_CREDIT,
	CMD_MFDF_LIMITED_CREDIT,
	CMD_MFDF_DEBIT,
	CMD_MFDF_CREATE_RECORD_FILE,
	CMD_MFDF_WRITE_RECORD,
	CMD_MFDF_READ_RECORD,
	CMD_MFDF_CLEAR_RECORDS,
	CMD_MFDF_DELETE_FILE,
	CMD_MFDF_GET_FREEMEM,
	CMD_MFDF_FORMAT,
	CMD_MFDF_COMMIT_TRANSACTION,
	CMD_MFDF_ABORT_TRANSACTION,
	CMD_MFDF_GET_FILE_SETTINGS,
	CMD_MFDF_SET_FILE_SETTINGS,

	CMD_ICODE_INVENTORY_START = 0x90,
	CMD_ICODE_INVENTORY_NEXT,
	CMD_ICODE_STAY_QUIET,
	CMD_ICODE_READ_BLOCK,
	CMD_ICODE_WRITE_BLOCK,
	CMD_ICODE_LOCK_BLOCK,
	CMD_ICODE_WRITE_AFI,
	CMD_ICODE_LOCK_AFI,
	CMD_ICODE_WRITE_DSFID,
	CMD_ICODE_LOCK_DSFID,
	CMD_ICODE_GET_SYSTEM_INFORMATION,
	CMD_ICODE_GET_MULTIPLE_BSS,
	CMD_ICODE_PASSWORD_PROTECT_AFI,
	CMD_ICODE_READ_EPC,
	CMD_ICODE_GET_NXP_SYSTEM_INFORMATION,
	CMD_ICODE_GET_RANDOM_NUMBER,
	CMD_ICODE_SET_PASSWORD,
	CMD_ICODE_WRITE_PASSWORD,
	CMD_ICODE_LOCK_PASSWORD,
	CMD_ICODE_PROTECT_PAGE,
	CMD_ICODE_LOCK_PAGE_PROTECTION,
	CMD_ICODE_GET_MULTIPLE_BPS,
	CMD_ICODE_DESTROY,
	CMD_ICODE_ENABLE_PRIVACY,
	CMD_ICODE_ENABLE_64BIT_PASSWORD,
	CMD_ICODE_READ_SIGNATURE,
	CMD_ICODE_READ_CONFIG,
	CMD_ICODE_WRITE_CONFIG,
	CMD_ICODE_PICK_RANDOM_ID,

	CMD_ICODE_SET_EAS,
	CMD_ICODE_RESET_EAS,
	CMD_ICODE_LOCK_EAS,
	CMD_ICODE_EAS_ALARM,
	CMD_ICODE_PASSWORD_PROTECT_EAS,
	CMD_ICODE_WRITE_EASID,
	CMD_ICODE_EXT_READ_BLOCK,
	CMD_ICODE_EXT_WRITE_BLOCK,

	CMD_FOPEN = 0xE0,
	CMD_FREAD,
	CMD_FWRITE,
	CMD_FCLOSE,

	CMD_OTA_BEGIN = 0xF0,
	CMD_OTA_FRAME,
	CMD_OTA_FINISH,
	CMD_PN_EEPROM,
	CMD_TEST_MODE,
	CMD_PN_REG,

	CMD_BLE_UID = 0xFD,
	CMD_ASYNC = 0xFE,
	CMD_ERROR = 0xFF,
} cmd_binary_num;

/*

0x00 - command succesfull

Error answers

0x1980 - MF DF Response - No changes done to backup files
0x1981 - MF DF Response - Insufficient NV-Memory
0x1982 - MF DF Invalid key number specified
0x1983 - MF DF Current configuration/status does not allow the requested command
0x1984 - MF DF Requested AID not found on PICC
0x1985 - MF DF Attempt to read/write data from/to beyond the files/record's limits
0x1986 - MF DF Previous cmd not fully completed. Not all frames were requested or provided by the PCD
0x1987 - MF DF Num. of applns limited to 28. No additional applications possible
0x1988 - MF DF File/Application with same number already exists
0x1989 - MF DF Specified file number does not exist
0x198A - MF DF Crypto error returned by PICC
0x198B - MF DF Parameter value error returned by PICC
0x198C - MF DF DesFire Generic error. Check additional Info
0x198D - MF DF ISO 7816 Generic error. Check Additional Info

ICODE specific errors – layer byte 0x15

Error byte:
0x1501 - The command is not supported, i.e. the request code is not recognized
0x1502 - The command is not recognized, for example: a format error occurred
0x1503 - The command option is not supported
0x150F - Error with no information given or a specific error code is not supported
0x1510 - The specified block is not available (doesn't exist)
0x1511 - The specified block is already locked and thus cannot be locked again
0x1512 - The specified block is locked and its content cannot be changed
0x1513 - The specified block was not successfully programmed
0x1514 - The specified block was not successfully locked
0x1515 - The specified block is protected
0x1540 - Generic cryptographic error
0x1581 - The command is not supported, i.e. the request code is not recognized
0x1582 - The command is not recognized, for example: a format error occurred
0x1583 - The command option is not supported
0x1584 - Error with no information given or a specific error code is not supported
0x1585 - The specified block is not available (doesn't exist)
0x1586 - The specified block is already locked and thus cannot be locked again
0x1587 - The specified block is locked and its content cannot be changed
0x1588 - The specified block was not successfully programmed
0x1589 - The specified block was not successfully locked
0x158A - The specified block is protected
0x158B - Generic cryptographic error

Other layers errors:
0x01 - No reply received, e.g. PICC removal
0x02 - Wrong CRC or parity detected
0x03 - A collision occurred
0x04 - Attempt to write beyond buffer size
0x05 - Invalid frame format
0x06 - Received response violates protocol
0x07 - Authentication error
0x08 - A Read or Write error occurred in RAM/ROM or Flash
0x09 - The RC sensors signal over heating
0x0A - Error due to RF.
0x0B - An error occurred in RC communication
0x0C - A length error occurred
0x0D - An resource error
0x0E - TX Rejected sanely by the counterpart
0x0F - RX request Rejected sanely by the counterpart
0x10 - Error due to External RF
0x11 - EMVCo EMD Noise Error
0x12 - Used when HAL ShutDown is called
0x20 - Invalid data parameters supplied (layer id check failed)
0x21 - Invalid parameter supplied
0x22 - Reading/Writing a parameter would produce an overflow.
0x23 - Parameter not supported
0x24 - Command not supported
0x25 - Condition of use not satisfied
0x26 - A key error occurred
0x7F - An internal error occurred
0xF0 – Protocol authorization error. This command is not allowed without protocol authorization (Command 0x12)

0xFF – No answer from the device
*/

class C1_Interface {

    public:
        C1_Interface(Stream &serial);
		//generic commands
        uint16_t SendDummyCommand();
		uint16_t GetTagCount(uint8_t *count);
		uint16_t GetUid(uint8_t idx, uint8_t *type, uint8_t *param, uint8_t *uid, uint8_t *uid_len);
		uint16_t ActivateTag(uint8_t idx);
		uint16_t Halt(void);
		uint16_t SetPolling(uint8_t on);
		uint16_t SetKey(uint8_t key_no, uint8_t key_type, uint8_t *key);
		uint16_t SaveKeys(void);
		uint16_t Reboot(void);
		uint16_t GetVersion(char *version);
		uint16_t Sleep(void);
		uint16_t FactoryReset(void);
		uint16_t SetLed(uint8_t r, uint8_t g, uint8_t b, uint16_t timeout);

		//Mifare Classic commands
		uint16_t MC_ReadBlock(uint8_t block_no, uint8_t block_count, uint8_t key_type, uint8_t key_no, uint8_t *buff);
		uint16_t MC_WriteBlock(uint8_t block_no, uint8_t block_count, uint8_t key_type, uint8_t key_no, uint8_t *buff);	
		uint16_t MC_ReadValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, uint32_t *value);
		uint16_t MC_WriteValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, uint32_t value);
		uint16_t MC_IncrementValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, int32_t value);
		uint16_t MC_DecrementValue(uint8_t block_no, uint8_t key_type, uint8_t key_no, int32_t value);

		//Mifare Ultralight commands
		uint16_t MU_ReadPage(uint8_t page_no, uint8_t page_count, uint8_t *buff);
		uint16_t MU_WritePage(uint8_t page_no, uint8_t page_count, uint8_t *buff);
		uint16_t MU_GetVersion(uint8_t *buff);
		uint16_t MU_ReadSignature(uint8_t *buff);
		uint16_t MU_WriteSignature(uint8_t *buff);
		uint16_t MU_LockSignature(uint8_t lock_byte);
		uint16_t MU_ReadCounter(uint8_t counter_no, uint8_t *counter);
		uint16_t MU_IncrementCounter(uint8_t counter_no, uint8_t *value);
		uint16_t MU_PasswordAuth(uint8_t *pass);
		uint16_t MU_UlcAuth(uint8_t key_no);
		uint16_t MU_CheckTearingEvent(uint8_t counter_no, uint8_t *event);

		//Mifare Desfire commands
		uint16_t MD_GetVersion(uint8_t *buff);
		uint16_t MD_SelectApp(uint8_t *aid);
		uint16_t MD_ListApps(uint8_t *apps, uint8_t *count);
		uint16_t MD_ListFiles(uint8_t *files, uint8_t *count);
		uint16_t MD_Authenticate(uint8_t key_in_storage, uint8_t key_on_card);
		uint16_t MD_AuthenticateIso(uint8_t key_in_storage, uint8_t key_on_card);
		uint16_t MD_AuthenticateAes(uint8_t key_in_storage, uint8_t key_on_card);
		uint16_t MD_CreateApp(uint8_t *app_id, uint8_t key_settings1, uint8_t key_settings2);
		uint16_t MD_DeleteApp(uint8_t *app_id);
		uint16_t MD_ChangeKey(uint8_t old_key, uint8_t new_key, uint8_t key_on_card);
		uint16_t MD_GetKeySettings(uint8_t *key_settings);
	    uint16_t MD_ChangeKeySettings(uint8_t *key_settings);
		uint16_t MD_CreateDataFile(uint8_t file_no, uint8_t *access_rights, uint32_t file_size, uint8_t file_type);
		uint16_t MD_WriteData(uint8_t file_no, uint32_t file_offset, uint8_t *data, uint32_t len);
		uint16_t MD_ReadData(uint8_t file_no, uint32_t file_offset, uint8_t *data, uint32_t len);
		uint16_t MD_CreateValueFile(uint8_t file_no, uint8_t *access_rights, int32_t low_limit, int32_t up_limit, int32_t initial_velue, uint8_t get_free_enabled, uint8_t limit_credited);
		uint16_t MD_GetValue(uint8_t file_no, int32_t *value);
		uint16_t MD_CreditFile(uint8_t file_no, int32_t credit);
		uint16_t MD_DebitFile(uint8_t file_no, int32_t credit);
		uint16_t MD_CreateRecordFile(uint8_t file_no, uint8_t *access_rights, uint16_t record_size, uint16_t number_of_records, uint8_t cycling);
		uint16_t MD_WriteRecord(uint8_t file_no, uint8_t *data, uint16_t data_len);
		uint16_t MD_ReadRecord(uint8_t file_no, uint16_t record_num, uint16_t data_len, uint8_t *data);
		uint16_t MD_ClearRecords(uint8_t file_no);
		uint16_t MD_DeleteFile(uint8_t file_no);
		uint16_t MD_GetFreeMem(uint32_t *free_mem);
		uint16_t MD_FormatMemory(void);
		uint16_t MD_CommitTransaction(void);
		uint16_t MD_AbortTransaction(void);
		uint16_t MD_GetFileSettings(uint8_t file_no, uint8_t *file_settings, uint8_t *file_settings_len);
		uint16_t MD_SetFileSettings(uint8_t file_no, uint8_t *access_rights);

		//ICODE commands
		uint16_t ICODE_InventoryStart(uint8_t afi, uint8_t *uid, uint8_t *dsfid, uint8_t *more_cards);
		uint16_t ICODE_InventoryNext(uint8_t afi, uint8_t *uid, uint8_t *dsfid, uint8_t *more_cards);
		uint16_t ICODE_StayQuiet(void);
		uint16_t ICODE_ReadBlock(uint8_t block_numer, uint8_t count, uint8_t *data);
		uint16_t ICODE_WriteBlock(uint8_t block_numer, uint8_t count, uint8_t *data);
		uint16_t ICODE_LockBlock(uint8_t block_numer);
		uint16_t ICODE_WriteAfi(uint8_t afi);
		uint16_t ICODE_LockAfi(void);
		uint16_t ICODE_WriteDsfid(uint8_t dsfid);
		uint16_t ICODE_LockDsfid(void);
		uint16_t ICODE_GetSystemInformation(uint8_t *info);
		uint16_t ICODE_GetMultipleBss(uint8_t block_number, uint8_t count, uint8_t *bss_info);
		uint16_t ICODE_PasswordProtectAfi(void);
		uint16_t ICODE_ReadEpc(uint8_t *epc_info);
		uint16_t ICODE_GetNxpInformation(uint8_t *nxp_info);
		uint16_t ICODE_GetRandomNumber(uint16_t *number);
		uint16_t ICODE_SetPassword(uint8_t id, uint8_t *xor_pwd);
		uint16_t ICODE_WritePassword(uint8_t id, uint8_t *pwd);
		uint16_t ICODE_LockPassword(uint8_t id);
		uint16_t ICODE_PageProtect(uint8_t id, uint8_t status);
		uint16_t ICODE_LockPageProtect(uint8_t id);
		uint16_t ICODE_GetMultipleBps(uint8_t block_number, uint8_t count, uint8_t *bps_info);
		uint16_t ICODE_Destroy(uint8_t *xor_pwd);
		uint16_t ICODE_EnablePrivacy(uint8_t *xor_pwd);
		uint16_t ICODE_Enable64Password(void);
		uint16_t ICODE_ReadSignature(uint8_t *sig);
		uint16_t ICODE_ExtReadBlock(uint16_t block_numer, uint8_t count, uint8_t *data);
		uint16_t ICODE_ExtWriteBlock(uint16_t block_numer, uint8_t count, uint8_t *data);
		uint16_t ICODE_ReadConfig(uint8_t block_numer, uint8_t count, uint8_t *data);
		uint16_t ICODE_WriteConfig(uint8_t option, uint8_t block_numer, uint8_t *config);
		uint16_t ICODE_PickRandomId(void);
    private:
        //uint16_t SendPacket(uint16_t size);
        BinaryProtocol *binary_protocol;
		bool wait4Answer(uint16_t timeout);
};

#endif
