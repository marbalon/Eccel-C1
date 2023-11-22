#include <Arduino.h>
#include <HardwareSerial.h>
#include "C1_Interface.h"

#define RXD1 3
#define TXD1 1
#define RXD2 16
#define TXD2 17

HardwareSerial SerialPort(0); // logs
HardwareSerial SerialPort2(2); // C1 connection

C1_Interface interface(SerialPort2);

void ckeck_status(uint16_t status)
{
  switch (status)
  {
    case 0: //success
    break;
    case 0xff:
      SerialPort.println("Timeout/No answer from the Pepper C1 - please check connection/cables");
    break;
    default:
      SerialPort.printf("Error received: 0x%04X\n", status);
    break;
  }
}

void setup()  
{
  SerialPort.begin(115200, SERIAL_8N1, RXD1, TXD1); 
  SerialPort2.begin(115200, SERIAL_8N1, RXD2, TXD2)
}

void loop() {
  uint8_t count, type, param, uid[8], uid_len;
  char sbuff[128];
  uint8_t ubuff[128];
  uint16_t res;

  if (interface.SendDummyCommand() == 0xff)
  {
    SerialPort.println("No answer from the Pepper C1 - please check connection/cables");
    return;
  }
  SerialPort.println("Communication OK");

  ckeck_status(interface.GetTagCount(&count));
  SerialPort.printf("Get tag count: %d\n", count);
  if (count > 0)
  do
  {
      interface.GetUid(0, &type, &param, uid, &uid_len);
      SerialPort.printf("UID (type 0x%02X):", type);
      for (uint8_t k= 0; k < uid_len; k++)
      {
        SerialPort.printf(" %02X", uid[k]);
      }
      SerialPort.println("");

      switch (type)
      {
        case 0x0D:
          SerialPort.println("MIFARE Desfire");
          break;
        default:
          SerialPort.println("This is not a Mifare Desfire TAG, test not executed.");
          interface.Halt(); //turn of RF field
          sleep(3);
          return;
      }

      //set key in the key storage as 0x00...
      memset(ubuff, 0, 16);
      if ((res=interface.SetKey(0, KEY_TYPE_DES, ubuff)) != 0)
      {
        SerialPort.printf("Set key command failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Setting key 0 - OK\r\n", res);
      }
      
      //select default app 0x00 0x00 0x00
      if ((res=interface.MD_SelectApp(ubuff)) != 0) 
      {
        SerialPort.printf("Selecting default app failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Default app selected\r\n", res);
      }

      //auth default app
      if ((res=interface.MD_Authenticate(0, 0)) != 0)
      {
        SerialPort.printf("Auth default app failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Default app authorised\r\n", res);
      }

      if ((res=interface.MD_FormatMemory()) != 0)
      {
        SerialPort.printf("Formatting card failed - 0x%04X\r\n", res);
      }
      else{
        SerialPort.printf("Formatting card OK\r\n");
      } 

      //create example app
      ubuff[0] = 0xAA;
      ubuff[1] = 0x55;
      ubuff[2] = 0xAA;

      if ((res=interface.MD_CreateApp(ubuff, 0xED, 0x84)) != 0)
      {
        SerialPort.printf("Exmaple app not created - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Example app created\r\n");
      }      

      if ((res=interface.MD_SelectApp(ubuff)) != 0)
      {
        SerialPort.printf("Select exmaple app failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Select example app OK\r\n");
      }      

      //create file
      ubuff[0] = 0xEE;
      ubuff[1] = 0xEE;

      //for backup file please use commit transaction below
      if ((res=interface.MD_CreateDataFile(0x01, ubuff, 256, 0)) != 0)
      {
        SerialPort.printf("Create data file failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Data file 0x01 with 256 bytes created\r\n");
      }

      sprintf(sbuff, "Hello world! Desfire example for Eccel Peper C1");

      if ((res=interface.MD_WriteData(0x01, 0, (uint8_t*)sbuff, strlen(sbuff)) ) != 0)
      {
        SerialPort.printf("Write data file failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Data written to the file\r\n");
      }

      /*if ((res=interface.MD_CommitTransaction() ) != 0)
      {
        SerialPort.printf("Commit data failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Commit to the file\r\n");
      }*/

      memset(sbuff, 0, sizeof(sbuff));
      if ((res=interface.MD_ReadData(0x01, 0, (uint8_t*)sbuff, 50)) != 0)
      {
        SerialPort.printf("Read data file failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Data from file: %s\r\n", sbuff);
      }

      //create value file
      ubuff[0] = 0xEE;
      ubuff[1] = 0xEE;

      if ((res=interface.MD_CreateValueFile(0x02, ubuff, 0, 100, 0, 1, 1)) != 0)
      {
        SerialPort.printf("Create value file failed - 0x%04X\r\n", res);
        break;
      }
      else{
        SerialPort.printf("Value file id 0x02 with 100 creadit created\r\n");
      }

      //trying to credit file above the limit, last credit by 10 should not be succesful
      for (int k=0; k < 11; k++)
      {
        int32_t val;
        if ((res=interface.MD_CreditFile(0x02, 10)) != 0)
        {
          SerialPort.printf("Credit file by 10 failed - 0x%04X\r\n", res);
          break;          
        }
        else{
          SerialPort.printf("Credit file by 10 OK\r\n");
        }

        if ((res=interface.MD_CommitTransaction() ) != 0)
        {
          SerialPort.printf("Commit data failed - 0x%04X\r\n", res);
          break;
        }
        else{
          SerialPort.printf("Commit to the file\r\n");
        }

        if ((res=interface.MD_GetValue(0x02, &val)) != 0)
        {
          SerialPort.printf("Get file value failed - 0x%04X\r\n", res);
          break;
        }
        else{
          SerialPort.printf("Credit one the card - %d\r\n", val);
        }
      }

  } while (0); //just to use break in case of errors


  //select default app 0x00 0x00 0x00
  memset(ubuff, 0, 3);
  if ((res=interface.MD_SelectApp(ubuff)) != 0) 
  {
    SerialPort.printf("Selecting default app failed - 0x%04X\r\n", res);
  }
  else{
    SerialPort.printf("Default app selected\r\n", res);
  }

  //auth default app
  if ((res=interface.MD_Authenticate(0, 0)) != 0)
  {
    SerialPort.printf("Auth default app failed - 0x%04X\r\n", res);
  }
  else{
    SerialPort.printf("Default app authorised\r\n", res);
  }

  if ((res=interface.MD_FormatMemory()) != 0)
  {
    SerialPort.printf("Formatting card failed - 0x%04X\r\n", res);
  }
  else{
    SerialPort.printf("Formatting card OK\r\n");
  } 

  interface.Halt(); //turn of RF field
  SerialPort.println("Next test in 3 seconds...");
  sleep(3);
}

