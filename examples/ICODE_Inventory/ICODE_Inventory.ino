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
  SerialPort2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
}

void loop() {
  uint8_t count, type, param, uid[8], uid_len;
  char sbuff[128];
  uint8_t more, dsfid;
  uint16_t res;
  bool start = true;

  if (interface.SendDummyCommand() == 0xff)
  {
    SerialPort.println("No answer from the Pepper C1 - please check connection/cables");
    sleep(3);
    return;
  }
  SerialPort.println("Communication OK");

  do {
    if (start)
    {
      if (interface.ICODE_InventoryStart(0,uid, &dsfid, &more) != 0)
      {
        SerialPort.println("No tag found");
        break;
      }
      start = false;
    }
    else
    {
      if (interface.ICODE_InventoryNext(0,uid, &dsfid, &more) != 0)
      {
        break;        
      }
    }

    SerialPort.print("UID: ");
    for (int k= 0; k < 7; k++)
    {
      SerialPort.printf(" %02X", uid[k]);
    }
    SerialPort.println("");
    if (more == 0)
    {
      break;
    }
  } while(more);

  interface.Halt(); //turn of RF field
  SerialPort.println("Next test in 3 seconds...");
  sleep(3);
}

