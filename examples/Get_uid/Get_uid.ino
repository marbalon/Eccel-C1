#include <Arduino.h>
#include <HardwareSerial.h>
#include "C1_Interface.h"

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
  SerialPort.begin(115200, SERIAL_8N1, 3, 1); 
  SerialPort2.begin(115200, SERIAL_8N1, 17, 16); 
}

void loop() {
  uint8_t count, type, param, uid[8], uid_len;
  char sbuff[128];
  uint8_t ubuff[128];

  if (interface.SendDummyCommand() == 0xff)
  {
    SerialPort.println("No answer from the Pepper C1 - please check connection/cables");
    return;
  }
  SerialPort.println("Communication OK");

  interface.GetVersion(sbuff);
  SerialPort.printf("Fimrware version is: %s\n", sbuff);

  ckeck_status(interface.GetTagCount(&count));
  SerialPort.printf("Get tag count: %d\n", count);
  if (count > 0)
  {
    for (uint8_t idx = 0; idx < count; idx++)
    {
      interface.GetUid(idx, &type, &param, uid, &uid_len);
      SerialPort.printf("UID %d:", idx);
      for (uint8_t k= 0; k < uid_len; k++)
      {
        SerialPort.printf(" %02X", uid[k]);
      }
      SerialPort.println("");
    }
  }
  interface.SetPolling(1);
  SerialPort.println("Internal polling enabled again. Next test in 3 seconds...");
  sleep(3);
}

