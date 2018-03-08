#include <SPI.h>
#include <C:\Users\chris\Documents\Arduino\libraries\NRFLite\src\NRFLite.h>
#include <C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\EEPROM\src\EEPROM.h>

#pragma region Globals
const static uint8_t RADIO_ID = 2;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 0; // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 8;
const static uint8_t PIN_RADIO_CSN = 10;

const byte STATUS_LED = 2;

const byte COMMAND  = B00000001;
const byte STATE    = B00000010;
const byte PIN      = B00000100;
const byte MODE     = B00001000;
const byte TIME     = B00010000;
const byte KEEP_HIGH    = B00100000;
const byte KEEP_LOW = B10000010;
const byte TOGGLE   = B01000000;
const byte SET_OUTPUTS = B10000000;
const byte GET_INPUTS = B10000001;
const byte HEARTBEAT = B10000100;
const byte INPUTS = B10010000;
const byte DATA      = B10001000;
const byte NEW_ID      = B10100000;
const byte RELAY    = B11000000;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t FromRadioId;
    char Payload[28];
};

struct OutputPinState
{
    int pinNumber;
    byte state;
    int duration;
    bool mode;
};

NRFLite _radio;
RadioPacket _radioData;
unsigned long prevMillis;
bool bufferReady = false;
int failcount = 0;
bool input6, input7 = false;
String inputArray[10];
OutputPinState outputPins;
uint8_t relayNode = 0;

#pragma endregion
void printBits(byte myByte){
 for(byte mask = 0x80; mask; mask >>= 1){
   if(mask  & myByte)
       Serial.print('1');
   else
       Serial.print('0');
 }
 Serial.println();
}

#pragma region EEPROM


bool writeDeviceID(uint8_t deviceID)
{
    _radioData.FromRadioId = deviceID;
    EEPROM.update(0, deviceID);
    initialiseRadio();
    return deviceID;
}

char readDeviceID()
{
    return EEPROM.read(0);
}

#pragma endregion
#pragma region Messages

void getRadioID()
{
    _radioData.Payload[0] = NEW_ID;
    sendRadioPacket(DESTINATION_RADIO_ID);
}

void announce()
{
    _radioData.Payload[0] = HEARTBEAT;
    bufferReady = true;
}

#pragma endregion
#pragma region hardware
void sendRadioPacket(int destination)
{
    //Serial.print("Sending to: ");
    //Serial.println(destination);
    //Serial.println(_radioData.Payload);
    if (_radio.send(destination, &_radioData, sizeof(_radioData)))
    {
        bufferReady = false;
        failcount = 0;
    } else
    {
        //Serial.println("increment failcount");
        failcount++;
        Serial.print(".");
    }

}

//void setOutput(OutputPinState data)
void setOutput(uint8_t pinNumber, uint8_t pinMode, uint8_t pinState, uint16_t duration)
{
    if (pinState == TOGGLE)
    {
        Serial.print("TOGGLE ");
    }
    if (pinState == KEEP_HIGH)
    {
        Serial.print("SET HIGH ");
    }
    if (pinState == KEEP_LOW)
    {
        Serial.print("SET LOW ");
    }
    
    Serial.print(pinNumber); Serial.print(" to "); Serial.print(pinMode); Serial.print(" for "); Serial.println(duration); 
    digitalWrite(pinNumber, pinMode);
    if (pinState == TOGGLE)
    {
        //this is blocking.  Refactor to unset the pin on a parallel timer;
        delay(duration);
        digitalWrite(pinNumber, !pinMode);
    }
}

bool readInput(uint8_t pinNumber)
{
    return digitalRead(pinNumber);
}

#pragma endregion
#pragma region data

void decodePacket(char data[28])
{
  byte command;
  byte state;
  byte pinNumber;
  byte pinMode;
  byte low;
  uint16_t high;
  uint16_t pinTime;
  if (data[0] = COMMAND)
  {
    command = data[1];
    if (data[2] == STATE)
    {
        state = data[3];
    }
    if (data[4] == PIN)
    {
        pinNumber = data[5];
    }
    if (data[6] == MODE)
    {
        pinMode = data[7];
    }
    low = data[8];
    high = data[9];
    high = high << 8;
    pinTime = low | high;
  }

    if (command == NEW_ID)
    {
        writeDeviceID(data[1]);
    }
    
  if (command == SET_OUTPUTS)
  {
    setOutput(pinNumber, pinMode, state, pinTime);
  } 

  else if (command == GET_INPUTS)
  {
      if (readInput(pinNumber))
      {
          _radioData.Payload[0] = INPUTS;
          _radioData.Payload[1] = pinNumber;
          _radioData.Payload[2] = 1;
      }
      else
      {
          _radioData.Payload[0] = INPUTS;
          _radioData.Payload[1] = pinNumber;
          _radioData.Payload[2] = 0;
      }
      bufferReady = true;
  }
}

#pragma endregion
#pragma region config

void onInputChanged(byte pinNumber, bool value)
{
    Serial.print("input ");
    Serial.print(pinNumber);
    Serial.print(" ");
    Serial.println(value);
    _radioData.Payload[0] = INPUTS;
    _radioData.Payload[1] = pinNumber;
    _radioData.Payload[2] = value;
    if (!bufferReady)
    {
        bufferReady = true;
    }
}

void initialiseRadio()
{
    //Serial.println("Initialising Radio");
    //int radioID = readDeviceID();
    int radioID = 2;
    if (!_radio.init(radioID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    } else
    {
        //Serial.print("radio ready. ID: ");
        Serial.println(radioID);
        _radioData.FromRadioId = radioID;
    }
    delay(50);
}


void setup()
{
    Serial.begin(115200);
    Serial.println("1st");
    initialiseRadio();
    randomSeed(analogRead(A0));
/*     if (_radioData.FromRadioId == 0)
    {
        Serial.println("ID is 0");
    }
 */ 
    //getNodeConfig();
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    //writeDeviceID(B00000001);
    //announce();
    prevMillis = millis();
    Serial.println("Setup done");
}
#pragma endregion

void loop()
{
    if (bufferReady)
    {
        Serial.println("buffer is ready");
        sendRadioPacket(DESTINATION_RADIO_ID);
        Serial.println("message sent");
    }

    while (_radio.hasData())
    {
        _radio.readData(&_radioData); 
        Serial.println("New data received:");
        String incomingData = _radioData.Payload;
        //Serial.println(incomingData);
        //processPayload(incomingData);
        decodePacket(_radioData.Payload);
    }

    if (!input6 == readInput(6))
    {
        input6 = !input6;
        onInputChanged(6, input6);
        Serial.println("6 changed");
    }

    if (!input7 == readInput(7))
    {
        input7 = !input7;
        onInputChanged(7, input7);
        Serial.println("7 changed");
    }


    if ((millis() - prevMillis) >= 10000)
    {
        prevMillis = millis();
        announce();
    }
    
    if (failcount > 1000)
    {
        failcount = 0;
        Serial.println("lots of fail");
    }
}
