#include <SPI.h>
#include <Ethernet.h>

#define MAX_DIGITAL_WRITE_COMMANDS 16
#define MAX_DIGITAL_READ_COMMANDS 16
#define MAX_ANALOG_READ_COMMANDS 4

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

//////////IP ADDRESS DEFINITIONS
IPAddress ip(192, 168, 1, 88);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 254);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(8888); 
/////////////////////


class DigitalWriteSetup
{
  public:
    byte writeCmd[3];
    unsigned int pattern;
    unsigned int bits;
    byte readCmd[3];
    byte readAnsw[3]; //plus terminator
};

class DigitalReadSetup
{
  public:
    byte readCmd[3];
    unsigned int pattern;
    unsigned int bits;
    byte readAnsw[3]; //plus terminator
};

class AnalogReadSetup
{
  public:
    byte readCmd[2];
    byte idx;
    byte readAnsw[4]; //plus terminator
};

class Configuration
{
  public:
    DigitalWriteSetup digitalWrites[MAX_DIGITAL_WRITE_COMMANDS];
    DigitalReadSetup digitalReads[MAX_DIGITAL_READ_COMMANDS];
    AnalogReadSetup analogReads[MAX_DIGITAL_READ_COMMANDS];
    int numDigitalWrites;
    int numDigitalReads;
    int numAnalogReads;
};

Configuration config;
unsigned int outBits = 0;
int lastWriteIdx = -1;
unsigned int totWriteMask;
/* Configuration Command format:
 *  Mode (1 Byte): 'W' (digital write configuration), 'R' (digital read configuration), 'A': analog read configuration
 *  For mode 'W':
 *  Command (2 Bytes) - write  command
 *  Pattern (4 bytes) - consedered bits battern (Hex)
 *  Bits (4 bytes) - Bit values (Hex)
 *  Read Command (2 bytes) - Configuration redout command
 *  Read Answer (2 bytes) - expected 
 *  For mode 'R':
 *  Command (2 Bytes) - read command
 *  Pattern (4 bytes) - consedered bits battern (Hex)
 *  Bits (4 bytes) - Bit values (Hex)
 *  ReadAnswer(2 bytes) - expected response 
 *  For mode 'A':
 *   Command (2 Bytes) - read command
 *   Chan Idx (1 byte) - Analog input idx. In this case the answer shann be the lelev (2 byues Hex) read
 *   
 *   Return number of bytes consumed, -1 in case of error
 */

void prgList()
{
    char outBuf[32];
    int i;
    for(i = 0; i < config.numDigitalWrites; i++)
    {
        outBuf[0] = 'W';
        outBuf[1]= config.digitalWrites[i].writeCmd[0];
        outBuf[2] = config.digitalWrites[i].writeCmd[1];
        outBuf[3] = config.digitalWrites[i].writeCmd[2];
        sprintf(&outBuf[4], "%04X", config.digitalWrites[i].pattern);
        sprintf(&outBuf[8], "%04X", config.digitalWrites[i].bits);
        outBuf[12]= config.digitalWrites[i].readCmd[0];
        outBuf[13] = config.digitalWrites[i].readCmd[1];
        outBuf[14] = config.digitalWrites[i].readCmd[2];
        outBuf[15]= config.digitalWrites[i].readAnsw[0];
        outBuf[16] = config.digitalWrites[i].readAnsw[1];
        outBuf[17]= 0;
        Serial.println(outBuf);
    }
    for(i = 0; i < config.numDigitalReads; i++)
    {
        outBuf[0] = 'R';
        outBuf[1]= config.digitalReads[i].readCmd[0];
        outBuf[2] = config.digitalReads[i].readCmd[1];
        outBuf[3] = config.digitalReads[i].readCmd[2];
        sprintf(&outBuf[4], "%04X", config.digitalReads[i].pattern);
        sprintf(&outBuf[8], "%04X", config.digitalReads[i].bits);
        outBuf[12]= config.digitalReads[i].readAnsw[0];
        outBuf[13] = config.digitalReads[i].readAnsw[1];
        outBuf[14]= 0;
        Serial.println(outBuf);
    }
    for(i = 0; i < config.numAnalogReads; i++)
    {
        outBuf[0] = 'A';
        outBuf[1]= config.analogReads[i].readCmd[0];
        outBuf[2] = config.analogReads[i].readCmd[1];
        sprintf(&outBuf[3], "%d", config.analogReads[i].idx);
        outBuf[7]= 0;
        Serial.println(outBuf);
    }
}
 
char * handleSingleConfiguration(char *confStr, unsigned int *writeMask)
{
  char hBuf[5];
  hBuf[4] = 0;
  int currPattern;
  switch(confStr[0])  {
    case 'W': 
        if( config.numDigitalWrites >= MAX_DIGITAL_WRITE_COMMANDS)
            return -1;
        config.digitalWrites[config.numDigitalWrites].writeCmd[0] = confStr[1];
        config.digitalWrites[config.numDigitalWrites].writeCmd[1] = confStr[2];
        config.digitalWrites[config.numDigitalWrites].writeCmd[2] = confStr[3];
        config.digitalWrites[config.numDigitalWrites].readCmd[0] = confStr[12];
        config.digitalWrites[config.numDigitalWrites].readCmd[1] = confStr[13];
        config.digitalWrites[config.numDigitalWrites].readCmd[2] = confStr[14];
        config.digitalWrites[config.numDigitalWrites].readAnsw[0] = confStr[15];
        config.digitalWrites[config.numDigitalWrites].readAnsw[1] = confStr[16];
        config.digitalWrites[config.numDigitalWrites].readAnsw[2] = 0;
        hBuf[0] = confStr[4];
        hBuf[1] = confStr[5];
        hBuf[2] = confStr[6];
        hBuf[3] = confStr[7];
        sscanf(hBuf, "%X", &config.digitalWrites[config.numDigitalWrites].pattern);
        hBuf[0] = confStr[8];
        hBuf[1] = confStr[9];
        hBuf[2] = confStr[10];
        hBuf[3] = confStr[11];
        sscanf(hBuf, "%X", &config.digitalWrites[config.numDigitalWrites].bits);
        *writeMask |= config.digitalWrites[config.numDigitalWrites].pattern;
        config.numDigitalWrites++;
        return "OK";
    case 'R':
        if( config.numDigitalReads >= MAX_DIGITAL_READ_COMMANDS)
            return "ER";
        config.digitalReads[config.numDigitalReads].readCmd[0] = confStr[1];
        config.digitalReads[config.numDigitalReads].readCmd[1] = confStr[2];
        config.digitalReads[config.numDigitalReads].readCmd[2] = confStr[3];
        hBuf[0] = confStr[4];
        hBuf[1] = confStr[5];
        hBuf[2] = confStr[6];
        hBuf[3] = confStr[7];
        sscanf(hBuf, "%X", &config.digitalReads[config.numDigitalReads].pattern);
        hBuf[0] = confStr[8];
        hBuf[1] = confStr[9];
        hBuf[2] = confStr[10];
        hBuf[3] = confStr[11];
        sscanf(hBuf, "%X", &config.digitalReads[config.numDigitalReads].bits);
        config.digitalReads[config.numDigitalReads].readAnsw[0] = confStr[12];
        config.digitalReads[config.numDigitalReads].readAnsw[1] = confStr[13];
        config.digitalReads[config.numDigitalReads].readAnsw[2] = 0;
        config.numDigitalReads++;
        return "OK";
    case 'A':
        if( config.numAnalogReads >= MAX_ANALOG_READ_COMMANDS)
            return -1;
        config.analogReads[config.numAnalogReads].readCmd[0] = confStr[1];
        config.analogReads[config.numAnalogReads].readCmd[1] = confStr[2];
        config.analogReads[config.numAnalogReads].idx = confStr[3];
        config.numAnalogReads++;
        return "OK";
    default:
        return "ER";
    
  }
}

char *handleCommand(char *cmd)
{
    char buf[64];
    int idx, bitIdx;
    unsigned int currReadBits;
    //First check whether this is a read digital configuration command
    for(idx = 0; idx < config.numDigitalWrites; idx++)
    {
        if((cmd[0] == config.digitalWrites[idx].readCmd[0]) &&  (cmd[1] == config.digitalWrites[idx].readCmd[1]) 
          &&  (cmd[2] == config.digitalWrites[idx].readCmd[2])
          && ((config.digitalWrites[idx].pattern & outBits) == (config.digitalWrites[idx].pattern & config.digitalWrites[idx].bits)))
            return config.digitalWrites[idx].readAnsw;
    }
    //Then check wether this is a write digital command
    for(idx = 0; idx < config.numDigitalWrites; idx++)
    {
        if((cmd[0] == config.digitalWrites[idx].writeCmd[0]) &&  (cmd[1] == config.digitalWrites[idx].writeCmd[1])
         &&  (cmd[2] == config.digitalWrites[idx].writeCmd[2]))
        {
            for(bitIdx = 0; bitIdx < 14; bitIdx++)
            {
                if((config.digitalWrites[idx].pattern >> bitIdx)&1)
                {
                    if((config.digitalWrites[idx].bits >> bitIdx)&1)
                    {
                        digitalWrite(bitIdx, HIGH);
                        outBits |= (1 << bitIdx);
                  }
                  else 
                  {
                        digitalWrite(bitIdx, LOW);
                        outBits &= ~(1 << bitIdx);
                   }
                }
            }
            lastWriteIdx = idx;
            return "OK";
        }
    }
    //Then check if this is a digital ready
    currReadBits = 0;
    for(int i = 9; i >= 0; i--)
    {
        currReadBits = currReadBits << 1;
        if(!((totWriteMask >> i) & 1))
        {
            if(digitalRead(i) == HIGH)
                currReadBits |= 1;
        }
    }
    for(idx = 0; idx < config.numDigitalReads; idx++)
    {
        if((cmd[0] == config.digitalReads[idx].readCmd[0]) &&  (cmd[1] == config.digitalReads[idx].readCmd[1])&&  (cmd[2] == config.digitalReads[idx].readCmd[2])
          && ((currReadBits & config.digitalReads[idx].pattern) == (config.digitalReads[idx].bits & config.digitalReads[idx].pattern)))
            return  config.digitalReads[idx].readAnsw;
    }
    //Finally check Analog reads
    for(idx = 0; idx < config.numAnalogReads; idx++)
    {
        if(cmd[0] == config.analogReads[idx].readCmd[0] &&  (cmd[1] == config.analogReads[idx].readCmd[1]))
        {
            int val = analogRead(config.analogReads[idx].idx);
            sprintf(config.analogReads[idx].readAnsw, "%3X", val);
            return config.analogReads[idx].readAnsw;
        }
    }
    return "ER"; //unrecognized command or bit pattern
}

int inCharIdx;
char inCommand[32];
byte state;
#define PROGRAMMING 0
#define EXECUTING 1

void setup() {
  // put your setup code here, to run once:
  outBits = 0;
  inCharIdx = 0;
  for(int i = 0; i < 16; i++)
      inCommand[i] = 0;
  state = PROGRAMMING;
  //test
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  Serial.begin(9600);
  config.numDigitalWrites = 0;
  config.numDigitalReads = 0;
  config.numAnalogReads = 0;
  pinMode(13, OUTPUT);
  
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start listening for clients
  server.begin();
  Serial.print("Command server address:");
  Serial.println(Ethernet.localIP());  
}
int alreadyConnected = 0;
void loop() {
  // put your main code here, to run repeatedly:
  char buf[16];
  //digitalWrite(13, HIGH);

  if(!server)  Serial.println("Server is NOT listening");
  EthernetClient client = server.available();
  //Serial.println(client);
  if (client) {
    Serial.println("New connection");
    while(client.connected()) {
     if(!alreadyConnected) {
        Serial.println("New Connection");
        client.flush();
        alreadyConnected = true;
      }
      if (client.available() > 0) {
        // read the bytes incoming from the client:
        char a = client.read();
        if(a == '\n' || a == '\r')
        {
            client.println(inCommand);
            Serial.println(inCommand);
            if(inCharIdx > 0)
            {
                if(!strcmp(inCommand,"PRGSTART"))
                {
                    config.numDigitalWrites = 0;
                    config.numDigitalReads = 0;
                    config.numAnalogReads = 0;
                    totWriteMask = 0;
                    state = PROGRAMMING;
                    client.println("OK");
                    Serial.println("OK");
                }
                else if(!strcmp(inCommand,"PRGSTOP"))
                {
                    if(state != PROGRAMMING)
                    {
                        client.println("ER");
                        Serial.println("ER");
                    }
                    else
                    {
                        client.println("OK");
                        Serial.println("OK");
                    }
                    state = EXECUTING;
                    for(int i = 0; i < 10; i++)
                    {
                        if((totWriteMask >> i) & 1)
                        {
                            pinMode(i, OUTPUT);
                            digitalWrite(i, LOW);
                        }
                        else
                            pinMode(i, INPUT);
                    }
                }
                else if (!strcmp(inCommand,"PRGLIST"))
                  prgList();
                else if (!strcmp(inCommand, "STOP"))
                {
                    client.println("OK");
                    Serial.println("OK");
                    //client.flush();
                    //client.stop();
                }
                else if(state == PROGRAMMING)
                {
                    char *answ = handleSingleConfiguration(inCommand, &totWriteMask);
                    client.println(answ);
                    Serial.println(answ);
                }
                else //Execution
                {
                    char *answ = handleCommand(inCommand);
                    client.println(answ);
                    Serial.println(answ);
                }
            }
            inCharIdx = 0; 
            for(int i = 0; i < 16; i++)
                inCommand[i] = 0;
        }
        else
            inCommand[inCharIdx++] = a;
      }
    }
    Serial.println("Client no more connected");
    alreadyConnected = false;
    client.stop();
   // server.begin();
  }
}
