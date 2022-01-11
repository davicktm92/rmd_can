#include <mcp_can.h>
#include <SPI.h>

long StepValue=1296000*1;   //Valor de posicion seteado por numero de vueltas (*num=1)
#define nummax 4294967296

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];  
long speedControl=500000;
long speedControl_pos=5000;
long speedZero=0;
long angleControl=20000;
long angleInicio=0;
char lecturaserial; // for incoming serial data
long iqn;
long velocidadn;
long encodern;


//Byte de inicialización
byte data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para modo de operación (o)
byte datao[8] = {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para liberar freno de motor (b)
byte datab[8] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para apagar el motor (v)
byte datav[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para parar el motor (k)
byte datak[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para poner en marcha el motor (c)
byte datac[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para control de torque (t)
byte datat[8] = {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//Byte para velocidad en sentido horario (a)
byte dataa[8] = {0xA2, 0x00, 0x00, 0x00, (speedControl & 0xFF),(speedControl>>8) & 0xff,(speedControl>>16) & 0xff,(speedControl>>24) & 0xff};
//Byte para velocidad en sentido antihorario (d)
byte datad[8] = {0xA2, 0x00, 0x00, 0x00, (-speedControl & 0xFF),(-speedControl>>8) & 0xff,(-speedControl>>16) & 0xff,(-speedControl>>24) & 0xff};
//Byte para velocidad cero (s)
byte datas[8] = {0xA2, 0x00, 0x00, 0x00, speedZero,speedZero,speedZero,speedZero};
//Byte para posicion en cero (r)
byte datar[8] = {0xA4, 0x00,0x14,0x14,(angleInicio & 0xFF),(angleInicio >> 8) & 0xff,(angleInicio >> 16) & 0xff,(angleInicio >> 24) & 0xff};
//Byte para posicion deseada (q)
byte dataq[8] = {0xA4, 0x00,0x14 ,0x14,(angleControl & 0xFF),(angleControl >> 8) & 0xff,(angleInicio >> 16) & 0xff,(angleInicio >> 24) & 0xff};

byte* data=data1;

#define CAN0_INT 2 
MCP_CAN CAN0(10);     // Set CS to pin 10

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN0_INT, INPUT);

  //Activar motor para el uso
  for (int i=0; i<2; i++){
    CAN0.sendMsgBuf(0x141, 0, 8, datac);
  }
  
  
}

long GenPos=0;

void loop() {
  unsigned char len=0;
  unsigned char buf[8];
  
  if (Serial.available() > 0) {
    // Lectura del valor ingresado por Serial
    lecturaserial = char(Serial.read());
    
    switch (lecturaserial) {
      case 'o':
        data=datao;
        break;
      case 'b':
        data=datab;
        break;
      case 'v':
        data=datav;
        break;
      case 'k':
        data=datak;
        break;
      case 'c':
        data=datac;
        break;
      case 't':
        data=datat;
        break;
      case 'a':
        data=dataa;
        break;
      case 'd':
        data=datad;
        break;
      case 's':
        data=datas;
        break;
      case 'r': 
        GenPos=GenPos-StepValue;
        if (GenPos>=(nummax)){
          GenPos=(nummax-1);
        }
        if (GenPos<0){
          GenPos=min(GenPos+(nummax-1),(nummax-1));
        }
        Serial.println(GenPos);
        
        buf[0]=0xA4;
        buf[1]=0x00;
        buf[2]=speedControl_pos & 0xFF;
        buf[3]=(speedControl_pos>>8) & 0xff;
        buf[4]=GenPos & 0xFF;
        buf[5]=(GenPos >> 8) & 0xff;
        buf[6]=(GenPos >> 16) & 0xff;
        buf[7]=(GenPos >> 24) & 0xff;
        data=buf;
        CAN0.sendMsgBuf(0x141, 0, 8, buf);    
        break;
        
      case 'q':
        GenPos=GenPos+StepValue;
        if (GenPos>(nummax-1)){
          GenPos=max(GenPos-(nummax-1),0);
        }
        if (GenPos<-nummax){
          GenPos=0;
        }
        Serial.println(GenPos);
        
        buf[0]=0xA4;
        buf[1]=0x00;
        buf[2]=speedControl_pos & 0xFF;
        buf[3]=(speedControl_pos>>8) & 0xff;
        buf[4]=GenPos & 0xFF;
        buf[5]=(GenPos >> 8) & 0xff;
        buf[6]=(GenPos >> 16) & 0xff;
        buf[7]=(GenPos >> 24) & 0xff;

        data=buf;
        
        CAN0.sendMsgBuf(0x141, 0, 8, buf);
        break;
    }
    
  }
      byte sndStat=CAN0.sendMsgBuf(0x141, 0, 8, data); 
      
      //Respuesta del protocolo CAN del motor
      if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
      {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);
        
        if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
          sprintf(msgString, " REMOTE REQUEST FRAME");
          Serial.print(msgString);
          } else {
            
            int8_t temperatura=rxBuf[1];
            int16_t iq=rxBuf[2];
            int16_t iq1=rxBuf[3];
            int16_t velocidad=rxBuf[4];
            int16_t velocidad1=rxBuf[5];
            uint16_t encoder=rxBuf[6];
            uint16_t encoder1=rxBuf[7];

            iqn = (iq1 << 8) | iq;
            velocidadn=(velocidad1 << 8) | velocidad;
            encodern=(encoder1 << 8) | encoder;
            
            Serial.print("Temperatura: ");
            Serial.print(temperatura);
            Serial.print("\t");

            Serial.print("Corriente: ");
            Serial.print(iqn);
            Serial.print("\t");

            Serial.print("Velocidad: ");
            Serial.print(velocidadn);
            Serial.print("\t");

            Serial.print("Encoder: ");
            Serial.print(encodern);
            Serial.print("\t");
          }

          Serial.println();
      }
  delay(100);
}
