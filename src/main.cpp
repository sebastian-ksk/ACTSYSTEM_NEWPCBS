#include <Arduino.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include<stdio.h>
#include<string.h>
#include<math.h>

char VALV_HB1 = 0;
char VALV_HB2 = 0;
char VALV_RPA = 0;
char VALV_IFS = 0;

#define STATUS_INDICATOR_PIN PINC5

enum FLAGS_STATES {CLEAR,SET};
volatile unsigned int irrigPulsesNumberArr[4];
volatile unsigned int pulsesCounterArr[4]={0};
volatile unsigned char autoIrrigCompleteFlagArr[4] = {CLEAR};
volatile unsigned char actAutoIrrigFlagArr[4] = {CLEAR};
volatile unsigned char busyValveFlags[2]={CLEAR};
volatile unsigned char time_comp_Flags[2]={CLEAR};

volatile unsigned int activeStationIndicator = 0;

volatile static uint8_t pcint_temp;

//Scheduled Irrigation Variables
volatile unsigned int timer0ItptCounter = 0;
volatile unsigned int secondsCounter = 0;
volatile uint8_t increment;
volatile unsigned int schedTaskTimeArr[4]={0};
volatile unsigned int secondsCounterArr[4]={0};
volatile unsigned char actSchedIrrigFlagArr[4]={CLEAR};
volatile unsigned char busyValveFlagArr[4]={CLEAR};

volatile unsigned int batteryADCValue;



//inicilzacion de objeto xbee
//#define XBEE_RESP 0X91 antes 
#define XBEE_RESP 0X90
XBee xbee = XBee();
uint8_t text[100] = {};
XBeeAddress64 addr64 = XBeeAddress64(0x000000000, 0x000000000);
ZBTxRequest zbTx = ZBTxRequest(addr64, text, sizeof(text));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
//

int rout;
int statusLed = 28;
int errorLed = 28;
int dataLed = 28;

String FUNC_STR_ARR[4];
static const int functionsNumber = 4;
unsigned char functionID;
unsigned char ID_func;

char requestStrPtrArr[4][50];
unsigned int requestsNumber = 0;

char responseBufferArr[7][50];
volatile int responseBufferArrIndex = -1;
volatile unsigned char remainActSchedTaskFlag = CLEAR;

// funciones 
void envia(String D);
void flashLed(unsigned int loops);
unsigned char findFunction(String paramStrPtr );
void runSensorControledIrrigation();
void runTimeControledIrrigation(int Valv,int time);
void getBatteryLevelValue();
void turnOnValve(unsigned char VALV_HB1, unsigned char VALV_RPA, char valve);
void turnOffValve1(unsigned char VALV_HB2, unsigned char VALV_RPA, char valve);
void runSensorControledIrrigation(int Valv,int pulses);
void end_task_irr(unsigned char i);


void setup() {

  Serial.println("init ...");
  //=======PINES PARA CONTROL DE PUENTEH =====
  DDRB = 0b00001111; // Set  PORTB as output
  PORTB = 0b00000000; // Set all PORTB LOW Level
  // =========================================
  //==========PINES DE CONTROL PARA RELEE=====
  DDRD = (1 << PIND6) | (1 << PIND7) ;
  PORTD = 0b00000000;
  PORTD |= (1 << PIND6) | (1 << PIND7) ;
  //==========================================
  //==========CONEXION A SERIAL PORT==========
  Serial.begin(9600); 
  //==========================================
  //=====CONEXION CON XBEE====================
  xbee.setSerial(Serial);
  flashLed(5);
  //==========================================
  //=========PALABRAS DE PETICION============
  FUNC_STR_ARR[0] = "AITASK";
  FUNC_STR_ARR[1] = "SITASK";
  FUNC_STR_ARR[2] = "ADCREQ";
  FUNC_STR_ARR[3] = "SSTOPP";
  //==========================================
  //=======HABILITACION DE INTERRUPCIONES=====
  cli();
  

  
  //---Timer interrupt contador de segundos para riego------
  TCNT2 = 0x00;
  TCCR2B = (1<<CS22) | (1<<CS21)| (1<<CS20); //Clock/1024
  TIMSK2 |= (1<<TOIE2);

  //Timer for revit
  TCCR1A = 0; 
  TCCR1B = 0; 
  TCCR1B |= (1<<CS12) |  (1<<WGM12) |  (1 << CS10); 
  TIMSK1 =  (1<<OCIE1A); 
  OCR1A = 100;
  OCR1B = 100; 
 
  //Enable Global Interrupts
  PCICR |= (1 << PCIE1);    // set PCIE1 (PCINT[8:14])to enable PCMSK1 scan
  
  //==========================================

  
  //===============CONFIGURACION ADC======== 
  /*  ADCSRA
    Bit 7 - 	ADEN: ADC Enable
    Bit 6 - 	ADSC: ADC Start Conversion
    Bit 5 - 	ADATE: ADC Auto Trigger Enable
    Bit 4 - 	ADIF: ADC Interrupt Flag
    Bit 3 -		ADIE: ADC Interrupt Enable
    Bits 2:0 - 	ADPS[2:0]: ADC Prescaler Select Bits (100 to divide by 16)
    */
  ADCSRA = 0b10001111;
  /*  ADMUX
    Bit 7:6 – 	REFS[1:0]: Reference Selection Bits
    Bit 5 – 	ADLAR: ADC Left Adjust Result
    Bit 4 – 	Reserved
    Bits 3:0 – 	MUX[3:0]: Analog Channel Selection Bits
    */
  ADMUX = 0b00001111;
  /* Disable Digital Input on ADC Channel 5 to reduce power consumption */
  DIDR0 |= (1 << ADC5D);

  _delay_ms(1000);
  flashLed(10); 
  Serial.println("init now");
   //========LED INIDCADOR Y ALIMENTACION DE SENSORES=========
  DDRC |= (1 << STATUS_INDICATOR_PIN) | (1<<PINC3) | (1<<PINC4)  ; // LED, AFSENSVALV1,AFSENSVALV2
  PORTC &= ~(1 << STATUS_INDICATOR_PIN);
  PORTC &= ~(1<<PINC3);
  PORTC &= ~(1<<PINC4);
  // =========================================

  sei();

}

// the loop function runs over and over again forever
void loop() {

if(responseBufferArrIndex >= 0){
			//USART_Transmit('t');
			_delay_ms(250);
			 envia(responseBufferArr[responseBufferArrIndex]);
			responseBufferArrIndex--;
			//Deactivate XBee Awake Pin
}


  //==========LECTURA DE XBEE ==================	
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { 
    //======MENSAJE RECIBIDO====================
    Serial.println("MESSAGE");
    
    //envia(String(xbee.getResponse().getApiId()));
    Serial.println(xbee.getResponse().getApiId()); 
    

   
    // if(String(xbee.getResponse().getApiId())!="139"){
    //    responseBufferArrIndex++;
		// 	sprintf(responseBufferArr[responseBufferArrIndex], "%d\n", String(xbee.getResponse().getApiId()));
    // }

    if (xbee.getResponse().getApiId() == XBEE_RESP ) {
      _delay_ms(100);
      
      
      xbee.getResponse().getZBRxResponse(rx);

      Serial.println("MESSAGE RECEIVED!");
      Serial.print("checksum is ");
      Serial.println(rx.getChecksum(), HEX);
      Serial.print("Longitud Paquete RX ");
      Serial.println(rx.getPacketLength(), DEC);
      String dataStringXbee = "";
      String dataHexXbee = "";
      char temp;
      char c;
      char str[100];
    
          //=============DISCRIMINACION DE LAS 8 PRIMEROS CARACTERES DEL MENSAJE =======
      for (int i = 0; i < xbee.getResponse().getFrameDataLength(); i++) {
        if(i==7 && xbee.getResponse().getFrameData()[i]==0x46){
          rout=1;    
        }
        if(i>=11){ 
          temp = char(xbee.getResponse().getFrameData()[i]);
          c = toascii(temp);   
          dataStringXbee=dataStringXbee+String(c);
          dataHexXbee=dataHexXbee+String(xbee.getResponse().getFrameData()[i], HEX);
          str[i-11]=c;
        }

      if(rout==1){
        Serial.println("Paquete de router completado");
        requestsNumber++;
      }

      }

      //==============================================================================
      //==========================IMPRIMIR DATOS RECIBIDOS============================
      Serial.println("data get: ");
      Serial.println(dataStringXbee);
      Serial.println(dataHexXbee);
      //==============================================================================
      //==========================SEPARACION DE MENSAJE POR (;) ====================
      char *decoder = NULL;
      decoder = strtok(str, ";");
      char *decodedMessag[4];
      int n=0;
      while(decoder != NULL)
      {
        Serial.println(decoder);
        decodedMessag[n]= decoder;
        decoder = strtok(NULL, ";");
        n++;
      }
      //======conversion a string de orden de riego================
      String Order="";
       for(int i=0;i<6;i++){
         Order = Order + String(decodedMessag[0][i]);
       }
      //==============================================================================
      //============================DETECCION DE PALABRA CLAVE========================
      ID_func=findFunction(Order);
      //envia(String(decodedMessag[0]));
      // responseBufferArrIndex++;
			// sprintf(responseBufferArr[responseBufferArrIndex], "%s%d,%d,%d\n", "yes ",ID_func,atoi(decodedMessag[1]),atoi(decodedMessag[2]));
      // Serial.println(String(ID_func));
      //==============================================================================
      //-------------------------------EJECUCION DE TAREA ---------------------------
      switch(ID_func){
        case 1:
          //=======================RIEGO CONTROLADO POR SENSORES----------------------
          flashLed(4);
          runSensorControledIrrigation(atoi(decodedMessag[1]),atoi(decodedMessag[2]));
          break;
          //--------------------------------------------------------------------------
        case 2:
          //-----------------------RIEGO CONTROLADO POR SENSORES----------------------
          flashLed(6);
          runTimeControledIrrigation(atoi(decodedMessag[1]),atoi(decodedMessag[2]));
          break;
          //--------------------------------------------------------------------------
        case 3:
          //-----------------------NIVEL DE BATERIA-----------------------------------
          flashLed(8);
          getBatteryLevelValue();
          break;
          //--------------------------------------------------------------------------
        case 4:
          //------------------------APAGADO DE EMERGENCIA------------------------------        
          flashLed(10);
          end_task_irr(atoi(decodedMessag[1]));
          break;
          //---------------------------------------------------------------------------
        default:
          break;
      } 
	

	remainActSchedTaskFlag = CLEAR;		
	for( unsigned char k = 0; k < 2; k++){
		if(actSchedIrrigFlagArr[k] == SET){
			remainActSchedTaskFlag = SET;
			break;
		}
	}
	//Deactivate TIMER for seconds counting in schedlued Tasks
	if( remainActSchedTaskFlag == CLEAR ){
		TIMSK2 &=~ (1<<TOIE2);
	}

   
  } 

  }
  //=========ERROR EN CONECCION DE XBEE============= 
  else if (xbee.getResponse().isError()) {
    Serial.println(".......ERROR DE XBEE........");
  }
}


unsigned char findFunction(String paramStrPtr ){
	unsigned char functionID = 0;
	for(int i = 0; i < functionsNumber; i++){
		if (paramStrPtr==FUNC_STR_ARR[i]){
			functionID = i + 1;
      Serial.print("ok....");
			break;
		}
	}
	return functionID;
}

void envia(String D)
{
  for(int i=0;i<=100;i++)
  {
    text[i]=D[i];
  }
 xbee.send(zbTx);
}

void flashLed(unsigned int loops) {   
	PORTC &=~ (1 << STATUS_INDICATOR_PIN);
	PORTC |= (1 << STATUS_INDICATOR_PIN);
	while (0 < loops){
		_delay_ms(300);
		--loops;
	}
	PORTC &=~ (1 << STATUS_INDICATOR_PIN);
}

void runSensorControledIrrigation(int Valv,int pulses)
{
  Serial.println("irr por control de sensores ");
 
  for (unsigned char i = 1; i < 3; i++)
  {
    if(Valv==i && busyValveFlags[i-1]==CLEAR){
      Serial.println("valv: "+String(Valv));
      cli();
      irrigPulsesNumberArr[i-1] = pulses;
      PCMSK1 |= (1 << (i));
      PORTC |= (1<<(i+2));
      turnOnValve((2*(i-1)),(((i-1)+13)/2),i);
	    actAutoIrrigFlagArr[i-1] = SET;
      busyValveFlagArr[i-1] = SET;
	    responseBufferArrIndex++;
	    sprintf(responseBufferArr[responseBufferArrIndex],  "IRRIG:INIT;SensControl");
      sei();
    }
}

}


void runTimeControledIrrigation(int Valv,int time){
Serial.println("irr por control de tiempo ");
Serial.println("");
	for (unsigned char i = 1; i < 3; i++)
	{
    if(Valv==i && busyValveFlags[i-1]==CLEAR){
      Serial.println("valv: "+String(Valv));
      cli();
      schedTaskTimeArr[i-1] = time;
      secondsCounterArr[i-1]=0;
      actSchedIrrigFlagArr[i-1] = SET;
      busyValveFlagArr[i-1] = SET;
      Serial.println("tiempo: "+String(schedTaskTimeArr[i-1]));
      //activar sensores
      PORTC |= (1<<(i+2));
      PCMSK1 |= (1 << (i));
      turnOnValve((2*(i-1)),(((i-1)+13)/2),i);
      sei();
      responseBufferArrIndex++;
      sprintf(responseBufferArr[responseBufferArrIndex],  "IRRIG:INIT;TimeControl");
    }
	}
}



void turnOnValve(unsigned char VALV_HB1, unsigned char VALV_RPA, char valve){
	PORTD &=~ (1 << VALV_RPA);
	_delay_ms(50);
		PORTB |= (1 << VALV_HB1);
		_delay_ms(50);
		PORTB &=~ (1 << VALV_HB1);
	_delay_ms(50);
	PORTD |= (1 << VALV_RPA);
	PORTC |= (1 <<  PINC4);
	activeStationIndicator++;
}

void turnOffValve1(unsigned char VALV_HB2, unsigned char VALV_RPA, char valve){
	PORTD &=~ (1 << VALV_RPA);
	_delay_ms(50);
		PORTB |= (1 << VALV_HB2);
		_delay_ms(50);
		PORTB &=~ (1 << VALV_HB2);
	_delay_ms(50);
	PORTD |= (1 << VALV_RPA);

	activeStationIndicator--;
}


/*TIMER INTERRUPT FUNCTIONS*/
ISR(TIMER2_OVF_vect) {
	if( timer0ItptCounter > 31){
    //Serial.println(String(timer0ItptCounter));
		timer0ItptCounter = 0;
	for(unsigned char i = 0; i < 2; i++){
			if(actSchedIrrigFlagArr[i] == SET){
				increment = 1;
			}
			else{
				increment = 0;
			}
			secondsCounterArr[i] = secondsCounterArr[i] + increment;  
		}
	}
	else{
		timer0ItptCounter++;
	}

}

// Review end of irrigation tasks
ISR(TIMER1_COMPA_vect){

  for( unsigned char i = 1; i < 3; i++){
		if( autoIrrigCompleteFlagArr[i-1] == SET){
			autoIrrigCompleteFlagArr[i-1] = CLEAR;
			turnOffValve1(((2*i)-1),(((i-1)+13)/2),i);
			PCMSK1 &= ~(1 << (i));
      PORTC &= ~(1<<(i+2));
			busyValveFlagArr[i-1] = CLEAR;
			actAutoIrrigFlagArr[i-1] = CLEAR;
			responseBufferArrIndex++;
			sprintf(responseBufferArr[responseBufferArrIndex], "%s;%d;%d;%d \n", "IRRIG:COMPLETE", i, (secondsCounterArr[i-1]-1),pulsesCounterArr[i-1]);
			irrigPulsesNumberArr[i-1] = 0;
			pulsesCounterArr[i-1] = 0;
		}
		if(secondsCounterArr[i-1] >schedTaskTimeArr[i-1]){
      PCMSK1 &=~ (1 << (i));
      PORTC &= ~(1<<(i+2));
      turnOffValve1(((2*i)-1),(((i-1)+13)/2),i);
			responseBufferArrIndex++;
			sprintf(responseBufferArr[responseBufferArrIndex], "%s;%d;%d;%d \n", "IRRIG:COMPLETE", i, (secondsCounterArr[i-1]-1),pulsesCounterArr[i-1]);
			pulsesCounterArr[i-1] = 0;
			irrigPulsesNumberArr[i-1] = 0;
      actSchedIrrigFlagArr[i-1] = CLEAR;
			secondsCounterArr[i-1] = 0;
			busyValveFlagArr[i-1] = CLEAR;

		}
	}
}



void end_task_irr(unsigned char i){ 
	   
  if(actSchedIrrigFlagArr[i-1] == SET){
        turnOffValve1(((2*i)-1),(((i-1)+13)/2),i);
      Serial.println("");
      Serial.println("apagado.... ");
			PCMSK1 &=~ (1 << (i-1));
      PORTC &= ~(1<<(i+2));
      actSchedIrrigFlagArr[i-1] = CLEAR;
      actAutoIrrigFlagArr[i-1] = CLEAR;
		  busyValveFlagArr[i-1] = CLEAR;
      autoIrrigCompleteFlagArr[i-1] = CLEAR;
      responseBufferArrIndex++;
	    sprintf(responseBufferArr[responseBufferArrIndex], "%s;%d;%d;%d\n", "IRRIG:STOP;", i, secondsCounterArr[i-1],pulsesCounterArr[i-1]);
	    irrigPulsesNumberArr[i-1] = 0;
      pulsesCounterArr[i-1] = 0;
      secondsCounterArr[i-1] = 0;
      schedTaskTimeArr[i-1]=0;
      flashLed(5);
     }
    
}




void getBatteryLevelValue(){
			ADMUX = 0b00000101;
			_delay_ms(100);
			ADCSRA |= (1 << ADSC);

}




/*EXTERNA INTERRUPT*/
ISR(PCINT1_vect)
{
    uint8_t changedbits;
    uint8_t reg;
    // Get the port byte.
    reg = PINC;
    // XOR saved reg with current reg to get difference.
    changedbits = reg ^ pcint_temp;
    // Save the new value of PORTB.
    pcint_temp = reg;
    // PCIN(i-1)
	 for(unsigned char i = 1; i < 3; i++){
		if(changedbits & (1 << i)){
			pulsesCounterArr[i-1]++;
			//printInt(pulsesCounterArr[i-1]);
		}
		if( actAutoIrrigFlagArr[i-1] == SET && pulsesCounterArr[i-1] == irrigPulsesNumberArr[i-1]){
		    	autoIrrigCompleteFlagArr[i-1] = SET;
		}
	}

}

ISR(ADC_vect)
{
	/* Read the ADC Result */
	batteryADCValue = ADC;
	/* Clear the ADC Interrupt Flag */
	ADCSRA |= (1 << ADIF);
  String dataStringXbee = "BATT:"+String(batteryADCValue)+"\n";
  responseBufferArrIndex++;
  sprintf(responseBufferArr[responseBufferArrIndex], "%s%d;\n", "BATT:", batteryADCValue);
	//printInt(batteryADCValue);
}