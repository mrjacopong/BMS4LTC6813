/*!
  DC2259
  ltc6813-1: Battery stack monitor

  REVISION HISTORY
  $Revision: 1200 $
  $Date: 2017-01-26

  @verbatim

  NOTES
  Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.
   Ensure all jumpers on the demo board are installed in their default positions from the factory.
   Refer to Demo Manual DC2259.

  USER INPUT DATA FORMAT:
  decimal : 1024
  hex     : 0x400
  octal   : 02000  (leading 0)
  binary  : B10000000000
  float   : 1024.0
  @endverbatim


  Copyright (c) 2017, Linear Technology Corp.(LTC)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of Linear Technology Corp.

  The Linear Technology Linduino is not affiliated with the official Arduino team.
  However, the Linduino is only possible because of the Arduino team's commitment
  to the open-source community.  Please, visit http://www.arduino.cc and
  http://store.arduino.cc , and consider a purchase that will help fund their
  ongoing work.

  Copyright 2017 Linear Technology Corp. (LTC)
*/

//--------------------compatibilità con schede non arduino uno-----------------------------//

/*modifiche su LT_SPI.cpp
FUNCTION-> void spi_write(int8_t  data) :175
  SPI.transfer(data);
  //SPDR = data;                  
	//while (!(SPSR & _BV(SPIF)));  

FUNCTION-> int8_t spi_read(int8_t  data) :183    
  int8_t receivedVal = SPI.transfer(data);
	return receivedVal;
	//SPDR = data;
	//while (!(SPSR & _BV(SPIF)));
	//return SPDR; 
*/


#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "ltc681x.h"
#include "ltc6813.h"
#include <SPI.h>

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0


char get_char();
void print_menu();
void read_config_data(uint8_t cfg_data[][6], uint8_t nIC);
void print_cells(uint8_t datalog_en);
void print_open();
void print_config();
void print_rxconfig();
void print_aux(uint8_t datalog_en);
void print_stat();
void check_error(int error);
/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 1;//!<number of ICs in the daisy chain
const uint8_t TOTAL_CH = 18; // number of channel used per ADC
const uint8_t TOTAL_NTC = 8; // number of temperatures per ADC
const uint8_t unused_ch_1=9;//celle non usate
const uint8_t unused_ch_2=18;//celle no nusate

//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See ltc6813_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See ltc6813_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See ltc6813_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See ltc6813_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See ltc6813_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See ltc6813_daisy.h for Options
const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold ADC Code. LSB = 0.0001
const uint16_t MAXTEMP = 60;         // Over temperature GRADI CENTIGRADI
double MAXVOLTAGE = OV_THRESHOLD;    // per convertirla in double nella funzione error_check
const uint16_t OV_TIME_LIMIT=500;    // limite di tempo in millisecondi OV
const uint16_t OT_TIME_LIMIT=1000;    // limite di tempo in millisecondi OT


//per algoritmo di carica
const uint16_t delta_carica = 2000;  // massima differenza tra due batterie in serie 
const uint16_t SogliaCarica=41000;   // soglia tensione carica (4.0V)
bool in_carica=true;                 // true->in carica ; false->non in carica;
bool IsCharged=false;                //serve per controllare che stiamo caricando
//questo valore viene controllato nel loop in modo tale che sia modificabile 
//dinamicamente ogni volta che avviene il loop
const uint8_t RelayPin=10;           //pin del relay da aprire in caso di errore
const uint8_t ChargeSwitchPin=11;         //pin dello switch per avviare la carica
/*--variabili per ntc--*/
float AA1 = 3.354016 * pow(10, -3);
float BB1 = 2.569850 * pow(10, -4);
float CC1 = 2.620131 * pow(10, -6);
float D1 = 6.383091 * pow(10, -8);
float Resistenza = 10000;


//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED
/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
  These variables store the results from the ltc6813
  register reads and the array lengths must be based
  on the number of ICs on the stack
 ******************************************************/

cell_asic bms_ic[TOTAL_IC];                   //array di tensioni
int8_t temperatura[TOTAL_IC][TOTAL_NTC];      //array di temperature

/*!**********************************************************************
  \brief  Inititializes hardware and variables
 ***********************************************************************/

/*!**********************************************************************
  \struttura per gli errori
 ***********************************************************************/
struct control{
  boolean flag;                //FALSE->okay  TRUE->errore
  unsigned long time;          //momento in cui si verifica il primo errore
};
control error_OV[TOTAL_IC][TOTAL_CH];
control error_OT[TOTAL_IC][TOTAL_NTC];
void init_error_ov (control);
void init_error_ot (control);

void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  ltc681x_init_cfg(TOTAL_IC, bms_ic);
  ltc6813_reset_crc_count(TOTAL_IC, bms_ic);
  ltc6813_init_reg_limits(TOTAL_IC, bms_ic);
  init_error_ov (error_OV);
  init_error_ot (error_OT);
  init_pinout();
}

void loop(){
  uint8_t error = 0;
  uint32_t conv_time = 0;
  uint8_t user_command;
  uint8_t readIC = 0;
  char input = 0;
  uint8_t ChargeSwitc=digitalRead (ChargeSwitchPin);
  //--interfaccia utente temporanea--//
  if (Serial.available()){           // Check for user input
    uint8_t user_command;
    Serial.println("inserisci un numero qualsiasi per lanciare il programma");
    user_command = read_int();      // whait for a key
  
   //---------------------------------//
 
    voltage_measurment();       //leggo le tensioni dall'adc
    error_check();              //controllo degli errori e scrittura della matrice di errori
    while (false) carica();     //faccio finta che non sia in carica
   
    /*
    while (ChargeSwitch==HIGH && !IsCharged) {
      close_relay(RelayPin);
      voltage_measurment();
      if (error_check()) {      //se c'è un errore evito di tornare in questo ciclo
        IsCharged=true;
        break;
      }
      IsCharged=carica();
    }
    if(IsCharged && ChargeSwitc==LOW) {
      IsCharged=false;
    }
    se lo switch è low vuol dire che non voglio caricare
    appena si triggera entro nel while per la prima volta
    faccio la carica in loop, appena finisce IsCharged è vero, quindi non entra nel loop di nuovo
    appena rimetto lo switch in low, vuol dire che non voglio più caricare
    quindi setto IsCharged falso in modo fa autorizzare la carica 
    per la prossima volta che voglio caricare

    un po' difficile da capire, ma dovrebbe funzionare
    */

    Serial.println(ReadTempGrad (3,0));//0 sarebbe il prmio IC

    /*debug*/
    print_cells_debug();
  }
}



/******************************************************************************
FUNCTIONS
*******************************************************************************/

bool error_check(){                                                   //controllo degli errori
  for(uint8_t current_ic=0;current_ic<TOTAL_IC;current_ic++){
		for(uint8_t current_ch=0;current_ch<TOTAL_CH;current_ch++){
      /**overcurrent**/
      //salta la cella  unused_ch_1=9 e unused_ch_2=18
      if ((current_ch == unused_ch_1) || (current_ch == unused_ch_2)){}
      if(bms_ic[current_ic].cells.c_codes[current_ch]> MAXVOLTAGE){
         if (error_OV[current_ic][current_ch].flag==false){          //si triggera l'if se c'è
          set_new_error(error_OV[current_ic][current_ch]);           //un error_OT nuovo lo segno 
        }                                                            //e inizializzo il tempo.
        /*se un error_OV è vecchio non c'è bisogno di flaggarlo
        ma bisogna controllare che il tempo non ecceda il limite
        questo avviene nell'else*/
        else{
          if(time_check(error_OT[current_ic][current_ch].time, OT_TIME_LIMIT))
            shoutdown_error(RelayPin); 
            return true;
        }
      }
      else reset_error(error_OV[current_ic][current_ch]);           //in asssenza di error_OV il flag è diasttivato
    }
      /**overtemperature**/
    for(uint8_t current_ntc=0;current_ntc<TOTAL_NTC;current_ntc++){
      if(bms_ic[current_ic].aux.a_codes[current_ntc]  > MAXTEMP){
        if (error_OT[current_ic][current_ntc].flag==false){           //si triggera l'if se c'è  
          set_new_error(error_OV[current_ic][current_ntc]);           //un error_OT nuovo lo segno 
        }                                                             //e inizializzo il tempo.
        /*se un error_OV è vecchio non c'è bisogno di flaggarlo
          ma bisogna controllare che il tempo non ecceda il limite
          questo avviene nell'else*/
        else{
         if(time_check(error_OT[current_ic][current_ntc].time, OT_TIME_LIMIT))
          shoutdown_error(RelayPin);
          return true;
        }
      }
      else reset_error(error_OT[current_ic][current_ntc]);            //in asssenza di error_OV il flag è diasttivato
     }
  }
  return false;
}
void shoutdown_error(uint8_t pinOut){
  open_relay(pinOut);
}

void set_new_error(control errore_da_segnare){
  errore_da_segnare.time=millis();            
  errore_da_segnare.flag=true;
}

void reset_error(control errore_da_resettare){
  errore_da_resettare.flag=false;
}

bool time_check(unsigned long t_inizio ,uint16_t durata_max ){        //true se l'errore persiste
  /*controllo della durata dell'errore*/
  if (millis()-t_inizio>durata_max){
    return true;
  }
  return false;
}

void init_pinout(){
  pinMode(RelayPin,OUTPUT);
  pinMode(ChargeSwitchPin,INPUT);
}

void init_error_ov (control error_OV[][TOTAL_CH]){
  for(uint8_t current_ic=0;current_ic<TOTAL_IC;current_ic++){
		for(uint8_t current_ch=0;current_ch<TOTAL_CH;current_ch++){
			error_OV[current_ic][current_ch].flag=false;
      error_OV[current_ic][current_ch].time=0;
		}
	}
}

void init_error_ot (control error_OT[][TOTAL_NTC]){
 for(uint8_t current_ic=0;current_ic<TOTAL_IC;current_ic++){
		for(uint8_t current_ntc=0;current_ntc<TOTAL_NTC;current_ntc++){
			error_OT[current_ic][current_ntc].flag=false;
      error_OT[current_ic][current_ntc].time=0;
		}
	}
}  

void carica() {
  uint8_t top_voltage[TOTAL_IC];
  bool modulo_carico=0;
  uint8_t numero_moduli_carichi=0;

  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    modulo_carico=true;
    /* true-> tutte cariche */
    /* false-> non tutte cariche */
    for (uint8_t current_ch = 0; current_ch < TOTAL_CH; current_ch++) {
      if ((current_ch == unused_ch_1)||(current_ch == unused_ch_2)) {}//le trascuro
      /*aggiorno la tensione massima per ogni IC*/
      top_voltage[current_ic]=IsTop(top_voltage[current_ic],bms_ic[current_ic].cells.c_codes[current_ch]);
      /*controllo se la cella non è carica*/
      if(bms_ic[current_ic].cells.c_codes[current_ch] < SogliaCarica){
        modulo_carico=false;                                        //se le celle sono tutte cariche
      }                                                             // allora la variabile rimane true  
      else{
        numero_moduli_carichi++;
        /*bilanciamento finale*/
      }
      if(bms_ic[current_ic].cells.c_codes[current_ch]-top_voltage[current_ic]>delta_carica){
        intermediate_balance(current_ch);
        if(bms_ic[current_ic].cells.c_codes[current_ch]-top_voltage[current_ic]>delta_carica+0.2){
          /*bilanciamento intermedio ma più potente*/
          /*ferma la carica e bilancia*/
          greater_balance(current_ch,RelayPin);
        }
      }
    }
  }
  if(numero_moduli_carichi==TOTAL_IC){
    return stop_charge(RelayPin);
  }
  return true;
}

bool stop_charge(uint8_t pinOut){
  open_relay(pinOut);
  return true;
}

void greater_balance(int8_t cella,uint8_t pinOut){
  open_relay(pinOut);
  set_discharge(cella);
}

void intermediate_balance(int8_t cella){
  set_discharge(cella);
}

void gpio_measurment(){
  wakeup_sleep(TOTAL_IC);                                             //converte gpio
  ltc6813_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
  ltc6813_pollAdc();
  wakeup_sleep(TOTAL_IC);
  ltc6813_rdaux(0,TOTAL_IC,bms_ic);                                   // questo serve a caso
}

float ReadTempGrad (uint8_t pin,uint8_t current_ic){                  //solo il pin passato             
  gpio_measurment();                                                  //e l'IC passato
  float Vout = bms_ic[current_ic].aux.a_codes[pin]*0.0001;
  float Vref2=bms_ic[current_ic].aux.a_codes[5]*0.0001;
  float Rntc = ((Resistenza * Vref2) / Vout) - Resistenza;
  float Rdif = Rntc / Resistenza;
  float Peppa = log(Rdif);
  float B = BB1 * Peppa;
  float C = CC1 * (pow(Peppa,2));
  float D = D1 * (pow(Peppa,3));
  float sum = AA1 + B + C + D;
  /*--un po debug--*/
  Serial.print("Rntc: ");
  Serial.println(Rntc);
  Serial.print("Vref2: ");
  Serial.println(Vref2,4);
  Serial.print("Vout: ");
  Serial.println(Vout,4);
  float Temp = (pow(sum, -1)-274);
  return (Temp);
}

void set_discharge(int8_t cella){
  ltc6813_set_discharge(cella,TOTAL_IC,bms_ic);
  wakeup_sleep(TOTAL_IC);
  ltc6813_wrcfg(TOTAL_IC,bms_ic);
  ltc6813_wrcfgb(TOTAL_IC,bms_ic);
}

void reset_discharge(){
  clear_discharge(TOTAL_IC,bms_ic);
  wakeup_sleep(TOTAL_IC);
  ltc6813_wrcfg(TOTAL_IC,bms_ic);
  ltc6813_wrcfgb(TOTAL_IC,bms_ic);
}

void open_relay(uint8_t relay){
  digitalWrite(relay, HIGH);
}

void close_relay(uint8_t relay){
  digitalWrite(relay, LOW);
}

void PrintTempGrad (){                                                //misura tutte e le stampa
  gpio_measurment();
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
   
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    for (int i=0; i < 5; i++)
    {
      Serial.print(F(" GPIO-"));
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
      Serial.print(",");
    }
    Serial.print(F(" Vref2"));
    Serial.print(":");
    Serial.print(bms_ic[current_ic].aux.a_codes[5]*0.0001,4);
    for (int i=6; i < 9; i++)
    {
      Serial.print(F(", GPIO-"));
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
      Serial.println();
    }
  Serial.print("AUX, ");
    for (int i=0; i < 6; i++)
    {
      Serial.print(bms_ic[current_ic].aux.a_codes[i]*0.0001,4);
      Serial.print(",");
    }
  }
  Serial.println();
}

uint8_t IsTop(uint8_t top,uint8_t actual){                            //ritorna il valore più grande
  if (top>actual){
    return top;
  }
  return actual;
}

void voltage_measurment(){
  wakeup_sleep(TOTAL_IC);
  ltc6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  uint8_t conv_time = ltc6813_pollAdc();
  Serial.print(F("cell conversion completed in:"));
  Serial.print(((float)conv_time / 1000), 1);
  Serial.println(F("mS"));
  Serial.println();
  ltc6813_rdcv(0, TOTAL_IC, bms_ic);                                  //Set to read back all cell voltage registers
}

void print_cells_debug(){
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    Serial.print(", ");
    for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++)
    {
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(bms_ic[current_ic].cells.c_codes[i]*0.0001,4); 
      Serial.print("(");
      Serial.print(error_OV[current_ic][i].flag);                     //stampa il flag per ogni cella per debug
      Serial.print(")");
      Serial.print(",");
    }
    Serial.println();
  }
  Serial.println();
}

