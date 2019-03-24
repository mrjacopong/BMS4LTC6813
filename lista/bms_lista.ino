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
const uint8_t TOTAL_IC = 8;//!<number of ICs in the daisy chain
const uint8_t TOTAL_CH = 18; // number of channel used per ADC
const uint8_t TOTAL_NTC = 8; // number of temperatures per ADC

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
  int8_t id;                   //identificatore temporanea
  int8_t posiz;                //posizione dell'errore
  /*sarà un numero che conta tutte le celle/ntc, 
  ed ogniuno avrà il proprio numero seriale
  questo serve perchè abbiamo una lista dinamica*/
  unsigned long time;          //momento in cui si verifica il primo errore
  control* succ;
};
typedef control* lista;
lista primo_elemento=0;   //inizializzo il primo puntatore della lista a 0
control elemento; //elemento generico della lista

void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  ltc681x_init_cfg(TOTAL_IC, bms_ic);
  ltc6813_reset_crc_count(TOTAL_IC, bms_ic);
  ltc6813_init_reg_limits(TOTAL_IC, bms_ic);
  init_problem (error_OV);
}

void loop(){
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint8_t user_command;
  int8_t readIC = 0;
  char input = 0;
  //--interfaccia utente temporanea--//
  if (Serial.available()){           // Check for user inpu
    uint8_t user_command;
    Serial.println("inserisci un numero qualsiasi per lanciare il programma");
    user_command = read_int();      // whait for a key
  
   //---------------------------------//
 
    voltage_measurment(); //leggo le tensioni dall'adc

    float cell_v[TOTAL_IC][TOTAL_CH];
    read_voltages(cell_v);
    print_voltages(cell_v);//--interfaccia utente temporanea--//
  }
}



/******************************************************************************
FUNCTIONS
*******************************************************************************/
void new_error(lista& inizio,control e){      //creare un elemento della lista errori
  lista p = 0, q, r;
  for (q = inizio; q != 0; q = q->succ) p = q;
  r = new control;
 *r = e;
  r->succ = q;
  if (q == inizio) inizio = r;
  else p->succ = r;
}

bool del_error(lista& inizio, control& e){    //elimina un elemento della lista errori
  lista p=0, q;
  for (q=inizio; q!=0 ; q=q->succ) p=q;
  e = *q;
  if (q==0) return false;
  if (q==inizio) inizio = q->succ;
  else p->succ=q->succ;
  delete q;
  return true;
}


void error_check(){                   //controllo degli errori
  for(uint8_t current_ic=0;current_ic<TOTAL_IC;current_ic++){
		for(uint8_t current_ch=0;current_ch<TOTAL_CH;current_ch++){
      /**overcurrent**/
      double MAXVOLTAGE = OV_THRESHOLD;
      if(bms_ic[current_ic].cells.c_codes[current_ch] * 0.0001 > MAXVOLTAGE){
        for (lista q=primo_elemento; q!=0 || q->posiz != (current_ic*current_ch)+current_ch; q=q->succ){}
        //cerca il nostro errore
        if(q==0) {
          lista primo_elemento=*elemento;
        }
        new_error(q,elemento);         //se c'è un error_OV nuovo lo segno
          q->time=millis();        //e inizializzo il tempo.
          q->flag=1;               //se un error_OV è vecchio non c'è bisogno
        }                            //di inizalizzare 
        else {
        q->flag=0;              //in asssenza di error_OV il flag è zero
        }
    }
      /**overtemperature**/
    for(uint8_t current_ntc=0;current_ntc<TOTAL_NTC;current_ntc++){
      if(temperatura[current_ic][current_ntc] > MAXTEMP){
        if (error_OT[current_ic][current_ntc].flag==0){           //se c'è un error_OV nuovo lo segno 
          error_OT[current_ic][current_ntc].time=millis();        //e inizializzo il tempo.
          error_OT[current_ic][current_ntc].flag=1;               //se un error_OV è vecchio non c'è bisogno
        }                                                       //di inizalizzare
      }
      else error_OT[current_ic][current_ntc].flag=0;              //in asssenza di error_OV il flag è zero
     }
  }
}

void init_problem (control error_OV[][TOTAL_CH]){
  for(uint8_t current_ic=0;current_ic<TOTAL_IC;current_ic++){
		for(uint8_t current_ch=0;current_ch<TOTAL_CH;current_ch++){
			error_OV[current_ic][current_ch].id=current_ch;
			error_OV[current_ic][current_ch].flag=0;
      error_OV[current_ic][current_ch].time=0;
		}
	}
}


void voltage_measurment(){
  wakeup_sleep(TOTAL_IC);
  ltc6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  uint8_t conv_time = ltc6813_pollAdc();
  Serial.print(F("cell conversion completed in:"));
  Serial.print(((float)conv_time / 1000), 1);
  Serial.println(F("mS"));
  Serial.println();
  uint8_t error = ltc6813_rdcv(0, TOTAL_IC, bms_ic); // Set to read back all cell voltage registers
  //check_error(error);
}

void read_voltages(float cell_voltage[][TOTAL_CH]){
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
      for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++) {
        if ((i!=9)&&(i!=18)) {
        int j = 0;
        cell_voltage[current_ic][j] = bms_ic[current_ic].cells.c_codes[j] * 0.0001;
        j++;
        }
      }
    }
  }

void print_voltages(float cell_voltage[][TOTAL_CH]){
  Serial.println();
  for(int j=0;j<2;j++){
    Serial.print(" IC ");
    Serial.print(j,DEC);
    Serial.print(", ");
    for (int i=0;i<TOTAL_CH;i++){
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(cell_voltage[j][i]);
      Serial.print(",");
    }
    Serial.println();
  }
}