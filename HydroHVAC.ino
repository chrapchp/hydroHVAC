/**
*  @file    hydroHVAC.ino
*  @author  peter c
*  @date    10/28/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  HVAC Arduino Nano to work with custom PC board
*  a left and right unit exists. Code is identical except
*  for the modbus station address and tagnames.
*  Code for the left is used and comment on which tag is for
*  right is included next to var declarations to avoid
*  maintaining two different code bases
*
** @section HISTORY
** 2017Oct28 - created
*/
#include <HardwareSerial.h>

#include <Streaming.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <flowmeter.h>
#include <DA_NonBlockingDelay.h>


#include "UnitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
// #define TRACE_1WIRE
//#define TRACE_ANALOGS
 //#define TRACE_FLOW_SENSOR
// #define TRACE_DISCRETES
//#define TRACE_MODBUS_COILS
//#define TRACE_MODBUS_HR
// comment//out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate

// 
// 
#define FLOW_SENSOR_INTERUPT 2 // pin
#define FLOW_CALC_PERIOD_SECONDS 1 // flow rate calc period
#define ENABLE_FLOW_SENSOR_INTERRUPTS attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_INTERUPT), onB1R1_1A_FT_001_PulseIn, RISING)
#define DISABLE_FLOW_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_INTERUPT))
#define FAN_SPEED_INTERUPT 2 // pin does not work return 0 for now to host
#define HUMIDITY_RETURN_FEED 7 // pin
#define HUMIDITY_INTAKE_FEED 9 // pin


// DHT-22 - one wire type humidity sensor (won't work with one wire lib)
DHT B1R1_1A_DHT_INTAKE = DHT(HUMIDITY_INTAKE_FEED, DHT22);
float B1R1_1A_AT_005 = NAN; // or B1R1-1A-AT-007 for right
float B1R1_1A_TT_012 = NAN; // or B1R1-1A-TT-014 for right
// float HT_101HI = NAN; // humidex
DHT B1R1_1A_DHT_RETURN = DHT(HUMIDITY_RETURN_FEED, DHT22);
float B1R1_1A_AT_006 = NAN; // or B1R1-1A-AT-008 for right
float B1R1_1A_TT_013 = NAN; // or B1R1-1A-TT-015 for right


// or right B1R1-1A-FT-002
FlowMeter B1R1_1A_FT_001(FLOW_SENSOR_INTERUPT, FLOW_CALC_PERIOD_SECONDS); // interrupt pin, calculation period in seconds
DA_AnalogInput B1R1_1A_PDT_008 = DA_AnalogInput(A3, 0.0, 1023.); // min max

// PWM control
#define B1R1_1A_SY_004_PIN 10 // Pin 10 for PWM to control fan speed
unsigned short B1R1_1A_SY_004 = 0; // current duty cycle to write to fan


DA_DiscreteOutput B1R1_1A_XY_001 = DA_DiscreteOutput(12, LOW); // V1


// HEARTBEAT
unsigned int heartBeat = 0;


// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( POLL_CYCLE_SECONDS*1000, &doOnPoll);
DA_NonBlockingDelay flowRateTimer = DA_NonBlockingDelay( FLOW_CALC_PERIOD_SECONDS*1000, &doOnCalcFlowRate);



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = &Serial2;
#endif

void onB1R1_1A_FT_001_PulseIn()
{
  B1R1_1A_FT_001.handleFlowDetection();
}

void onEdgeDetect(bool state, int pin)
{

#ifdef PROCESS_TERMINAL
  *tracePort << "Edge Detection:" << state << " on pin " << pin << endl;
#endif

}


void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  pinMode(B1R1_1A_SY_004, OUTPUT);
  randomSeed(analogRead(3));

  // humidity sensors start
  B1R1_1A_DHT_INTAKE.begin();
  B1R1_1A_DHT_RETURN.begin();
  ENABLE_FLOW_SENSOR_INTERRUPTS;
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
  pollTimer.refresh();
  flowRateTimer.refresh();


  analogWrite(B1R1_1A_SY_004_PIN, B1R1_1A_SY_004);
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  doReadAnalogs();
  B1R1_1A_AT_005 = B1R1_1A_DHT_INTAKE.readHumidity();
  B1R1_1A_TT_012 = B1R1_1A_DHT_INTAKE.readTemperature();
  B1R1_1A_AT_006 = B1R1_1A_DHT_RETURN.readHumidity();
  B1R1_1A_TT_013 = B1R1_1A_DHT_RETURN.readTemperature();
   #ifdef TRACE_1WIRE
  *tracePort << "B1R1_1A_PDT_008:" << B1R1_1A_AT_005  ; 
  *tracePort << " B1R1_1A_TT_012:" << B1R1_1A_TT_012  ;
    *tracePort << " B1R1_1A_AT_006:" << B1R1_1A_AT_006  ; 
  *tracePort << " B1R1_1A_TT_013:" << B1R1_1A_TT_013  << endl;
  #endif
  heartBeat++;
}

void doOnCalcFlowRate()
{
  DISABLE_FLOW_SENSOR_INTERRUPTS;
  B1R1_1A_FT_001.end();

  #ifdef TRACE_FLOW_SENSOR
    B1R1_1A_FT_001.serialize( tracePort, true);
  #endif
  B1R1_1A_FT_001.begin();
  ENABLE_FLOW_SENSOR_INTERRUPTS;
  // resetTotalizers();
}

void doReadAnalogs()
{
  B1R1_1A_PDT_008.refresh();

#ifdef TRACE_ANALOGS
  B1R1_1A_PDT_008.serialize(tracePort, true);
#endif

}

// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{

  modbusRegisters[HR_DIFFDP1] = B1R1_1A_PDT_008.getRawSample();
  modbusRegisters[HR_HUMIDITY1] = B1R1_1A_AT_005 * 100;
  modbusRegisters[HR_TEMPERATURE1] = B1R1_1A_TT_012 * 100;
  modbusRegisters[HR_HUMIDITY2] = B1R1_1A_AT_006 * 100;
  modbusRegisters[HR_TEMPERATURE2] = B1R1_1A_TT_013 * 100;
  modbusRegisters[HR_FLOW1] = B1R1_1A_FT_001.getCurrentPulses();
  modbusRegisters[B1R1_1A_ST_004] = -1; // not implemented
    modbusRegisters[HR_HEARTBEAT] = heartBeat;

}

bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkAndActivateDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->activate();

  #ifdef TRACE_MODBUS_COILS
    *tracePort << "Activate DO:";
    aDO->serialize(tracePort, true);
   // LED.activate();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (!getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->reset();

  #ifdef TRACE_MODBUS_COILS
    *tracePort << "Reset DO:";
    aDO->serialize(tracePort, true);
    //LED.reset();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}
void processValveCommands()

{
  checkAndActivateDO(VALVE1_OPEN_CLOSE, &B1R1_1A_XY_001);
  checkAndResetDO(VALVE1_OPEN_CLOSE, &B1R1_1A_XY_001);
}

void processSetFanSpeedCommand()
{
  B1R1_1A_SY_004 = modbusRegisters[B1R1_1A_SY_004S];
    #ifdef TRACE_MODBUS_HR
  *tracePort << "Set Fan Speed :" << B1R1_1A_SY_004 << endl;
  #endif
}

void processModbusCommands()
{
  processValveCommands();
  processSetFanSpeedCommand();
}

#endif
