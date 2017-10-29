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
#include <TimeAlarms.h>
#include <Streaming.h>
#include <Adafruit_Sensor.h>
#include <DHT.h> // DHT-22 humidity sensor
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#include <flowmeter.h>
#include "UnitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
#define ALARM_REFRESH_INTERVAL 100 // ms
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


// FIND REAL PIN !!!!!!
DA_DiscreteOutput B1R1_1A_XY_001 = DA_DiscreteOutput(3, LOW); // V1

#ifdef PROCESS_TERMINAL
DA_DiscreteOutput LED = DA_DiscreteOutput(13, HIGH); // for debugging
#endif

// HEARTBEAT
unsigned int heartBeat = 0;
struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
  bool firstTime = true;
};

typedef _AlarmEntry AlarmEntry;
AlarmEntry onRefreshAnalogs; // sonar and 1-wire read refresh
AlarmEntry onFlowCalc; // flow calculations

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

int polling; // 1=polling analogs, 2=polling digitals, -1 nothing
int currentTogglePin; // -1 none, >0 pin
void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(19200);
#endif

  pinMode(B1R1_1A_SY_004, OUTPUT);
  randomSeed(analogRead(3));
  onRefreshAnalogs.id = Alarm.timerRepeat(POLL_CYCLE_SECONDS, doOnPoll);
  onFlowCalc.id = Alarm.timerRepeat(FLOW_CALC_PERIOD_SECONDS, doOnCalcFlowRate);
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

  Alarm.delay(ALARM_REFRESH_INTERVAL);
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
  heartBeat++;
}

void doOnCalcFlowRate()
{
  DISABLE_FLOW_SENSOR_INTERRUPTS;
  B1R1_1A_FT_001.end();
  // FT_002.serialize( tracePort, true);
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
  modbusRegisters[HR_HUMIDITY1] = B1R1_1A_AT_005;
  modbusRegisters[HR_TEMPERATURE1] = B1R1_1A_TT_012;
  modbusRegisters[HR_HUMIDITY2] = B1R1_1A_AT_006;
  modbusRegisters[HR_TEMPERATURE2] = B1R1_1A_TT_013;
  modbusRegisters[HR_FLOW1] = B1R1_1A_FT_001.getCurrentFlowRate();
  modbusRegisters[B1R1_1A_ST_004] = -1; // not implemented
  /*
  modbusRegisters[HR_TEMPERATURE1] = 1000;
  modbusRegisters[HR_TEMPERATURE2] = 1001;
  modbusRegisters[HR_TEMPERATURE3] = 1002;
  modbusRegisters[HR_TEMPERATURE4] = 1003;
  modbusRegisters[HR_PRESSURE1] = 1004;
  modbusRegisters[HR_PRESSURE2] = 1005;
  modbusRegisters[HR_PRESSURE3] = 1006;
  modbusRegisters[HR_PRESSURE4] = 1007;
  modbusRegisters[HR_HUMIDITY1] = 1109;
  modbusRegisters[HR_SPARE1] = 1008;
  modbusRegisters[HR_HUMIDITY2] = 1009;
  modbusRegisters[HR_SPARE2] = 1010;
  modbusRegisters[HR_DIFFDP1] = 1011;
  modbusRegisters[HR_DIFFDP2] = 1111;
  modbusRegisters[HR_SPARE1] = 1012;
  modbusRegisters[HR_SPARE2] = 1013;
  modbusRegisters[HR_SPARE3] = 1014;
  modbusRegisters[HR_SPARE4] = 1015;
  modbusRegisters[HR_SPARE5] = 1016;
  modbusRegisters[HR_HEARTBEAT] = 1017;
  */
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
    aDO -> activate();

  #ifdef TRACE_MODBUS
    *tracePort << "Activate DO:";
    aDO -> serialize(tracePort, true);
    LED.activate();
  #endif

    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO -> reset();

  #ifdef TRACE_MODBUS
    *tracePort << "Reset DO:";
    aDO -> serialize(tracePort, true);
    LED.reset();
  #endif

    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void processValveCommands()
{
  checkAndActivateDO(VALVE1_OPEN, &B1R1_1A_XY_001);
  checkAndResetDO(VALVE1_CLOSE, &B1R1_1A_XY_001);
}

void processSetFanSpeedCommand()
{
  B1R1_1A_SY_004 = modbusRegisters[B1R1_1A_SY_004S];
}

void processModbusCommands()
{
  processValveCommands();
  processSetFanSpeedCommand();
}

#endif
