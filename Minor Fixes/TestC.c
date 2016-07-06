
// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <file.h>
#include "DSP28x_Project.h"
#include "sci_io.h"

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5
#pragma CODE_SECTION(mainISR,"ramfuncs");


// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdateA = true;
bool Flag_Latch_softwareUpdateB = true;

CTRL_Handle ctrlHandleA;
CTRL_Handle ctrlHandleB;

// HAL Handle
HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmDataA = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
HAL_PwmData_t gPwmDataB = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcDataA;
HAL_AdcData_t gAdcDataB;

int gRunB = 1; // WTF is this

FEM_Handle femHandleA;
FEM_Obj    femA;
FEM_Handle femHandleB;
FEM_Obj    femB;

volatile long gCounter = 0;
uint32_t   gNumFreqErrorsA = 0;
uint32_t   gMaxDeltaCntObservedA = 0;
uint32_t   gNumFreqErrorsB = 0;
uint32_t   gMaxDeltaCntObservedB = 0;

CPU_USAGE_Handle cpu_usageHandle;
CPU_USAGE_Obj    cpu_usage;
float_t          gCpuUsagePercentageMin = 0.0;
float_t          gCpuUsagePercentageAvg = 0.0;
float_t          gCpuUsagePercentageMax = 0.0;

_iq gMaxCurrentSlopeA = _IQ(0.0);
_iq gMaxCurrentSlopeB = _IQ(0.0);

// Controller
#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_objA;
CTRL_Obj *controller_objB;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

uint16_t gLEDcnt = 0;

// Motor initial Variables
volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;
volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT; 

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
// Dual Motors 
DRV_SPI_8301_Vars_t gDrvSpi8301VarsA;
DRV_SPI_8301_Vars_t gDrvSpi8301VarsB;
#endif


volatile bool Flag_Enable_Inverter = false;
volatile bool Flag_is_Inverter_Enabled = false;
_iq FlyingStartSpeedRef_krpm = _IQ(2.0);
_iq FlyingStartInitSpeedRef_krpm = _IQ(2.0);
uint_least32_t TorqueModeCounter = 0;
uint_least32_t TorqueModeCounterMaxCnt = (uint_least32_t)(0.05 * USER_ISR_FREQ_Hz);
MATH_vec2 gVdq_in;




// **************************************************************************
// the functions

void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);


  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // initialize the frequency of execution monitoring module
  femHandle = FEM_init(&fem,sizeof(fem));
  FEM_setParams(femHandle,
                USER_SYSTEM_FREQ_MHz * 1000000.0,                  // timer frequency, Hz
                (uint32_t)USER_SYSTEM_FREQ_MHz * 1000000,          // timer period, cnts
                USER_CTRL_FREQ_Hz,                                 // set point frequency, Hz
                1000.0);                                           // max frequency error, Hz


  // initialize the CPU usage module
  cpu_usageHandle = CPU_USAGE_init(&cpu_usage,sizeof(cpu_usage));
  CPU_USAGE_setParams(cpu_usageHandle,
                     (uint32_t)USER_SYSTEM_FREQ_MHz * 1000000,     // timer period, cnts
                     (uint32_t)USER_ISR_FREQ_Hz);                  // average over 1 second of ISRs


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);


  // reload timer to start running frequency of execution monitoring
  HAL_reloadTimer(halHandle,0);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle, 0);
  HAL_enableDrv(halHandle, 1);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsA, 0);
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsB, 1);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandleA, true);
  CTRL_setFlag_enableDcBusComp(ctrlHandleB, true);

  // scaling for torque
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

  // disable offsets recalibration by default
  gMotorVarsA.Flag_enableOffsetcalc = true;
  gMotorVarsB.Flag_enableOffsetcalc = true;
  /*
  gMotorVarsA.Flag_enableRsRecalc = true;
  gMotorVarsB.Flag_enableRsRecalc = true;
  gMotorVarsA.Flag_enableSys = true;
  gMotorVarsB.Flag_enableSys = true;
  gMotorVarsA.Flag_Run_Identify = true;
  gMotorVarsB.Flag_Run_Identify = true;
  gMotorVarsA.SpeedRef_krpm = _IQ(0.0);
  gMotorVarsB.SpeedRef_krpm = _IQ(0.0);
  gMotorVarsA.MaxAccel_krpmps = _IQ(0.0);
  gMotorVarsB.MaxAccel_krpmps = _IQ(0.0);
*/
  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVarsA.Flag_enableSys));
    while(!(gMotorVarsB.Flag_enableSys));

    CTRL_getFlag_enableSpeedCtrl(ctrlHandleA, true);
    CTRL_getFlag_enableSpeedCtrl(ctrlHandleB, true);

    // loop while the enable system flag is true
    while(gMotorVarsA.Flag_enableSys && gMotorVarsB.Flag_enableSys)
      {
        CTRL_Obj *objA = (CTRL_Obj *)ctrlHandleA;
        CTRL_Obj *objB = (CTRL_Obj *)ctrlHandleB;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandleA,true);
        CTRL_setFlag_enableUserMotorParams(ctrlHandleB,true);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(objA->estHandle,false);
        EST_setFlag_enableRsRecalc(objB->estHandle,false);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandleA,false);
        CTRL_setFlag_enableOffset(ctrlHandleB,false);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(objA->estHandle,false);
        EST_setFlag_enableForceAngle(objB->estHandle,false);

        if(CTRL_isError(ctrlHandleA) || CTRL_isError(ctrlHandleB))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandleA,false);
            CTRL_setFlag_enableCtrl(ctrlHandleB,false);

            // set the enable system flag to false
            gMotorVarsA.Flag_enableSys = false;
            gMotorVarsB.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
          
        else
          {
            // update the controller state
            bool flag_ctrlStateChangedA = CTRL_updateState(ctrlHandle);
            bool flag_ctrlStateChangedB = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandleA, gMotorVarsA.Flag_Run_Identify);
            CTRL_setFlag_enableCtrl(ctrlHandleB, gMotorVarsB.Flag_Run_Identify);

            if(flag_ctrlStateChangedA)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OnLine)
                  {
                    // set the current bias
                    HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                    // set the voltage bias
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }
              }

              if(flag_ctrlStateChangedB)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OnLine)
                  {
                    // set the current bias
                    HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

                    // set the voltage bias
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
                    HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }
              }
          }



        if(EST_isMotorIdentified(objA->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(objA->estHandle,gMaxCurrentSlope);
            gMotorVarsA.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandleA,gMotorVarsA.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandleA,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            if(Flag_Latch_softwareUpdateA)
            {
              Flag_Latch_softwareUpdateA = false;

#ifdef FAST_ROM_V1p6
              if(CTRL_getFlag_enableUserMotorParams(ctrlHandleA) == true)
              {
                  // call this function to fix 1p6
                  softwareUpdate1p6(ctrlHandle);
              }
#endif

              calcPIgains(ctrlHandleA);
            }

          }
        else
          {
            Flag_Latch_softwareUpdateA = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(objA->estHandle);
          }


        if(EST_isMotorIdentified(objB->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(objB->estHandle,gMaxCurrentSlope);
            gMotorVarsB.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandleB,gMotorVarsB.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandleB,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            if(Flag_Latch_softwareUpdateB)
            {
              Flag_Latch_softwareUpdateB = false;

#ifdef FAST_ROM_V1p6
              if(CTRL_getFlag_enableUserMotorParams(ctrlHandleB) == true)
              {
                  // call this function to fix 1p6
                  softwareUpdate1p6(ctrlHandleB);
              }
#endif

              calcPIgains(ctrlHandleB);
            }

          }
        else
          {
            Flag_Latch_softwareUpdateB = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(objB->estHandle);
          }



        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandleA);
            updateGlobalVariables_motor(ctrlHandleB);
          }

        // get the maximum delta count observed
        gMaxDeltaCntObserved = FEM_getMaxDeltaCntObserved(femHandle);

        // check for errors
        if(FEM_isFreqError(femHandleA))
          {
            gNumFreqErrorsA = FEM_getErrorCnt(femHandle);
          }

        if(FEM_isFreqError(femHandleB))
          {
            gNumFreqErrorsB = FEM_getErrorCnt(femHandle);
          }

        // update CPU usage
        updateCPUusage(ctrlHandleA);
        updateCPUusage(ctrlHandleB);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(objA->estHandle,gMotorVarsA.Flag_enableForceAngle);
        EST_setFlag_enableForceAngle(objB->estHandle,gMotorVarsB.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandleA,gMotorVarsA.Flag_enablePowerWarp);
        CTRL_setFlag_enablePowerWarp(ctrlHandleB,gMotorVarsB.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandleA,&gDrvSpi8301VarsA);
        HAL_writeDrvData(halHandleB,&gDrvSpi8301VarsB);
        HAL_readDrvData(halHandleA,&gDrvSpi8301VarsA);
        HAL_readDrvData(halHandleB,&gDrvSpi8301VarsB);
#endif

      } // end of while(gFlag_enableSys) loop



    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandleA,&gUserParams);
    CTRL_setParams(ctrlHandleB,&gUserParams);

    gMotorVarsA.Flag_Run_Identify = false;
    gMotorVarsB.Flag_Run_Identify = false;

  } // end of for(;;) loop
} // end of main() function

// Might have to rewrite because not sure if for one or two
interrupt void mainISR(void)
{
  uint32_t timer0Cnt;
  uint32_t timer1Cnt;


  // read the timer 1 value and update the CPU usage module
  timer1Cnt = HAL_readTimerCnt(halHandle,1);
  CPU_USAGE_updateCnts(cpu_usageHandle,timer1Cnt);


  // read the timer 0 value and update the FEM
  timer0Cnt = HAL_readTimerCnt(halHandle,0);
  FEM_updateCnts(femHandle,timer0Cnt);
  FEM_run(femHandle);


  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }


  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandleA,ADC_IntNumber_1);
  HAL_acqAdcInt(halHandleB,ADC_IntNumber_1);
  // convert the ADC data
  HAL_readAdcData(halHandleA,&gAdcData);
  HAL_readAdcData(halHandleB,&gAdcData);
  // run the controller
  CTRL_run(ctrlHandleA,halHandleA,&gAdcData,&gPwmData);
  CTRL_run(ctrlHandleB,halHandleB,&gAdcData,&gPwmData);

  if(CTRL_getState(ctrlHandleA) == CTRL_State_OnLine)
    {
      if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == true))
        {
  	      // disable the PWM
          HAL_disablePwm(halHandleA);

  	      Flag_is_Inverter_Enabled = false;
        }
      else if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == false))
        {
          CTRL_Obj *objA = (CTRL_Obj *)ctrlHandleA;

    	  if((gMotorVarsA.Speed_krpm > _IQ(0.0)) && (gMotorVarsA.Speed_krpm < FlyingStartSpeedRef_krpm))
            {
              Flag_Enable_Inverter = true;
            }
          else if((gMotorVarsA.Speed_krpm < _IQ(0.0)) && (gMotorVarsA.Speed_krpm > FlyingStartSpeedRef_krpm))
            {
              Flag_Enable_Inverter = true;
            }


  if(CTRL_getState(ctrlHandleB) == CTRL_State_OnLine)
    {
      if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == true))
        {
          // disable the PWM
          HAL_disablePwm(halHandleB);

          Flag_is_Inverter_Enabled = false;
        }
      else if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == false))
        {
          CTRL_Obj *objB = (CTRL_Obj *)ctrlHandleB;

        if((gMotorVarsB.Speed_krpm > _IQ(0.0)) && (gMotorVarsB.Speed_krpm < FlyingStartSpeedRef_krpm))
            {
              Flag_Enable_Inverter = true;
            }
          else if((gMotorVarsB.Speed_krpm < _IQ(0.0)) && (gMotorVarsB.Speed_krpm > FlyingStartSpeedRef_krpm))
            {
              Flag_Enable_Inverter = true;
            }





          // Reset Speed, Id and Iq integral outputs Ui
          PID_setUi(obj->pidHandle_Id,gVdq_in.value[0]);
          PID_setUi(obj->pidHandle_Iq,gVdq_in.value[1]);

          CTRL_setVdq_out_pu(ctrlHandle,&gVdq_in);

          CTRL_setFlag_enableSpeedCtrl(ctrlHandle,false);

          TorqueModeCounter = 0;
        }
      else if((Flag_Enable_Inverter == true)&&(Flag_is_Inverter_Enabled == true)&&(CTRL_getFlag_enableSpeedCtrl(ctrlHandle) == false))
        {
    	  TorqueModeCounter++;
          if(TorqueModeCounter > TorqueModeCounterMaxCnt)
            {
              CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

              PID_setUi(obj->pidHandle_spd,_IQ(0.0));

        	  // Set Speed Reference target and intermediate values to estimated speed
        	  TRAJ_setIntValue(obj->trajHandle_spd, EST_getFm_pu(obj->estHandle));
        	  gMotorVars.SpeedRef_krpm = FlyingStartInitSpeedRef_krpm; //EST_getSpeed_krpm(obj->estHandle)

              // set the speed reference
              CTRL_setSpd_ref_krpm(ctrlHandle,gMotorVars.SpeedRef_krpm);

        	  CTRL_setFlag_enableSpeedCtrl(ctrlHandle,true);
            }
        }
      else if((Flag_Enable_Inverter == true)&&(Flag_is_Inverter_Enabled == false))
        {
          // enable the PWM
          HAL_enablePwm(halHandle);

          Flag_is_Inverter_Enabled = true;
        }
    }
  else if(CTRL_getState(ctrlHandle) == CTRL_State_Idle)
    {
      Flag_Enable_Inverter = false;
      Flag_is_Inverter_Enabled = false;
    }


  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  // setup the controller
  CTRL_setup(ctrlHandle);


  // read the timer 1 value and update the CPU usage module
  timer1Cnt = HAL_readTimerCnt(halHandle,1);
  CPU_USAGE_updateCnts(cpu_usageHandle,timer1Cnt);


  // run the CPU usage module
  CPU_USAGE_run(cpu_usageHandle);


  return;
} // end of mainISR() function


void updateGlobalVariables_motor(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint32_t minDeltaCntObserved = CPU_USAGE_getMinDeltaCntObserved(cpu_usageHandle);
  uint32_t avgDeltaCntObserved = CPU_USAGE_getAvgDeltaCntObserved(cpu_usageHandle);
  uint32_t maxDeltaCntObserved = CPU_USAGE_getMaxDeltaCntObserved(cpu_usageHandle);
  uint16_t pwmPeriod = HAL_readPwmPeriod(halHandle,PWM_Number_1);
  float_t  cpu_usage_den = (float_t)pwmPeriod * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * 2.0;

  // get the speed estimate
  gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVars.Torque_lbin = calcTorque_lbin(handle);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // calculate the minimum cpu usage percentage
  gCpuUsagePercentageMin = (float_t)minDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the average cpu usage percentage
  gCpuUsagePercentageAvg = (float_t)avgDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the maximum cpu usage percentage
  gCpuUsagePercentageMax = (float_t)maxDeltaCntObserved / cpu_usage_den * 100.0;

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function


#ifdef FAST_ROM_V1p6
void softwareUpdate1p6(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t fullScaleInductance = EST_getFullScaleInductance(obj->estHandle);
  float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(obj->estHandle));
  int_least8_t lShift = ceil(log(obj->motorParams.Ls_d_H/(Ls_coarse_max*fullScaleInductance))/log(2.0));
  uint_least8_t Ls_qFmt = 30 - lShift;
  float_t L_max = fullScaleInductance * pow(2.0,lShift);
  _iq Ls_d_pu = _IQ30(obj->motorParams.Ls_d_H / L_max);
  _iq Ls_q_pu = _IQ30(obj->motorParams.Ls_q_H / L_max);


  // store the results
  EST_setLs_d_pu(obj->estHandle,Ls_d_pu);
  EST_setLs_q_pu(obj->estHandle,Ls_q_pu);
  EST_setLs_qFmt(obj->estHandle,Ls_qFmt);

  return;
} // end of softwareUpdate1p6() function
#endif


void calcPIgains(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  float_t Ls_d = EST_getLs_d_H(obj->estHandle);
  float_t Ls_q = EST_getLs_q_H(obj->estHandle);
  float_t Rs = EST_getRs_Ohm(obj->estHandle);
  float_t RoverLs_d = Rs/Ls_d;
  float_t RoverLs_q = Rs/Ls_q;
  float_t fullScaleCurrent = EST_getFullScaleCurrent(obj->estHandle);
  float_t fullScaleVoltage = EST_getFullScaleVoltage(obj->estHandle);
  float_t ctrlPeriod_sec = CTRL_getCtrlPeriod_sec(handle);
  _iq Kp_Id = _IQ((0.25*Ls_d*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  _iq Ki_Id = _IQ(RoverLs_d*ctrlPeriod_sec);
  _iq Kp_Iq = _IQ((0.25*Ls_q*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  _iq Ki_Iq = _IQ(RoverLs_q*ctrlPeriod_sec);
  _iq Kd = _IQ(0.0);

  // set the Id controller gains
  PID_setKi(obj->pidHandle_Id,Ki_Id);
  CTRL_setGains(handle,CTRL_Type_PID_Id,Kp_Id,Ki_Id,Kd);

  // set the Iq controller gains
  PID_setKi(obj->pidHandle_Iq,Ki_Iq);
  CTRL_setGains(handle,CTRL_Type_PID_Iq,Kp_Iq,Ki_Iq,Kd);

  return;
} // end of calcPIgains() function


float_t calcTorque_Nm(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Id_A = _IQtoF(PID_getFbackValue(obj->pidHandle_Id)) * USER_IQ_FULL_SCALE_CURRENT_A;
  float_t Iq_A = _IQtoF(PID_getFbackValue(obj->pidHandle_Iq)) * USER_IQ_FULL_SCALE_CURRENT_A;
  float_t Polepairs = (float_t)USER_MOTOR_NUM_POLE_PAIRS;
  float_t Flux_Wb = EST_getFlux_Wb(obj->estHandle);
  float_t Lsd_H = EST_getLs_d_H(obj->estHandle);
  float_t Lsq_H = EST_getLs_q_H(obj->estHandle);

  return((Flux_Wb * Iq_A + (Lsd_H - Lsq_H) * Id_A * Iq_A) * Polepairs * 1.5);
} // end of calcTorque_Nm() function


float_t calcTorque_lbin(CTRL_Handle handle)
{

  return(calcTorque_Nm(handle) * MATH_Nm_TO_lbin_SF);
} // end of calcTorque_lbin() function


//@} //defgroup
// end of file
