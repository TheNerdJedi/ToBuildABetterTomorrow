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
#define TORQUE_MODE_COUNTER_MAX_CNT_AFTER_FS (uint_least32_t)(0.01 * USER_ISR_FREQ_Hz) // super short for FS since speeds are expected to be so low
#define TORQUE_MODE_COUNTER_MAX_CNT (uint_least32_t)(0.05 * USER_ISR_FREQ_Hz) // used to be 0.015 for both, Jorge recommneds 0.05, im trying really high val
#define MY_BUFFER_LEN 20
#define REMOTE_NEGATIVE_THRESHOLD  _IQ(2.0) //asking what out of 0.4 to 23.5 is considered negative speed (so 0.4 to 3.5 is now negative speed or break)  ** changed 3.5 to 2.0
#define REMOTE_FORWAR_THRESHOLD  _IQ(8.0) //#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif so only values above 8 get used for forward speed control. 8.1 is basically translated to 0.1 speed, times scaler ** changed 8.0 to 9.5
#define MAXIMUM_SPEED_KRPM   _IQ(5.6)
#define REMOTE_COMMAND_SCALER  _IQ((float_t)5.6/(23.5 - 8.0))  //substituting de-IQed numbers from above. max speed 5.6, highest incoming value 23.5, remte_forward_threshold 8.0

// **************************************************************************
// the globals XXX
////////options...
bool safetyStop_pingPONG_enabled = true;
bool need_based_battery_pairing_allowed = false;

/////////////////////////////////battery stuff
uint_least8_t bat_show = 6; // represents how many LEDs are on on the display board
uint_least8_t bat_actual = 6; // the believed battery level, at first assumuption
uint_least16_t bat_level[2]; //for tracking both batteries
uint_least8_t current_bat; //these two variables are used as the index for tracking batteries
uint_least8_t other_bat;
bool bat_LED_toggledON = true; //Flag used to keep track of wthere the zero level LED is or or off.
bool batteries_are_equal = false; // assume false at first, if true both batteries become connected and LEIF board has more avaliable juice
bool batteries_are_paired = false; // they are not connected at first
_iq battery_Ah = _IQ(4.6); //max possible is 4.6aH with two battery packs
_iq battery_Ah_half = _IQ(2.3);
uint_least8_t batteryAhEstimator_counter = 0; //time to wait till calling batteryAhEstimator()

/////////////////////////////////timing stuff
volatile uint_least32_t to10thsec_counter = 0; //counted at isr
uint_least8_t tofullsec_counter = 0; //10 means a full second has passed
uint_least8_t  till_batUpdateDisplay_counter = 0; //10 means 10 seconds have passed, may change this to 30 or something

///////////////////////////////////////power button stuff
uint_least16_t power_button_counter = 0; // tracks time that the power button is pressed for

/////////////////////////////////////Motor orientation stuff
uint_least8_t front_motor_index = 1; // 1 is alway default. If the orientation sensor breaks, will default to 1.
uint_least8_t user_mode = 2; //mode 2 is normal mode(back motor has intelligent managemnet)

/////////////////////////////////////Back Motor Power Stuff, only affects the ride in mode 2 (normal mode)
_iq back_power_limit = _IQ(0.7); //  power limit set to the back motor. 0.7 equates to about 70%
_iq back_motorkW = _IQ(0.0); //  a step used in the math to get current
_iq back_motorI[10] = {_IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0),_IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0)};
_iq back_motorI_average = _IQ(0.0); // and average of the last 10 samples
_iq overall_kW = _IQ(0.0);//  a step used in the math to get current
_iq overall_I = _IQ(0.0); //  past 10th sec
uint_least8_t power_index = 0;

/////////////////////////debug and reporting stuff
bool debug = false; //set as true via USB bluettooth dongle, and you gain some ability to control the motor
uint_least16_t power_high_counter =  0;
uint_least16_t power_high_counter_record =  0;
_iq back_power_limit_record = _IQ(0.0);
_iq back_motorI_average_record = _IQ(0.0);
_iq speed_record = _IQ(0.0);
_iq speed_record_demanded = _IQ(0.0);
uint_least8_t battery_calc_was_low = 0;
uint_least8_t battery_calc_was_high = 0;
uint_least8_t battery_calc_was_right = 0;

//////////////////////////////////RX in isr stuff
volatile char my_buffer[MY_BUFFER_LEN];
volatile uint_least8_t my_i=0; //my_buffer index
volatile bool RX_flag = false; //when true, the buffer gets read by proccessUARTbuffer()
bool silence_state = true; // remote has not been sending speed tate
uint_least8_t silence_counter = 200; //counting since the last speed command

////////////////////////////////////////Energy saving stuff
uint_least16_t HALT_counter = 0;  //counting till ready2HALT should be made true
bool ready_2HALT = false; //if true, execute the sleep routine

////////////////////////////////////////Flying Start stuff
volatile bool coasting = true; // if true, disable the controller. when false, enable it
volatile bool Flag_Enable_Inverter = false; // used by flying start to control stages
volatile bool Flag_is_Inverter_Enabled = false;// used by flying start to control stages
volatile uint_least32_t TorqueModeCounter = 0; //counting till timeout in torque mode during fling start
volatile uint_least32_t TorqueModeCounterUseNow; //the time out limit for the above counter
MATH_vec2 gVdq_in; // new variable used by flying start angle calculator, thanks Jorge

///////////////////////////////////////////////Motor speed inputs stuff
// TI Instruments Stuff
_iq remote_command_val = _IQ(0.0); //range is 0 to 25.5
_iq target_speed = _IQ(0.0); //in KRPM for each motor
_iq target_speed_temp = _IQ(0.0); //this is useful for accel calculations and for stabilizing speed demand
_iq current_accel = _IQ(0.0);  // this value step by step approaches target_accel
_iq target_accel = _IQ(0.0); //target KRPM acceleration.
_iq speed_of_faster_motor = _IQ(0.0);  // will represent the faster of the two motors
_iq speed_average_of_motors = _IQ(0.0); //will represent two motor speeds added and divided by 2
bool wheels_are_not_moving = true;
bool accelerating_forward = false;

/////////////////////////////////////////////// Patten Start stuff (custom open loop forced start method)
bool fs_mode = false; // when true, we engage Patten Start
bool fs_recently_used = false;  //for special treatment right after FS ends. Prevents immediate reengage of FS, and allows for extra fast flying start
uint_least16_t fs_counter = 0;
uint_least16_t fsThresholdShort = 60; // this is directly affected by ISR rate, 60 here works well with 12khz
_iq fs_phase = _IQ(0.0); //location of magnets in two radians, variables below now set whenever fsState == 0 in main()
_iq fs_phase_speed; //speed LK was .0005
_iq fs_phase_accelerate; //acceleration LK was .0001
_iq fs_power; // determine power used by fs. Not a real world value like amps
_iq fs_speed_limit = _IQ(0.033); // once this speed is hit, pattenStart is disabled. Works out to about 600 rpm.

//////////////////////////////////////////////Safety Stop stuff, based on bluetooth connection status
bool bluetooth_connected = true;
uint_least8_t bluetooth_disconnected_counter = 0; //counting till a timeout since bluetooth connection was last confirmed
bool safety_stop = false; // if true, the wheel are slowed down to a stop.
uint_least8_t safety_stop_counter = 0; // used as a time limit to how long safety stop can be engaged

///////////////////////////////////////////end of LK's new variable declerations

_iq gMaxCurrentSlope = _IQ(0.0);
uint_least16_t gCounter_updateGlobals = 0;
bool Flag_Latch_softwareUpdateA = true;
bool Flag_Latch_softwareUpdateB = true;

CTRL_Handle ctrlHandleA;
CTRL_Handle ctrlHandleB;

HAL_Handle halHandle; // HAL Handle 

USER_Params gUserParams;

HAL_PwmData_t gPwmDataA = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
HAL_PwmData_t gPwmDataB = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcDataA;
HAL_AdcData_t gAdcDataB;

int gRunB = 1;

FEM_Handle femHandleA;
FEM_Handle femHandleB;

FEM_Obj    femA;
FEM_Obj    femB;

long gCounter = 0;
//volatile long gIntCounter = 0;
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
CTRL_Obj ctrl;        //v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVarsA = MOTOR_Vars_INIT; // Motor A initial variable
volatile MOTOR_Vars_t gMotorVarsB = MOTOR_Vars_INIT; // Motor B initial variable 

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern Uint16 RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301VarsA;
DRV_SPI_8301_Vars_t gDrvSpi8301VarsB; // Dual Motor Setup
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;


// **************************************************************************
// Patten studio custom PROTOTYPES functions 
void initAll(); // setup sysctrl, ADC, button and LED pins. also SCIB: 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity //
void initBatStatus(); //setup the special case LED pins on the display, and turn on all LEDs
void initPowerPins(); //setup the pins used forconnecitng and disconnecting batteries
void pattenStart(); // an open loop forced start of wheels
void batteryAhEstimator(); // estimates battery Ah based purely on Voltage
void batAhSubtractUsed(); // estimates remaining battery Ah by subtracting Ah used by the motors
void batReadlow(); //reads very low voltage on battery and overrides the above two funcitons
void batteryPriorityManager(); //reads voltage on each battery, determines which is priority and if they are equal
void batteryPair(); //if batteries are equal, connects both
void batteryUnPair(); //disconnects the low priority battery
void batUpdateDisplay(); //updates the LEDs on the display based on bat_actual.
void sendString(char s[]);
void captureUART(); //used inside the ISR. Puts characters into the buffer and sets of the RX_flag if a \r is seen
int processUARTbuffer(); //looks for speed commands, and some additional commands like PING, MOVE, DEBUG, STATS
void clearMyBuffer(volatile char s[],int n);
_iq commandScalerToMotor(); //takes  the remote command value and turns it into speed in KRPM
int getOrientation(); //returns th emotor orientation
void delayMS(uint_least16_t thousandsUS); //a delay that is used if interrupts are not enabled
void delayMSinterrupt(uint_least32_t delay_in_ms); //the delay to use whenever interrupt clock timer is enabled. NOTE!! if called when the timer is not running, this fucntion will wait forever

//Revised TI fucntions
void runRsOnLineA(CTRL_Handle handle); //constantly reevaluate the resistance of the motor strator coils to calculate the right amount of power
void runRsOnLineB(CTRL_Handle handle);
void updateIqRefA(CTRL_Handle handle);
void updateIqRefB(CTRL_Handle handle);
void updateKpKiGainsA(CTRL_Handle handle);
void updateKpKiGainsB(CTRL_Handle handle);
void updateGlobalVariables_motorA(CTRL_Handle handle);
void updateGlobalVariables_motorB(CTRL_Handle handle);

//debugg only functions
void ftoa(unsigned char *buf, float f);
void sendStringU(unsigned char s[]);

__interrupt void WAKE_ISR(void);    // ISR for WAKEINT, not actually sure this has to exist

//********************************************************************************************
//////////////////////////////////////////////////////MAIN************************************************************************************************************* 
void main(void)
{
  initPowerPins(); //battery connection setup need to be done right away
  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart); //need for time delay functions
  initAll(); //Starts GPIO, SCI, LEDS, System control
  initBatStatus(); //setup leds on the display board, and turn them all ON

/*   uint_least8_t estNumber = 0;

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
*/

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);

  // store user parameter error in global variable
  gMotorVarsA.UserErrorCode = USER_getErrorCode(&gUserParams);
  gMotorVarsB.UserErrorCode = USER_getErrorCode(&gUserParams);

  // do not allow code execution if there is a user parameter error
  if(gMotorVarsA.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVarsA.Flag_enableSys = false;
        }
    }

  if(gMotorVarsB.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVarsB.Flag_enableSys = false;
        }
    }

  // initialize the user parameters
  USER_setParams(&gUserParams);

  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);
  ScibRegs.SCILBAUD  = 0x005F;/////////////////////////////////////////////////////////////////// LLL Hal_setParam changes clocks speeds, so baudrate needs adjustment here

  // initialize the controller////////////  
#ifdef FAST_ROM_V1p6
  ctrlHandleB = CTRL_initCtrl(1,1);     //v1p6 format (06xF and 06xM devices) (ctrlNumber, estNumber)
  controller_objB = (CTRL_Obj *)ctrlHandleB;
  ctrlHandleA = CTRL_initCtrl(0,0);     //v1p6 format (06xF and 06xM devices)
  controller_objA = (CTRL_Obj *)ctrlHandleA;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl)); //v1p7 format default
#endif

  {
    CTRL_Version versionA;
    CTRL_Version versionB;

    // get the version number
    CTRL_getVersion(ctrlHandleA,&versionA);
    gMotorVarsA.CtrlVersion = versionA;

    CTRL_getVersion(ctrlHandleB,&versionB);
    gMotorVarsB.CtrlVersion = versionB;
  }
  
  // set the default controller parameters
  CTRL_setParams(ctrlHandleA,&gUserParams);
  CTRL_setParams(ctrlHandleB,&gUserParams);


  // initialize the frequency of execution monitoring module
  femHandleA = FEM_init(&femA,sizeof(femA));
  FEM_setParams(femHandleA,
                USER_SYSTEM_FREQ_MHz * 1000000.0,                  // timer frequency, Hz
                (uint32_t)USER_SYSTEM_FREQ_MHz * 1000000,          // timer period, cnts
                USER_CTRL_FREQ_Hz,                                 // set point frequency, Hz
                1000.0);                                           // max frequency error, Hz

  // initialize the frequency of execution monitoring module
  femHandleB = FEM_init(&femB,sizeof(femB));
  FEM_setParams(femHandleB,
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


  //now that interrupts are enabled, do some battery management, awkwardly placed here to avoid using too many delays for visual ques and make startup as fast as possible
  	delayMSinterrupt(600);
  	batteryAhEstimator(); //reads voltage on the current battery and determines Ah
    bat_show = 6;
    batUpdateDisplay(); // All leds are ON, adjust to what is represenetaive of Ah on batteries
    sendString("ATMD\r"); //go to data mode after updating battery


#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle,0);  
  HAL_enableDrv(halHandle,1); 

  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsA,0);
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsB,1);
#endif


  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandleA, true);
  CTRL_setFlag_enableDcBusComp(ctrlHandleB, true);






  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();



  //////////////////////////////////////////////////////////////Set up things before forever loop

  /* All of these have been written into main.h but keep these here for referrence
  gMotorVarsA.Flag_enableRsRecalc = true;
  gMotorVarsB.Flag_enableRsRecalc = true;
  gMotorVarsA.Flag_enableOffsetcalc = true;
  gMotorVarsB.Flag_enableOffsetcalc = true;
  gMotorVarsA.Flag_enableSys = true;
  gMotorVarsB.Flag_enableSys = true;
  gMotorVarsA.Flag_Run_Identify = true;
  gMotorVarsB.Flag_Run_Identify = true;
  gMotorVarsA.SpeedRef_krpm = _IQ(0.0);
  gMotorVarsB.SpeedRef_krpm = _IQ(0.0);
  gMotorVarsA.MaxAccel_krpmps = _IQ(0.0);
  gMotorVarsB.MaxAccel_krpmps = _IQ(0.0);
  */
  gMotorVarsA.Kp_spd = _IQ(10.0); //values derived from tuning the speed controller
  gMotorVarsA.Ki_spd = _IQ(0.01);
  gMotorVarsB.Kp_spd = _IQ(10.0);
  gMotorVarsB.Ki_spd = _IQ(0.01);

  clearMyBuffer(my_buffer, 19); // because there is probably stuff in there from bluetooth initiation

  EALLOW; // Hack the ADC bits, enable frontmotor sensor. This gives us hal effect values from the skateboard orientation sensor
  AdcRegs.ADCSOC8CTL.bit.CHSEL  = 0x000E; // set SOC8 channel select to hex val E corresponding to ADCINB6
  EDIS;

  initPowerPins(); // redone here because TI code had done something to the relevant pins, since initial execute in 1st line of MAIN
  //delayMSinterrupt(10); //since a single battery was chosen, wait to read the voltage
  /*batteryAhEstimator(); //reads voltage on the current battery and determine Ah
  bat_show = 6;
  batUpdateDisplay(); // All leds are ON, adjust to what is represenetaive of Ah on batteries
  sendString("ATMD\r"); //go to data mode after updating battery
  delayMSinterrupt(150);*/


  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVarsA.Flag_enableSys));
    while(!(gMotorVarsB.Flag_enableSys));

    CTRL_setFlag_enableSpeedCtrl(ctrlHandleA, true);
    CTRL_setFlag_enableSpeedCtrl(ctrlHandleB, true);

    // loop while the enable system flag is true
    while(gMotorVarsA.Flag_enableSys && gMotorVarsB.Flag_enableSys)
    {

      CTRL_Obj *objA = (CTRL_Obj *)ctrlHandleA;
      CTRL_Obj *objB = (CTRL_Obj *)ctrlHandleB;

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandleA,gMotorVarsA.Flag_enableUserParams);
        CTRL_setFlag_enableUserMotorParams(ctrlHandleB,gMotorVarsB.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(objA->estHandle,gMotorVarsA.Flag_enableRsRecalc);
        EST_setFlag_enableRsRecalc(objB->estHandle,gMotorVarsB.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandleA,gMotorVarsA.Flag_enableOffsetcalc);
        CTRL_setFlag_enableOffset(ctrlHandleB,gMotorVarsB.Flag_enableOffsetcalc);

        if(CTRL_isError(ctrlHandleB) || CTRL_isError(ctrlHandleA))
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
            bool flag_ctrlStateChangedA = CTRL_updateState(ctrlHandleA);
            bool flag_ctrlStateChangedB = CTRL_updateState(ctrlHandleB);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandleA, gMotorVarsA.Flag_Run_Identify);
            CTRL_setFlag_enableCtrl(ctrlHandleB, gMotorVarsB.Flag_Run_Identify);


            if(flag_ctrlStateChangedA)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandleA);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                   // HAL_enablePwm(halHandle); //now done at interrupt for flying start
                  }
                if(ctrlState == CTRL_State_OnLine)
                  {

                    if(gMotorVarsA.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle,0);  //sospechoso
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset),0);

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset),0);
                    }

                    // Return the bias value for currents
                    gMotorVarsA.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0,0);
                    gMotorVarsA.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1,0);
                    gMotorVarsA.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2,0);

                    // Return the bias value for voltages
                    gMotorVarsA.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0,0);
                    gMotorVarsA.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1,0);
                    gMotorVarsA.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2,0);

                    // enable the PWM
                    //HAL_enablePwm(halHandle); //now done at interrupt for flying start
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVarsA.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandleA) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVarsA.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandleA);
                  }

              }

            if(flag_ctrlStateChangedB)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandleB);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    //HAL_enablePwm(halHandle);  //now done at interrupt for flying start
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {

                    if(gMotorVarsB.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle,1); //sospechoso-JP. I switched this from 0 to 1-LK
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current,0,_IQ(I_A_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Current,1,_IQ(I_B_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Current,2,_IQ(I_C_offset),0);

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset),0);
                      HAL_setBias(halHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset),0);
                    }

                    // Return the bias value for currents
                    gMotorVarsB.I_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Current,0,0);
                    gMotorVarsB.I_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Current,1,0);
                    gMotorVarsB.I_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Current,2,0);

                    // Return the bias value for voltages
                    gMotorVarsB.V_bias.value[0] = HAL_getBias(halHandle,HAL_SensorType_Voltage,0,0);
                    gMotorVarsB.V_bias.value[1] = HAL_getBias(halHandle,HAL_SensorType_Voltage,1,0);
                    gMotorVarsB.V_bias.value[2] = HAL_getBias(halHandle,HAL_SensorType_Voltage,2,0);

                    // enable the PWM
                    //HAL_enablePwm(halHandle); //now done at interrupt for flying start
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVarsB.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandleB) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVarsB.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandleB);
                  }
              }
          }


        if(EST_isMotorIdentified(objA->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(objA->estHandle,gMaxCurrentSlopeA);
            gMotorVarsA.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandleA,gMotorVarsA.SpeedRef_krpm);

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandleA,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVarsA.MaxAccel_krpmps));

            if(Flag_Latch_softwareUpdateA)
            {
              Flag_Latch_softwareUpdateA = false;

              USER_calcPIgains(ctrlHandleA);

            }
          }
        else
          {
            Flag_Latch_softwareUpdateA = true;
            Flag_Latch_softwareUpdateB = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlopeA = EST_getMaxCurrentSlope_pu(objA->estHandle);
            gMaxCurrentSlopeB = gMaxCurrentSlopeA;
          }


        if(EST_isMotorIdentified(objB->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(objB->estHandle,gMaxCurrentSlopeB);
            gMotorVarsB.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandleB,gMotorVarsB.SpeedRef_krpm); //original version

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandleB,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVarsB.MaxAccel_krpmps));

            if(Flag_Latch_softwareUpdateB)
            {
              Flag_Latch_softwareUpdateB = false;

              USER_calcPIgains(ctrlHandleB);

            }
          }
        else
          {
            Flag_Latch_softwareUpdateB = true;
            Flag_Latch_softwareUpdateA = true;
            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlopeB = EST_getMaxCurrentSlope_pu(objB->estHandle);
            gMaxCurrentSlopeA = gMaxCurrentSlopeB;
          }

        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;
            updateGlobalVariables_motorA(ctrlHandleA);
            updateGlobalVariables_motorB(ctrlHandleB);
          }

        // get the maximum delta count observed
        gMaxDeltaCntObservedA = FEM_getMaxDeltaCntObserved(femHandleA);
        gMaxDeltaCntObservedB = FEM_getMaxDeltaCntObserved(femHandleB);


        // check for errors
        if(FEM_isFreqError(femHandleA))
          {
            gNumFreqErrorsA = FEM_getErrorCnt(femHandleA);
          }

        if(FEM_isFreqError(femHandleB))
         {
            gNumFreqErrorsB = FEM_getErrorCnt(femHandleB);
          }
        
        // update CPU usage
        updateCPUusage(ctrlHandle);

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

//////////////except for variables, Custom LEIF code begins here ///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////// REMOTE or DEBUGGER control handling and target speed decisions

  if (processUARTbuffer())  // looking for speed commands in the buffer, returns true. Looks for other commands too but returns false in those cases
  {
    if (bat_show > 0)  //do not allow motor control if batteries are almost dead
    {
      coasting = false; //   flag used for flying start to turn on motor control PWM
      batteryPair(); //  if batteries are equal, use both
      target_speed_temp = commandScalerToMotor(); // uses the speed command found in UARTbuffer and turns it into to IQ speed in KRPM

      if (target_speed_temp >= MAXIMUM_SPEED_KRPM) // If remote speed value is > max speed, set target speed to max speed
      {
        target_speed = MAXIMUM_SPEED_KRPM;
      }  //eliminated governer speed control to test acceleration

      if ( target_speed_temp < _IQ(0.0)) // If remote speed value is < min speed value, set target speed to min speed (0)
      {
        target_speed = _IQ(0.0); // this prevents wheels from ever spinning backwards
      }
      if (_IQabs(target_speed_temp - target_speed) >= _IQ(0.20)) //asking if the new target value is at least 200rpm different from the previous, stabilizes the controller
      {
        target_speed = target_speed_temp;
      }
    }
  }

  else if ((!silence_state) && (!safety_stop) && (!debug))// no message so we start counting down towards coasting, except if safety stop routine is in effect. 
  // In debug mode, do not want to count down to silence
  {
    silence_counter++;
    if (silence_counter > 300) // maybe half a sec. Usually this never happens, becuase silence state is primarily called by the !!coast\r command from the remote
    {
      coasting = true; // turn off the controller PWM
      Flag_Enable_Inverter = false; // a variable paired with coasting, needed because flying start has multiple stages
      batteryUnPair(); // disconnect low priority battery so that the chemistry in each battery can stabilize separately
      fs_recently_used = 0; // makes it possible to do Forced Start right away after timeout or user lets go of the trigger on the remote
      silence_counter = 0;
      silence_state = true;
    }
  }



///////////////////////////////////////////////  BOARD ORIENTATION, USER level Mode, and part of POWER handling

front_motor_index = getOrientation(); //checks the orientation of the motor. Default is 1.

 // User modes are 1, 2, 3 
 // 1:Beginner 2:Normal 3:Expert 

if (user_mode == 3 || safety_stop) // expert mode or a safety stop is called. Full power to both wheels
 {
	 CTRL_setSpd_max_pu(ctrlHandleB, _IQ(1.0));
	 CTRL_setSpd_max_pu(ctrlHandleA, _IQ(1.0));
 }

else if (user_mode == 2) //normal mode, back has 70% to 100% power based on need.
 {
	 if (front_motor_index == 1)
	 {
		 CTRL_setSpd_max_pu(ctrlHandleA, _IQ(1.0));
		 CTRL_setSpd_max_pu(ctrlHandleB, back_power_limit);
	 }

	 else if (front_motor_index == 2)
	 {
		 CTRL_setSpd_max_pu(ctrlHandleB, _IQ(1.0));
		 CTRL_setSpd_max_pu(ctrlHandleA, back_power_limit);
	 }
 }

else if (user_mode == 1) //beginner mode, always 50% power on the back
 {
   if (front_motor_index == 1)
   {
     CTRL_setSpd_max_pu(ctrlHandleA, _IQ(1.0));
     CTRL_setSpd_max_pu(ctrlHandleB, _IQ(0.5));
   }

   else if (front_motor_index == 2)
   {
     CTRL_setSpd_max_pu(ctrlHandleB, _IQ(1.0));
     CTRL_setSpd_max_pu(ctrlHandleA, _IQ(0.5));
   }
 }


////////////Record speed of motors
// Makes sure both motors are going at the same speed
speed_of_faster_motor = gMotorVarsA.Speed_krpm;  // will represent the faster of the two motors
  if (speed_of_faster_motor < gMotorVarsB.Speed_krpm)
  {
	 speed_of_faster_motor = gMotorVarsB.Speed_krpm;  // If B is faster than A, then A gets the speed of B 
  }

// Checks the average speed of the motors,
speed_average_of_motors = _IQdiv(gMotorVarsA.Speed_krpm + gMotorVarsB.Speed_krpm, _IQ(2.0));

  if ((_IQabs(gMotorVarsA.Speed_krpm) < _IQ(0.20)) && (_IQabs(gMotorVarsB.Speed_krpm) < _IQ(0.20))) // the wheels have stopped threshold
  {
	 wheels_are_not_moving = true;
  }
  else
  {
	 wheels_are_not_moving = false;
  }


////////////////////////////////////////////////////ACCELERATION handling (during normal operation and differently during safety stop)
if (safety_stop) //set target speed at zero, a fairly mild accel, and disbale the safety stop of the wheel actually stop
 {
	 target_speed = _IQ(0.0);
	 target_accel = _IQ(0.6);
	 if (wheels_are_not_moving)// stopped
	 {
		 safety_stop = false; //  silence counter now begins to count down, soon coasting will be called. Using silence to enter coasting put a desired delay
	 }
 }

else //safety stop not engaged, the usual case
 {
	 //when ever switching acceleration direction, reset acceleration to near zero
	if (target_speed > speed_average_of_motors && accelerating_forward == false)
	 {
		 accelerating_forward = true;
		 current_accel = _IQ(0.05);
		 gMotorVarsA.MaxAccel_krpmps = current_accel;
		 gMotorVarsB.MaxAccel_krpmps = current_accel;
	 }
	else if(target_speed < speed_average_of_motors && accelerating_forward == true)
	 {
		 accelerating_forward = false;
		 current_accel = _IQ(0.05);
		 gMotorVarsA.MaxAccel_krpmps = current_accel;
		 gMotorVarsB.MaxAccel_krpmps = current_accel;
	 }
  if (target_speed_temp < _IQ(0.0)) // target speed is never negative, but temp_target_speed can be. This is used as a signal to put the brakes.
	 {
		  target_accel = _IQ(3.0); //very fast brake decell *** changed from 3.0 to 0.5

	   if (current_accel < target_accel) // shorten JERK length
	   {
		    current_accel = current_accel + _IQ(0.05); // approach target accell
	   }
	 }
	else if (target_speed  >= _IQ(0.0) && target_speed > speed_average_of_motors)// standard forward accel when demand is to go faster
	  { // target_accel = _IQ(3.0) - _IQmpy(_IQdiv(speed_average_of_motors, _IQ(2.8)), _IQ(2.5));
		 if (speed_average_of_motors <= _IQ(2.8))
		 {
			 target_accel = _IQ(5.0);
		 }
		 else
		 {
			 target_accel = _IQ(0.25);
		 }
			/*  target_accel = _IQ(1.5);*/
    }
	else // target speed is positive but less than current speed of motors
	  {
		  target_accel = _IQabs( _IQmpy(speed_average_of_motors - target_speed, _IQ(0.05))) + _IQ(0.1); // nice soft deacell. Considers distance from target. Sort of a P controller --- changed from 0.1 to 0.08
	  }
 }

///  variable acceleration to remove jerk. 3 step ramp up, max accel, and ramp down routines
  if (_IQabs(_IQdiv(speed_average_of_motors - target_speed , target_speed )) >= _IQ(.02) )//asking if current speed is more than 2 % different from target speed
   {
    if (current_accel < target_accel)
     {
       current_accel = current_accel + _IQ(0.02); //approach target accell
     }

    if (_IQabs(_IQdiv(speed_average_of_motors - target_speed, target_speed)) <= _IQ(.05) ) //asking if current speed is less than 5 % different from target. if yes, ramp down on accel
     {
       if (current_accel > _IQ(0.05))
       {
         current_accel = current_accel - _IQ(0.03); // ramp down the accel. (note this has to be faster than current_accel = current_accel + _IQ(0.02) since both if's can be true )
       }
     } // by the way!! this means if new target is between 2% and 5% away, the acceleration will hover around .04 and .08 without ramping up or down. That's good! no need for major accel here
   }
  else
   {
     current_accel = _IQ(0.05); // avoid a complete zero accell, so the controler can always adjust a bit
   }
//END of accel code


///////////////////////////////////////////////////////////FORCED START handling
  if (!fs_mode && !fs_recently_used) // Not in fs_mode now or recently
   {
     if (wheels_are_not_moving) // if we are too slow or arent moving at all, stop
     {
       if (target_speed >= _IQ(0.45)) //  but if we want to get going, engage fs
       {
         //reset all the fs variables, otherwise pattenStart() would pick up at a strange place
         fs_phase_speed = _IQ(0.00001);
         fs_phase_accelerate = _IQ(0.00001);
         fs_power = _IQ(0.15); // this is somewhat arbitrary, 0.15 has beeen working well. use 0.8 or less if doing lots of desktop testing of FS
         fs_mode = 1;
         fs_recently_used = 1;
       }
       else //otherwise, we are really stopping
       {
    	  target_speed = _IQ(0.0);
    	  //  gMotorVarsA.SpeedRef_krpm = target_speed;
    	  //  gMotorVarsB.SpeedRef_krpm = target_speed;
       }
     }

   }

  if (fs_mode) // pattenStart() is in control, keep an eye on target_speed, if it is too low, disable pattenStart()
   {
       if (target_speed >= _IQ(0.45))
       {
    	   target_speed = _IQ(0.6); // approximately the end speed in KRPM of pattenStart()
       }

       else  // if during fs, target speed is way low, cancel fs and slow down to a stop
       {
    	   fs_mode = 0;
    	   target_speed =  _IQ(0.0);
       }
       current_accel = _IQ(0.5); //do be ready to accellerate fairly quickly afer pattenStart() to avoid stalling
   }// END of force start code


   /// since all desired accelleration and speed has been calculated, update the variables used by the controller
   gMotorVarsA.MaxAccel_krpmps = current_accel;
   gMotorVarsB.MaxAccel_krpmps = current_accel;
   gMotorVarsA.SpeedRef_krpm = target_speed;
   gMotorVarsB.SpeedRef_krpm = target_speed;


/////////////////////////////////////////////////////////////Long chunk of code for things to execute at relatively low  frequency
if (to10thsec_counter > (USER_ISR_FREQ_Hz *0.1)) // 10 times per sec
  {
	to10thsec_counter = 0; //this is counted at the interrupt
  
  // SAFETY STOP CODE
	if (safetyStop_pingPONG_enabled)
	 {
		if (bluetooth_connected) // data came over bluetooth recently
		{
			bluetooth_disconnected_counter++;
			if (bluetooth_disconnected_counter > 23) // the last 2 PINGs plus a little bit of time did not receive a PONG response. bluetooth connection is not live
			{
				bluetooth_connected = false;
				bluetooth_disconnected_counter = 0;
			}
		}

		// Conditions below: basically safety stop has not been called yet, bluetooth is not working, wheels are moving, therefore we need to begin a safety stop. 
    // Disabled in debug mode
		if ((!safety_stop) && (!bluetooth_connected) && (gMotorVarsA.Speed_krpm > _IQ(0.6) || gMotorVarsB.Speed_krpm > _IQ(0.6)) && !debug)
		{
			safety_stop = true; //set of a flag to stop the wheels.
			safety_stop_counter = 0;
			current_accel = _IQ(0.05); //dont want to Jerk the rider, set a very low accel
			coasting = false; // turn on the controller
			silence_counter = 0; // silence_counter set here at zero, gets used at the end of safety_stop as a short delay.
			silence_state = false; //to avoid the silence_state flag from re-enabling coast
		}


		if (safety_stop) // this is a failsafe measure, in case an outside ciscumstance prevents the complete stop of the motors. Would not want to continue this process permanently
		{
			safety_stop_counter++;
			if (safety_stop_counter > 120) // DISABLE safety stop if it has not been able to lock the wheel in 12 seconds
			{
				safety_stop_counter = 0;
				safety_stop = false;
			}
		}
	 }

  // Whenever PWM is active, measure overall current and update battery usage
	if (fs_mode || !coasting) 
	 {
		overall_kW = _IQmpy(_IQmpy( gMotorVarsA.VdcBus_kV , _IQ(USER_IQ_FULL_SCALE_CURRENT_A / 2)),
				_IQmpy(gPwmDataB.Tabc.value[0], gAdcDataB.I.value[0]) + _IQmpy(gPwmDataB.Tabc.value[1], gAdcDataB.I.value[1]) + _IQmpy(gPwmDataB.Tabc.value[2], gAdcDataB.I.value[2]) +
				_IQmpy(gPwmDataA.Tabc.value[0], gAdcDataA.I.value[0]) + _IQmpy(gPwmDataA.Tabc.value[1], gAdcDataA.I.value[1]) + _IQmpy(gPwmDataA.Tabc.value[2], gAdcDataA.I.value[2]));


		overall_I =  _IQdiv(overall_kW, gMotorVarsA.VdcBus_kV); //divide kW by kV to get I
		batAhSubtractUsed(); //every 10th of a sec, subtract consumed Ah XXX
	 }  

  // normal riding operation, record and adjust back motor performance
	if ((!coasting) && (!fs_mode) && (!fs_recently_used)) 
	 {
		// calculate average current consumption for the past second by the back motor
		if (front_motor_index == 1)
		{
			back_motorkW = _IQmpy(_IQmpy( gMotorVarsA.VdcBus_kV , _IQ(USER_IQ_FULL_SCALE_CURRENT_A / 2)),
					_IQmpy(gPwmDataB.Tabc.value[0], gAdcDataB.I.value[0]) + _IQmpy(gPwmDataB.Tabc.value[1], gAdcDataB.I.value[1]) + _IQmpy(gPwmDataB.Tabc.value[2], gAdcDataB.I.value[2]));
		}
		else if (front_motor_index == 2)
		{
			back_motorkW = _IQmpy(_IQmpy( gMotorVarsA.VdcBus_kV , _IQ(USER_IQ_FULL_SCALE_CURRENT_A / 2)),
					_IQmpy(gPwmDataA.Tabc.value[0], gAdcDataA.I.value[0]) + _IQmpy(gPwmDataA.Tabc.value[1], gAdcDataA.I.value[1]) + _IQmpy(gPwmDataA.Tabc.value[2], gAdcDataA.I.value[2]));
		}

		back_motorI[power_index] =  _IQdiv(back_motorkW, gMotorVarsA.VdcBus_kV); //divide kW by kV to get I
		back_motorI_average = _IQdiv(back_motorI[0],_IQ(10)) + _IQdiv(back_motorI[1],_IQ(10)) + _IQdiv(back_motorI[2],_IQ(10)) + _IQdiv(back_motorI[3],_IQ(10)) + _IQdiv(back_motorI[4],_IQ(10)) +
				_IQdiv(back_motorI[5],_IQ(10)) + _IQdiv(back_motorI[6],_IQ(10)) + _IQdiv(back_motorI[7],_IQ(10)) + _IQdiv(back_motorI[8],_IQ(10)) + _IQdiv(back_motorI[9],_IQ(10));
		// back motor average is the result of adding the 10% value of the last 10 samples. 
    // It is important that we dont add everything and then divide by ten, because of the risk of exceeding the the IQ math range

		power_index++;
		power_index %= 10;
    } 
  }


/*********************************************************************************************************************************
instead of the above method which samples the last 10 readings to create an average, it is possibly to simply multiply the runing average value by 0.9, the newest value by 0.1 and add up the total.
This  method is computationally much simpler, but can in some cases behave quite differently. The sampling method has completely new data every one second. The running average method is affected by high values for a very long time.
for instance if the running average is 10amps, and then 0 current is drawn, after 1 sec, the average is calculated at 3.5amps, 1.2 amps after 2 seconds. .42 after 3 seconds
back_motorI_newest = _IQdiv(back_motorkW, gMotorVarsA.VdcBus_kV); //divide kW by kV to get I
back_motorI_average = _IQmpy(back_motorI_average, _IQ(0.9) + _IQmpy(back_motorI_newest, _IQ(0.1)
*/


//  Use back motor performance data to adjust power

if ( back_motorI_average < _IQ(6.0)) //only reduce power if demand drops below 6amps
		  {
			back_power_limit = _IQ(0.7);
			/*if (need_based_battery_pairing_allowed)
			{
				if (!batteries_are_equal) // at lower current consumptions, if batteries werent perfectly matched at the start, disconnect
				{
					batteryUnPair();
				}
			}*/
		  }

else if (back_motorI_average > _IQ(10.0)) // anything above 10amps requires full power
		  {
			back_power_limit = _IQ(1.0);

			/*if (need_based_battery_pairing_allowed) //allowing to pair both batteries even if not equal since motors will draw most of the current
			{
				batteryPair();
			}*/
      }

else if (back_motorI_average > _IQ(7.0) && back_power_limit < _IQmpy(back_motorI_average, _IQ(0.1))) 
    //between 7 and 10 amps, allow for smooth transitions, but only upwards
		  {
			back_power_limit = _IQmpy(back_motorI_average, _IQ(0.1));
			/*if (need_based_battery_pairing_allowed)  // allowing to pair both batteries even if not equal since motors will draw most of the current
			{
				batteryPair();
			}*/
		  }
		// else ... no change to power
		// result of the above statements is that we remain at what ever power we reach untill dropping all the way back down to < 6amps.
		// this ensures that during times of heavy use, a short low demand moment doesnt result in loss of need power setting


//////////////////////////////////////////////////////////////////////PURELY for debugg and recording purposes

if (gMotorVarsA.Speed_krpm > speed_record)
		{
			speed_record = gMotorVarsA.Speed_krpm;
		}

if (target_speed > speed_record_demanded)
		{
			speed_record_demanded = target_speed;
		}

if (back_motorI_average > back_motorI_average_record)
		{
			back_motorI_average_record = back_motorI_average;
		}

if (back_power_limit > back_power_limit_record)
		{
			back_power_limit_record = back_power_limit;
		}

if (back_power_limit > _IQ(0.85)) //asking if we are basically running high power, specifically 8.5 amps or more on the back
		{
			power_high_counter++;
		}
	
else if (back_power_limit < _IQ(0.85))
		{
			if (power_high_counter > power_high_counter_record)
			 {
				 power_high_counter_record =  power_high_counter;
			 }
			power_high_counter = 0;
		}
	

}// end if !coasting & !fs_mode !fspause, basically end of back motor output manager <= mistake or just important?


if (coasting)// during standby, pay attention to button presses and battery management
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////1st handle POWER BUTTON SWiICH
		if (!GpioDataRegs.GPADAT.bit.GPIO26 ) //pulled up so this means button pressed...
		  {
			power_button_counter++;
		  }

		else if (GpioDataRegs.GPADAT.bit.GPIO26 && power_button_counter > 15)// (button released after 5 seconds). A bit strange why 15 cycles is so long
		{
			if (user_mode == 1) //beginner mode, swithing to normal mode
			 {
				user_mode = 2;
				power_button_counter = 0;
				//do double flash
				sendString("+++\r");
				delayMSinterrupt(150);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(500); // you cannot use this function for a full 1000 or more ms
				delayMSinterrupt(500);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,0\r");

			 }

			else if (user_mode == 2) //normal mode, swtiching to expert mode
			{
				user_mode = 3;
				power_button_counter = 0;
				//do triple flash
				sendString("+++\r");
				delayMSinterrupt(150);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(500); // you cannot use this function for a full 1000 or more ms
				delayMSinterrupt(500);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300);
				sendString("ATSPIO,7,1,0\r");
			}

			else if (user_mode == 3) //expert mode, switching to beginner mode
			{
				user_mode = 1;
				power_button_counter = 0;
				//do long blink for beginner mode
				sendString("+++\r");
				delayMSinterrupt(150);
				sendString("ATSPIO,7,1,0\r");
				delayMSinterrupt(500); // you cannot use this function for a full 1000 or more ms
				delayMSinterrupt(500);
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(300); // you cannot use this function for a full 1000 or more ms
				sendString("ATSPIO,7,1,0\r");
			}

			delayMSinterrupt(500); // you cannot use this function for a full 1000 or more ms
			delayMSinterrupt(500);
			if (bat_show == 6) // 6th LED will be off after the above sequence. correct it if the 6th should be on
			{
				sendString("ATSPIO,7,1,1\r");
				delayMSinterrupt(200);
			}
			sendString("ATMD\r"); //done blinking LEDS, go back to data move
			to10thsec_counter = 0; //this is counted at the interrupt, since we just had so many delays, reset this
		}

		else if (GpioDataRegs.GPADAT.bit.GPIO26 && power_button_counter) //pressed and relesed in less than 3 seconds
		{
			ready_2HALT = true; // flag to put the board to sleep
			power_button_counter = 0;
			gMotorVarsA.Flag_enableSys = false;
			gMotorVarsB.Flag_enableSys = false;
		}
		////////////////////End of Power Button stuff


		tofullsec_counter++; //Start counting towards even lower fequency tasks only allowed while coasting
		if (tofullsec_counter >= 10) // full second reached
		{
			tofullsec_counter = 0;
			batteryPriorityManager(); //compares the two batteries, assigns current_bat and other_bat
			batReadlow(); //check for really low battery, this overrides batAhSubtractUsed()
			batteryAhEstimator_counter++;
			if (batteryAhEstimator_counter > 120)// if no strain has been put on the battery for the past 2 minutes, you can re-evaluate it's Ah based purely on voltage
			{
				batteryAhEstimator();
				batteryAhEstimator_counter = 0;
			}

			//May 4th, bit below moved here from section that only allows action at zero speed. Battery display will now update a second after starting to coast
			//till_batUpdateDisplay_counter++; //start counting to 5 seconds of no motion. Assume the user wont mind if the batterydisplay gets updated
			//if ( till_batUpdateDisplay_counter >= 2) // Aaron changed from 5 to 1
			//{
				if ( bat_show != bat_actual) // what is displayed does not match known battery level
				{
					sendString("+++\r");
					delayMSinterrupt(150);
					batUpdateDisplay(); //update the display LEDS
					sendString("ATMD\r");
					till_batUpdateDisplay_counter = 0;
					to10thsec_counter = 0; // counted at ISR, reset this becuase we just used delays
				}
			//}

				else if (bat_show == 0) //battery is almsot dead, flash the last LED on the display board
				{
					sendString("+++\r");
					delayMSinterrupt(150);
					batUpdateDisplay();
					sendString("ATMD\r");
					till_batUpdateDisplay_counter = 0;
					to10thsec_counter = 0; // counted at ISR, reset this becuase we just used delays
				}




			if  (wheels_are_not_moving) //following actions only allowed if skateboard is not in motion
			{
				HALT_counter++; //counting seconds till putting the board to sleep
				if ((HALT_counter > 300) || (bat_show==0 && HALT_counter > 30)) //approx 300 seconds, or just 30 seconds when bat low
				{
					HALT_counter = 0;
					gMotorVarsA.Flag_enableSys = false;
					gMotorVarsB.Flag_enableSys = false;
					ready_2HALT = true;
				}

				if (safetyStop_pingPONG_enabled) //PING the remote, processUARTbuffer will listen to the pong\r response
				{
					sendString("PING\r"); //ask remote to ping back, but ok to keep counting down towards sleep
				}
			}//end of motion check

			else //means board is in motions and therefore neither the board or the remote should sleep, and battery display updates should not execute
			{
				HALT_counter = 0;
				till_batUpdateDisplay_counter = 0;
				batteryAhEstimator_counter = 0; //added mays 1st XXX
				sendString("MOVE\r");//IMPORTANT! This keeps the remote awake as well! would be dangerous if LEIF board is coasting downhill and the remote went to sleep
				//this will aslo expect a ping back, and does NOT allow the remote to count towards sleep
			}
		}//end full sec
	}// end if coasting

	else //not coasting, means not silent and in action, reset most counters
	{
		tofullsec_counter = 0;
		till_batUpdateDisplay_counter = 0;
		batteryAhEstimator_counter = 0;
	}
}// end of 1/10th sec


////////////////////////////////////////////////////COME back to this, enabling this to reduce noise at zero speed, but does not allow propper function after force start

  // update Kp and Ki gains...............no worky, some lower level issue i guess, talk to JAMES
  // updateKpKiGainsA(ctrlHandleA);
  // updateKpKiGainsB(ctrlHandleB);

    // run Rs online....works great
    runRsOnLineA(ctrlHandleA);
    runRsOnLineB(ctrlHandleB);

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
        HAL_writeDrvData(halHandle,&gDrvSpi8301VarsA,0);
        HAL_writeDrvData(halHandle,&gDrvSpi8301VarsB,1);
        HAL_readDrvData(halHandle,&gDrvSpi8301VarsA,0);
        HAL_readDrvData(halHandle,&gDrvSpi8301VarsB,1);
#endif
      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);


    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandleA,&gUserParams);
    CTRL_setParams(ctrlHandleB,&gUserParams);

    gMotorVarsA.Flag_Run_Identify = false;
    gMotorVarsB.Flag_Run_Identify = false;


    if (ready_2HALT)
    {
      ///put DRV8301 into low power mode by setting EN_GATE pin low
    	GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    	GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1;

      // then disable bluetooth and battery display

      sendString("+++\r");
      delayMSinterrupt(200);
      bat_actual = 0; //pretend that battery level is zero so that batUpdateDisplay() turns off all LEDs
      bat_show = 6; // just to make sure the update on the next line hits every LED
      batUpdateDisplay();
      sendString("ATDH,0\r"); //disconnect any active connections
      delayMSinterrupt(200);
      sendString("ATDC\r"); // disable advertizing, allows lowest level sleep
      delayMSinterrupt(200);
      sendString("ATSZ,0,1,0\r");//bluetooth will fully wake up on receiving the first char from MCU
      delayMSinterrupt(200);
      sendString("ATZ\r");
      delayMSinterrupt(200);

      GpioDataRegs.GPBSET.bit.GPIO39 = 1; //turn off the "software running" LED
      GpioDataRegs.GPBSET.bit.GPIO34 = 1;

      batteryPair(); //if batteries are equal, connect them to each other so they drain over time together

   ///////////////////////////////////////////ported from HALT example, look there for explanation of code
      asm("  EALLOW");

        EALLOW;
        GpioIntRegs.GPIOLPMSEL.bit.GPIO26 = 1;  // Choose GPIO0 pin for wakeup
        EDIS;

      /// Step 3. Clear all interrupts and initialize PIE vector table:
      // Disable CPU interrupts
         DINT;

      // Disable CPU interrupts and clear all CPU interrupt flags:
         IER = 0x0000;
         IFR = 0x0000;

      // Interrupts that are used in this example are re-mapped to
      // ISR functions found within this file.
         EALLOW;  // This is needed to write to EALLOW protected registers
         PieVectTable.WAKEINT = &WAKE_ISR;
         EDIS;

      // Enable CPU INT1 which is connected to WakeInt:
         IER |= M_INT1;

      // Enable WAKEINT in the PIE: Group 1 interrupt 8
         PieCtrlRegs.PIEIER1.bit.INTx8 = 1;
         PieCtrlRegs.PIEACK.bit.ACK1 = 1;

      // Enable global Interrupts:
         EINT;   // Enable Global interrupt INTM

      // Write the LPM code value
          EALLOW;
        if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 1) // Only enter low power mode when PLL is not in limp mode.
        {
            SysCtrlRegs.LPMCR0.bit.LPM = 0x0002;   // LPM mode = Halt
          }
          EDIS;

      // Force device into HALT
      __asm(" IDLE");                           // Device waits in IDLE until falling edge on GPIO0/XNMI pin

      //////Now reinitiate a lot of things....
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

      //////////////////////////////////////////////////////////////////////Wake up Sequence: reset UART, wake up Bluetooth turn on LEDS
      GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;//turn LEDS back on

      if (ScibRegs.SCIRXST.bit.RXERROR == 1) //possibly the RX has been overrun, so reset if there is an error
      {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
      }



      //Wake up blueooth
      sendString("W\r"); //send a char to wake up local bluetooth module. If bluetooth is ever sleeping when MCU first powers up, should be ok, initBatStat sends enough junk
      delayMSinterrupt(200);
      sendString("+++\r");
      batteryUnPair(); //on wake up, unpair the batteries so that the their voltages can be read separatly
      delayMSinterrupt(200);
      batteryPriorityManager(); //pick higher battery
      batteryAhEstimator(); // restimate overal Ah left

      //sendString("ATSLED,1,0,1\r"); //sometiems pin five seazes to act as GPIO, remind it to be a GPIO
      initBatStatus();
      delayMSinterrupt(200);
      batUpdateDisplay();
      delayMSinterrupt(200);
      sendString("ATDSLE\r"); //become discoverable
      delayMSinterrupt(150);


      ////////////////////////////////////Re-initialize DRV8301. NOTE: just reenabling EN_gate causes bad motor screetching. The reinit below is pretty slow, requiring a few seconds, but it works fine
      HAL_enableDrv(halHandle,0);  // -jp
      HAL_enableDrv(halHandle,1);  // -jp
      // initialize the DRV8301 interface
      HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsA,0);
      HAL_setupDrvSpi(halHandle,&gDrvSpi8301VarsB,1); // - jp

      gMotorVarsA.Flag_enableSys = true;
      gMotorVarsB.Flag_enableSys = true;
      gMotorVarsA.Flag_Run_Identify = true;
      gMotorVarsB.Flag_Run_Identify = true;
      ready_2HALT = false;
    }
  } // end of for(;;) loop
} // end of main() function 


interrupt void mainISR(void) //
{
  uint32_t timer0Cnt;
  uint32_t timer1Cnt;
  captureUART();
  to10thsec_counter++;

  // read the timer 1 value and update the CPU usage module
  timer1Cnt = HAL_readTimerCnt(halHandle,1);
  CPU_USAGE_updateCnts(cpu_usageHandle,timer1Cnt);

  // read the timer 0 value and update the FEM
  timer0Cnt = HAL_readTimerCnt(halHandle,0);
  FEM_updateCnts(femHandleA,timer0Cnt);
  FEM_run(femHandleA);
  FEM_updateCnts(femHandleB,timer0Cnt);
  FEM_run(femHandleB);


  // toggle status LED
  if(gLEDcnt++ > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED2);
    gLEDcnt = 0;
  }

  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcDataA(halHandle,&gAdcDataA);
  HAL_readAdcDataB(halHandle,&gAdcDataB);


  ////////////////////////// Jorge's Flying Start code. so awesome
  if(CTRL_getState(ctrlHandleA) == CTRL_State_OnLine)
  {
    if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == true))
    {
      // disable the PWM
      HAL_disablePwm(halHandle);
      Flag_is_Inverter_Enabled = false;

    }

    else if((Flag_Enable_Inverter == false)&&(Flag_is_Inverter_Enabled == false))
    {
      CTRL_Obj *objA = (CTRL_Obj *)ctrlHandleA;
      CTRL_Obj *objB = (CTRL_Obj *)ctrlHandleB;

      if (!coasting)
      {
           Flag_Enable_Inverter = true;
      }

      // Reset Speed, Id and Iq integral outputs Ui
      PID_setUi(objA->pidHandle_Id,gVdq_in.value[0]);
      PID_setUi(objB->pidHandle_Id,gVdq_in.value[0]);
      PID_setUi(objA->pidHandle_Iq,gVdq_in.value[1]);
      PID_setUi(objB->pidHandle_Iq,gVdq_in.value[1]);

      CTRL_setVdq_out_pu(ctrlHandleA,&gVdq_in);
      CTRL_setVdq_out_pu(ctrlHandleB,&gVdq_in);

      CTRL_setFlag_enableSpeedCtrl(ctrlHandleA,false);
      CTRL_setFlag_enableSpeedCtrl(ctrlHandleB,false);
      TorqueModeCounter = 0;
    }

      else if((Flag_Enable_Inverter == true)&&(Flag_is_Inverter_Enabled == true)&&(CTRL_getFlag_enableSpeedCtrl(ctrlHandleA) == false))
      {
        TorqueModeCounter++;

        if (fs_recently_used) //this logic applies different flying start time for post FS and post coast situations
        {
          TorqueModeCounterUseNow = TORQUE_MODE_COUNTER_MAX_CNT_AFTER_FS; //super short but stronger flying start
        }
        else
        {
          TorqueModeCounterUseNow = TORQUE_MODE_COUNTER_MAX_CNT; // longer smoother flying start

        }
        // if(TorqueModeCounter > TorqueModeCounterMaxCnt)// original method by Jorge
        if(TorqueModeCounter > TorqueModeCounterUseNow)// custom method Dec 18th
        {
          CTRL_Obj *objA = (CTRL_Obj *)ctrlHandleA;
          CTRL_Obj *objB = (CTRL_Obj *)ctrlHandleB;

          PID_setUi(objA->pidHandle_spd,_IQ(0.0));
          PID_setUi(objB->pidHandle_spd,_IQ(0.0));

            // Set Speed Reference target and intermediate values to estimated speed
            TRAJ_setIntValue(objA->trajHandle_spd, EST_getFm_pu(objA->estHandle));
            TRAJ_setIntValue(objB->trajHandle_spd, EST_getFm_pu(objB->estHandle));


            gMotorVarsA.SpeedRef_krpm = EST_getSpeed_krpm(objA->estHandle);
            gMotorVarsB.SpeedRef_krpm = EST_getSpeed_krpm(objB->estHandle);

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandleA,gMotorVarsA.SpeedRef_krpm);
            CTRL_setSpd_ref_krpm(ctrlHandleB,gMotorVarsB.SpeedRef_krpm);


            CTRL_setFlag_enableSpeedCtrl(ctrlHandleA,true);
            CTRL_setFlag_enableSpeedCtrl(ctrlHandleB,true);
            }
        }
      else if((Flag_Enable_Inverter == true)&&(Flag_is_Inverter_Enabled == false))
        {
          // enable the PWM
          HAL_enablePwm(halHandle);
          Flag_is_Inverter_Enabled = true;
        }
    }

  else if(CTRL_getState(ctrlHandleA) == CTRL_State_Idle)
    {
      Flag_Enable_Inverter = false;
      Flag_is_Inverter_Enabled = false;
    }


  // run the controller
  CTRL_run(ctrlHandleA,halHandle,&gAdcDataA,&gPwmDataA,0);
  CTRL_run(ctrlHandleB,halHandle,&gAdcDataB,&gPwmDataB,1);

  if (fs_mode)
  {
    pattenStart(); //open loop forced start
  }


  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmDataA,0);
  HAL_writePwmData(halHandle,&gPwmDataB,1);


  // setup the controller
  CTRL_setup(ctrlHandleA);
  CTRL_setup(ctrlHandleB);

  // read the timer 1 value and update the CPU usage module
  timer1Cnt = HAL_readTimerCnt(halHandle,1);
  CPU_USAGE_updateCnts(cpu_usageHandle,timer1Cnt);


  // run the CPU usage module
  CPU_USAGE_run(cpu_usageHandle);

  return;
} // end of mainISR() function


void initAll()// setup sysctrl, ADC, button and LED pins. also SCIB: 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
{
  InitSysCtrl();// Step 1. Initialize System Control:
  InitScibGpio();//LLL this funciton set pin 58 tx as and pin 15 as rx
  InitAdc();

  ///* Might not need any of this other ADC stuff get overwritten in hall setup stuffs
  // Configure ADC , looks like SOC channels 0 and 8 are free, so assign ADCINB6(motor orientatio) to SOC0, and xxxxxxxxxxx to SOC8 for battery life
  EALLOW;
  GpioCtrlRegs.AIOMUX1.bit.AIO14 = 2;// Configure AIO14 for B6 (analog input) operation
  AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  // Enable non-overlap mode
  AdcRegs.ADCSOC8CTL.bit.CHSEL  = 0x000E;    // set SOC7 channel select to hex val E corresponding to ADCINB6
  AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;                  //Select internal reference mode


  //Initialize GPIOs for the LEDs and turn them off // actually cheeck what really happens here
  GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; //operation LEDS out//actually I think I can delete this
  GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;

  //set up power button and power LED
  GpioCtrlRegs.GPADIR.bit.GPIO23 = 1; //power LED out
  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;  // GPIO23 = GPIO23
  GpioDataRegs.GPACLEAR.bit.GPIO23 = 1; // clear Latch
  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1; // disable pull-up

  GpioCtrlRegs.GPADIR.bit.GPIO26 = 0; //power button as input
  GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;  // GPIO26 = GPIO26
  GpioDataRegs.GPACLEAR.bit.GPIO26 = 1; // clear Latch
  GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0; // enable pull-up

  GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0; //input on connection status from bluetooth (also RED LED)
  GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;  // GPIO56 = GPIO56
  GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1; // clear Latch
  GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1; // disable pull-up

  GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1; //reset pin out
  GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;  // GPIO57 = GPIO57
  GpioDataRegs.GPBSET.bit.GPIO57 = 1; // when clear, this reset the bluetooth chip
  GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1; // disable pull-up
  EDIS;

  //final touches to serial communication
  ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback,No parity,8 char bits, async mode, idle-line protocol
  ScibRegs.SCICTL1.all =0x0003;
  ScibRegs.SCICTL1.bit.RXENA =1;// enable TX, RX
  ScibRegs.SCICTL1.bit.TXENA =1;
  ScibRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).    //ok so LPSCLK just dropeed by a factor of 9, so new  if hex17 = Dec 23??
  ScibRegs.SCILBAUD    =0x0017; //
  ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

void initBatStatus() //Initiates and blinks all LEDS once
{
  //1st reset the bluetooth modukle
  GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1; // pulse reset pin
  delayMS(5);//5ms // use the non interrupt delay version since interrupts have not bee setup yet when this funciton is called
  GpioDataRegs.GPBSET.bit.GPIO57 = 1;
  delayMS(400); //long pause after reset


  //then turn all LEDS off, as they may be in the state last left in

   sendString("ATSPIO,14,1,0\r"); //level 2// ok to skip the firt one, never on on power up
   delayMS(150);
   sendString("ATSPIO,0,1,0\r"); //level 3
   delayMS(150);
   sendString("ATSPIO,9,1,0\r"); // level 4
   delayMS(150);
   sendString("ATSPIO,8,1,0\r");  // level 5
   delayMS(150);
   sendString("ATSPIO,7,1,0\r");  // level 6
   delayMS(150);


   // manage pins, turn all LEDS on
   sendString("ATSLED,1,0,1\r"); //free pin 5 from internal LED control
   delayMS(150);
   sendString("ATSPIO,5,1,1\r"); //bat level 1
   delayMS(150);
   sendString("ATSPIO,14,1,1\r"); //level 2
   delayMS(150);
   sendString("ATSPIO,0,1,1\r"); //level 3
   delayMS(150);
   sendString("ATSPIO,9,1,1\r"); // level 4
   delayMS(150);
   sendString("ATSPIO,8,1,1\r");  // level 5
   delayMS(150);
   sendString("ATSPIO,7,1,1\r");  // level 6
   delayMS(150);
   //sendString("ATSN,LEIFtech\r"); //also give local bluetooth a new name
   //delayMS(150);
   sendString("ATSLED,0,100,65535\r"); //pin 2 is connectin status LED
}

void initPowerPins()
{
  EALLOW;
  GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;

    //Initial Battery manager // maybe do this late in case TI stuff overwrites this
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1; // FETDRIVER_IN1 //Set battery FET control pins to out
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1; // FETDRIVER_IN2
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1; // FETDRIVER_IN3
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; // FETDRIVER_IN4

    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0; //So 44 = 44
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0; //So 54 = 54
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0; //So 32 = 32
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0; //So 33 = 33

    GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1; //clear all latches
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO44 = 1; //diable all pullups
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;
    EDIS;

    GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1; //maybe go ahead and disconect battery 2
    GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
    GpioDataRegs.GPBSET.bit.GPIO33 = 1; // disconnect battery 2
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;
    current_bat = 0;
    other_bat = 1;
}

void pattenStart()
{
  fs_counter++;
  fs_phase += fs_phase_speed; //phase is location, phase increment is speed
  if(fs_phase > _IQ(6.283)) { //6.283 is 2pi, or a full magnetic rotation
    fs_phase -= _IQ(6.283);
  }
  if (fs_counter > fsThresholdShort)
      { //threshold and counter marks sampling rate
        fs_counter = 0;
        if (fs_phase_speed <= fs_speed_limit) { // accelerate up to reaching to this speed //was 0.018 in lab5 , was 0.024 for starting skateboard tests
          fs_phase_speed += fs_phase_accelerate;
        }
        if (fs_phase_accelerate < _IQ(0.00005)) { //accelerattion slower at very start and ramp up was 0.0005
          fs_phase_accelerate += _IQ(0.00001); //was 0.0001
        }
        if (fs_phase_speed >= fs_speed_limit){ //was 0.018 in lab5, 0.0275 correlates to 450rpms, i think
          fs_mode = 0; // trust flying start
          Flag_Enable_Inverter = false;
          coasting = true;
          //OR use below to do permanent FS for debugs, lower fs_power if used!!
          //fs_phase_speed = fs_speed_limit;

        }
      }
      gPwmDataA.Tabc.value[0] = _IQmpy(fs_power, _IQsin(fs_phase));
      gPwmDataA.Tabc.value[1] = _IQmpy(fs_power, _IQsin(fs_phase - _IQ(2.094)));
      gPwmDataA.Tabc.value[2] = _IQmpy(fs_power, _IQsin(fs_phase - _IQ(4.189)));

      gPwmDataB.Tabc.value[0] = _IQmpy(fs_power, _IQsin(fs_phase));
      gPwmDataB.Tabc.value[1] = _IQmpy(fs_power, _IQsin(fs_phase - _IQ(2.094)));
      gPwmDataB.Tabc.value[2] = _IQmpy(fs_power, _IQsin(fs_phase - _IQ(4.189)));
}


void batteryAhEstimator() //look at battery level, assume there are 2, and estimate remaining Amp hours XXX
{
	uint_least8_t bat_temp = bat_actual; //for debug

	bat_level[current_bat] = AdcResult.ADCRESULT7;
	/*if ( bat_level[current_bat] < 3375) //25V	 			range(3000 to 3375)		 ext(3215 && 3415) 	r200		true medium is 3315
	{
		bat_actual = 0;
	}
	else if ( bat_level[current_bat] < 3454)//25.56V 		range(3375 to 3454)  	 ext(3350 && 3483)  r133      	true medium is 3415
	{
		bat_actual = 1;
	}
	else if ( bat_level[current_bat] < 3512)//26.0V 		range(3454 to 3512) 	 ext(3415 && 3522) 	r107		true medium is 3483
	{
		bat_actual = 2;
	}
	else if ( bat_level[current_bat] < 3532)//26.15V 		range(3512 to 3532) 	 ext(3483 && 3537) 	r54			true medium is 3522
	{
		bat_actual = 3;
	}
	else if ( bat_level[current_bat] < 3543)//26.23V 		range(3532 to 3543) 	 ext(3522 && 3561) 	r39			true medium is 3537
	{
		bat_actual = 4;
	}
	else if ( bat_level[current_bat] < 3570) //X26.49VX	range(3543 to 3570) 	 ext(3537 && 3585) 	r48		true medium is 3556
	{
		bat_actual = 5;
	}
	else // >26.49V										range(3580 to 4000)   	 ext(3561 && 3761) 	r200		true medium is 3661
	{
		bat_actual = 6;
	}*/

	if ( bat_level[current_bat] < 3375) //25V
	{
		bat_actual = 0;
	}
	else if ( bat_level[current_bat] < 3454)//25.56V
	{
		bat_actual = 1;
	}
	else if ( bat_level[current_bat] < 3512)//26.0V
	{
		bat_actual = 2;
	}
	else if ( bat_level[current_bat] < 3535)//26.15V
	{
		bat_actual = 3;
	}
	else if ( bat_level[current_bat] < 3550)//26.23V
	{
		bat_actual = 4;
	}
	else if ( bat_level[current_bat] < 3585) //26.49V ??
	{
		bat_actual = 5;
	}
	else // >26.49V
	{
		bat_actual = 6;
	}




	battery_Ah = _IQmpy(_IQ(4.6), _IQ((float)bat_actual / 6.0)); //battery cells are rated 2300mah each, assume both packs are in
	battery_Ah_half = _IQdiv(battery_Ah, _IQ(2.0));


	///////for debug.. so if the estimator ever has to correct battery level in bat_actual, we can ask to see in what way
	if (bat_actual > bat_temp)
	{
		battery_calc_was_low++;
	}

	else if (bat_actual < bat_temp)
	{
		battery_calc_was_high++;
	}

	else if (bat_actual == bat_temp)
	{
		battery_calc_was_right++;
	}
}


void batAhSubtractUsed()
{
	battery_Ah = battery_Ah - _IQmpy(overall_I, _IQ(0.0000277)); //0.0000277 represents the ration of 10th a second to one hour. 1/(10*60*60)
	battery_Ah_half = battery_Ah_half - _IQmpy(overall_I, _IQ(0.0000277));


	if (batteries_are_equal)
	{
		if ( battery_Ah < _IQ(0.4))
		{
			bat_actual = 0;
		}
		else if (battery_Ah < _IQ(1.1))
		{
			bat_actual = 1;
		}
		else if (battery_Ah < _IQ(1.8))
		{
			bat_actual = 2;
		}
		else if ( battery_Ah < _IQ(2.5))
		{
			bat_actual = 3;
		}
		else if ( battery_Ah < _IQ(3.2))
		{
			bat_actual = 4;
		}
		else if ( battery_Ah < _IQ(3.9))
		{
			bat_actual = 5;
		}
		else // < 4.6
		{
			bat_actual = 6;
		}
	}

	else//only one pack active
	{
		if ( battery_Ah_half < _IQ(0.2))
		{
			bat_actual = 0;
		}
		else if (battery_Ah_half < _IQ(0.55))
		{
			bat_actual = 1;
		}
		else if (battery_Ah_half < _IQ(0.9))
		{
			bat_actual = 2;
		}
		else if ( battery_Ah_half < _IQ(1.25))
		{
			bat_actual = 3;
		}
		else if ( battery_Ah_half < _IQ(1.6))
		{
			bat_actual = 4;
		}
		else if ( battery_Ah_half < _IQ(1.95))
		{
			bat_actual = 5;
		}
		else // < 2.3
		{
			bat_actual = 6;
		}
	}
}


void batteryPriorityManager() //connects to on ebattery at a time and declares which one is priority battery and if they are equal
{
  delayMSinterrupt(5);//5ms
  bat_level[current_bat] = AdcResult.ADCRESULT7; //measure current battery
  //take a look at "other" battery
  GpioDataRegs.GPBTOGGLE.bit.GPIO33 = 1;
  GpioDataRegs.GPBTOGGLE.bit.GPIO32 = 1;
  GpioDataRegs.GPBTOGGLE.bit.GPIO44 = 1;
  GpioDataRegs.GPBTOGGLE.bit.GPIO54 = 1;
  delayMSinterrupt(5);//5ms
  bat_level[other_bat] = AdcResult.ADCRESULT7; // measure other battery


  if (bat_level[current_bat] > bat_level[other_bat]) //then return to "current" battery and keep the labels as were
  {
	  GpioDataRegs.GPBTOGGLE.bit.GPIO33 = 1; //
	  GpioDataRegs.GPBTOGGLE.bit.GPIO32 = 1;
	  GpioDataRegs.GPBTOGGLE.bit.GPIO44 = 1;
	  GpioDataRegs.GPBTOGGLE.bit.GPIO54 = 1;
  }

  else //"other" battery is stonger so stay here and change the labels
  {
	  if (current_bat == 0)
	  {
		  current_bat = 1;
		  other_bat = 0;
	  }
	  else
	  {
		  current_bat = 0;
		  other_bat = 1;
	  }
  }

  if (abs(bat_level[current_bat] - bat_level[other_bat]) < 15) // That's pretty darn close right?
  {
    batteries_are_equal = true; // then whenever PWM is called, both batteries will connect
  }

  else
  {
    batteries_are_equal = false;
  }
}


void batteryPair() //called just before enabling PWM, or sleep. Connects the low priority battery
{
	if (batteries_are_equal && !batteries_are_paired)
	{

		if (current_bat == 0)
		{
			GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1; //connect other bat aka 1
			GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
		}

		else // == 1
		{
			GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1; //connect other bat aka 0
			GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
		}
		batteries_are_paired = true;
	}

	else if (need_based_battery_pairing_allowed && back_motorI_average > _IQ(7.0))// need based battery pairing needs to be tested, for now it is disabled
	{
		if (!batteries_are_paired)
			{
				if (current_bat == 0)
				{
					GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1; //connect other bat aka 1
					GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
				}

				else // == 1
				{
					GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1; //connect other bat aka 0
					GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1;
				}
				batteries_are_paired = true;
			}
	}

}

void batteryUnPair() //called after disabling PWM, or end of sleep
{
	if (batteries_are_paired)
	{
		if (current_bat == 0)
		{
			GpioDataRegs.GPBSET.bit.GPIO33 = 1; //disconnect other bat aka 1
			GpioDataRegs.GPBSET.bit.GPIO32 = 1;
		}

		else // == 1
		{
			GpioDataRegs.GPBSET.bit.GPIO44 = 1; //disconnect other bat aka 0
			GpioDataRegs.GPBSET.bit.GPIO54 = 1;
		}
		batteries_are_paired = false;
	}
}


void batReadlow()
{
	if ( bat_level[current_bat] < 3150) // way under 25 V, had it at 3300 till recently
    {
        bat_actual = 0;
    }
}


void batUpdateDisplay()
{
  uint_least8_t pin[6] = {5,14,0,9,8,7}; //LED pin names on bluetooth module
  char temp_num[3];//need because we are using ltoa. this is because one of the pins is 14, which cant be represented as a single char, but must be a string
  if (bat_show != bat_actual) //here because of the next else if
  {
    while (bat_show != bat_actual)
    {
      if (bat_show > bat_actual)
      {
        ltoa(pin[bat_show - 1],temp_num);//gives me the pin number i want to control
        sendString("ATSPIO,");
        sendString(temp_num);
        sendString(",1,0\r");//trun off
        delayMSinterrupt(100);
        bat_show--; //battery status shown is now one less
      }

      else if (bat_show < bat_actual)
      {
        ltoa(pin[bat_show],temp_num);//gives me the pin number i want to control
        sendString("ATSPIO,");
        sendString(temp_num);
        sendString(",1,1\r"); //turn on
        delayMSinterrupt(100);
        bat_show++;//battery status shown is now one more
      }
    }
  }

  else if (bat_show == 0) // means we just updated to zero level battery, so we blink the LED
  {
	  if(bat_LED_toggledON)
	  {
		  sendString("ATSPIO,5,1,0\r");
		  delayMSinterrupt(100);
		  bat_LED_toggledON = false;
	  }
	  else
	  {
		  sendString("ATSPIO,5,1,1\r");
		  delayMSinterrupt(100);
		  bat_LED_toggledON = true;
	  }
  }
}


void sendString(char s[]) //note that s[n] is outgoing, my_buffer[i] is incoming
{
  int n =0;
  while (s[n] != '\0')
  {
    while(!ScibRegs.SCICTL2.bit.TXRDY);
    ScibRegs.SCITXBUF = s[n];
    n++;
  }
}


void captureUART() //sciB version
{
  if (ScibRegs.SCIRXST.bit.RXRDY) //looking for receive flag
  {
    //RX_flag = true;// i think this should be commented out???? Feb 17th!! XXX
    char temp_char = ScibRegs.SCIRXBUF.all; //read RX register
    if (temp_char!= '\n') //ignore all '\n'
    {
      my_buffer[my_i] = temp_char;
      if(my_buffer[my_i]=='\r' )  // probably end of viable command, ok to look at now
      {
        RX_flag = true;        //Set String received flag
        my_buffer[my_i]='\0';   // replace \r with string terminator \0
        my_i=0;
      }
      else{my_i++;}

      if (my_i > MY_BUFFER_LEN - 2) // maybe missed the \r, try to process the buffer, but more importantly reset the buffer to avoid overrun
      {
        RX_flag = true;
        my_buffer[MY_BUFFER_LEN - 1]='\0';
        my_i=0;
      }
    }
  }
}


int processUARTbuffer() //process text coming from the remote. mostly looking !!onebitnumber!!onebitnumber
{
  uint_least8_t m = 0; //index for this buffer
  if (RX_flag)
  {
    RX_flag = false;
    if (my_buffer[1] == '!' && my_buffer[2] == '!')// this takes into account a possible trailing character at start of buffer and moved the index up by one
    {
      m = 1;
    }


    if ((my_buffer[m] == '!' && my_buffer[m+1]== '!') && (my_buffer[m+3] == '!' && my_buffer[m+4]== '!')) //incoming motor power instruction
    {
      if (my_buffer[m+2] == my_buffer[m+5])
      {
        float temp_remote_val = 0.1 * (float)my_buffer[m+2]; // scaling it down to 1/10th to fit IQ format -127 to 127
        remote_command_val = _IQ(temp_remote_val);

        if (safetyStop_pingPONG_enabled)
        {
           	bluetooth_connected = true;
           	bluetooth_disconnected_counter = 0;
           	safety_stop = false;
           	safety_stop_counter = 0;
        }
        silence_counter = 0;
        silence_state = false; // we just got a speed command, so we are clearly not in silent situation

        return 1;
      }

      else
      {
        return 0;// will lead to eventual slow down
      }
    }

    else if (my_buffer[m+2] == 'c' && my_buffer[m+3] == 'o' &&  my_buffer[m+4] == 'a' &&  my_buffer[m+5] == 's' &&  my_buffer[m+6] == 't')//This will immediately engage coast
    {
      coasting = true;
      batteryUnPair();
      Flag_Enable_Inverter = false;
      silence_counter = 0;
      silence_state = true;
      fs_recently_used = 0 ;
      return 0;
    }

    else if (my_buffer[m+2] == 's' && my_buffer[m+3] == 't' &&  my_buffer[m+4] == 'a' &&  my_buffer[m+5] == 't' &&  my_buffer[m+6] == 's')//purely for debug
    {

    	unsigned char records_string[22];
    	char signed_records_string[10];

    	ltoa(front_motor_index, signed_records_string);
    	sendString("front motor index: ");
    	sendString(signed_records_string);
    	sendString("\r\n");

    	ftoa(records_string, _IQtoF(speed_record));
    	sendString("max_speed_gone: ");
    	sendStringU(records_string);
    	sendString("\r\n");

    	ftoa(records_string, _IQtoF(speed_record_demanded));
    	sendString("max_speed_demanded: ");
    	sendStringU(records_string);
    	sendString("\r\n");

    	ftoa(records_string, _IQtoF(back_motorI_average_record));
    	sendString("max_I_B: ");
    	sendStringU(records_string);
    	sendString("\r\n");

    	ftoa(records_string, _IQtoF(back_power_limit_record));
    	sendString("max_limit: ");
    	sendStringU(records_string);
    	sendString("\r\n");

    	ltoa(power_high_counter_record, signed_records_string);
    	sendString("max_limit_time: ");
    	sendString(signed_records_string);
    	sendString("\r\n");

    	sendString("batteries: ");
    	if (batteries_are_equal)
    	{
    		sendString("equal with ");
    		ftoa(records_string, _IQtoF(battery_Ah));
    		sendStringU(records_string);
    	    sendString(" Ah");
    	}

    	else
    	{
    		sendString("side ");
    		ltoa(current_bat, signed_records_string);
    		sendString(signed_records_string);
    		sendString("with ");
    		ftoa(records_string, _IQtoF(battery_Ah));
    		sendStringU(records_string);
    		sendString(" Ah");
    	}
    	sendString("\r\n");

    	ltoa(bat_level[current_bat], signed_records_string);
    	sendString("battery ADC: ");
    	sendString(signed_records_string);
    	sendString("\r\n");

    	ltoa(battery_calc_was_low, signed_records_string);
    	sendString("bat_calc was low: ");
    	sendString(signed_records_string);
    	sendString("\r\n");

    	ltoa(battery_calc_was_high, signed_records_string);
    	sendString("was high: ");
    	sendString(signed_records_string);
    	sendString("\r\n");

    	ltoa(battery_calc_was_right, signed_records_string);
    	sendString("was right: ");
    	sendString(signed_records_string);
    	sendString("\r\n\n");
    	return 0;
    }

    else if (my_buffer[m+2] == 'p' && my_buffer[m+3] == 'o' &&  my_buffer[m+4] == 'n' &&  my_buffer[m+5] == 'g')//XXX ping back from the remote occured, connection can be considered good, even if otherwise silence
    {
    	 bluetooth_connected = true;
    	 bluetooth_disconnected_counter = 0;
    	 safety_stop = false; //connection is live so do not call safety stop
    	 safety_stop_counter = 0;
    	 return 0;
    }

    else if (my_buffer[m+2] == 'd' && my_buffer[m+3] == 'e' &&  my_buffer[m+4] == 'b' &&  my_buffer[m+5] == 'u' &&  my_buffer[m+6] == 'g')// request to have basically gain acces to motor control via dongle
    {
    	if (!debug)
    	{
    		debug = true;
    	}
    	else
    	{
    		debug = false;
    	}

    	return 0;
    }


    else//message was gibberish
    {
      //sendString("N\r"); // noise?
      return 0;// will lead to eventual slow down
    }
  }
  else //no RX flag at all
  {
    return 0;// will lead to eventual slow down
  }
}

_iq commandScalerToMotor() //takes the number sent from the remote into pomotr speed input
{
	_iq T = _IQ(0.0);
	// remote command val comes as _IQ(0.4) to _IQ(23.5), anything below negative_threshold returns as -T, and below valid_forward speed returns T=0, and above valid speed return +T

	if (remote_command_val > REMOTE_NEGATIVE_THRESHOLD && remote_command_val < REMOTE_FORWAR_THRESHOLD) //the command is generally around zero
	{
		T = _IQ(0.0);
	}

	else
	{
		T = _IQmpy(remote_command_val - REMOTE_FORWAR_THRESHOLD, REMOTE_COMMAND_SCALER); //gives both + and - values
	}
	return T;
}

void clearMyBuffer(volatile char s[], int n)
{
  int x;
  for (x = 0; x<= n; x++)
  {
    s[x] = '\0';
  }
}

/*
void getOrientation() //this funciton does nothing in expert mode
{
	uint_least16_t orientation_sensor_val  =  AdcResult.ADCRESULT8;
	if (orientation_sensor_val > 1700) //really high, can also mean failure of hall sensor usually disconnected
	{
		front_motor_index = 1;
	}
	else if (orientation_sensor_val < 800)
	{
		front_motor_index = 2;
	}
}
*/

int getOrientation() //determines orientation of the skateboard
{
	uint_least8_t orientation;
	uint_least16_t orientation_sensor_val  =  AdcResult.ADCRESULT8;

	if (orientation_sensor_val > 900) // Aaron changed from 1700 - really high, can also mean failure of hall sensor usually disconnected
	{
		orientation = 1; // aaron changed to 2
	}

	else if (orientation_sensor_val < 800) // Aaron changed from 800
	{
		orientation = 2;  //aaron changed to 1
	}

	else // Aaron commented out sensor value is between 800 and 1700, keep the last seen orientation
	{
		orientation = front_motor_index;
	}

	return orientation;
}


void delayMS(uint_least16_t thousandsUS) // used before interrupts is enabled
{
	uint_least16_t n;
	for (n = thousandsUS; n>0; n--)
	{
		DELAY_US(1000);
	}
}

void delayMSinterrupt(uint_least32_t delay_in_ms) // use while interrupts are enabled. Do not use delay near or above 1000ms, you may end up in permanent delay / wait state. Max 900 to be safe
{
	int_least32_t cnt_target = HAL_readTimerCnt(halHandle,0) - (delay_in_ms*90000); // 1 ms equals 90 000 timer cycles
	if (cnt_target < 0 )
	{
		cnt_target = cnt_target  + 90000000;
		while (cnt_target > HAL_readTimerCnt(halHandle,0)){} //if cnt_target overflowed, wait for cnt_now to overflow as well.
	}
	while (cnt_target <  HAL_readTimerCnt(halHandle,0)){} // now wait for cnt_now to go past cnt_target

}


///////////////////////////////////////////////////////////////TI revised functions

void updateGlobalVariables_motorA(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  //LK FORCE angle watcher
  gMotorVarsA.ForceAngleStatus = EST_getForceAngleStatus(obj->estHandle);

  // get the speed estimate
  gMotorVarsA.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVarsA.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVarsA.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVarsA.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVarsA.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVarsA.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator resistance online
  gMotorVarsA.RsOnLine_Ohm = EST_getRsOnLine_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVarsA.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVarsA.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVarsA.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVarsA.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVarsA.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVarsA.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVarsA.VdcBus_kV = _IQmpy(gAdcDataA.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));
}

void updateGlobalVariables_motorB(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  gMotorVarsA.ForceAngleStatus = EST_getForceAngleStatus(obj->estHandle);
  // get the speed estimate
  gMotorVarsB.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

  // get the real time speed reference coming out of the speed trajectory generator
  gMotorVarsB.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(obj->estHandle));

  // get the torque estimate
  gMotorVarsB.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVarsB.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVarsB.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVarsB.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator resistance online
  gMotorVarsB.RsOnLine_Ohm = EST_getRsOnLine_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVarsB.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVarsB.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVarsB.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVarsB.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVarsB.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVarsB.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVarsB.VdcBus_kV = _IQmpy(gAdcDataB.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGainsA(CTRL_Handle handle)
{
  if((gMotorVarsA.CtrlState == CTRL_State_OnLine) && (gMotorVarsA.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdateA == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVarsA.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVarsA.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVarsA.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVarsA.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVarsA.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVarsA.Ki_Idq);

  }

  return;
} // end of updateKpKiGains() function


void updateKpKiGainsB(CTRL_Handle handle)
{
  if((gMotorVarsB.CtrlState == CTRL_State_OnLine) && (gMotorVarsB.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdateB == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVarsB.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVarsB.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVarsB.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVarsB.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVarsB.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVarsB.Ki_Idq);
  }

  return;
} // end of updateKpKiGains() function


void updateIqRefA(CTRL_Handle handle) //LLL added from lab 4
{
  _iq iq_refA = _IQmpy(gMotorVarsA.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

  // set the speed reference so that the forced angle rotates in the correct direction for startup
  if(_IQabs(gMotorVarsA.Speed_krpm) < _IQ(0.01))
    {
      if(iq_refA < _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(-0.01));
        }
      else if(iq_refA > _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(0.01));
        }
    }

  // Set the Iq reference that use to come out of the PI speed control
  CTRL_setIq_ref_pu(handle, iq_refA);

  return;
} // end of updateIqRef() function


void updateIqRefB(CTRL_Handle handle) //LLL added from lab 4, but heavily edited
{
  _iq iq_refB = _IQmpy(gMotorVarsB.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

  // set the speed reference so that the forced angle rotates in the correct direction for startup
  if(_IQabs(gMotorVarsB.Speed_krpm) < _IQ(0.01))
    {
      if(iq_refB < _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(-0.01));
        }
      else if(iq_refB > _IQ(0.0))
        {
          CTRL_setSpd_ref_krpm(handle,_IQ(0.01));
        }
    }

  // Set the Iq reference that use to come out of the PI speed control
  CTRL_setIq_ref_pu(handle, iq_refB);

  return;
} // end of updateIqRef() function


void runRsOnLineA(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // execute Rs OnLine code
  if(gMotorVarsA.Flag_Run_Identify == true)
    {
      if(EST_getState(obj->estHandle) == EST_State_OnLine)
        {
        float_t RsError_Ohm = gMotorVarsA.RsOnLine_Ohm - gMotorVarsA.Rs_Ohm;

          EST_setFlag_enableRsOnLine(obj->estHandle,true);
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQmpy(gMotorVarsA.RsOnLineCurrent_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

          if(abs(RsError_Ohm) < (gMotorVarsA.Rs_Ohm * 0.05))
            {
              EST_setFlag_updateRs(obj->estHandle,true);
            }
        }
      else
        {
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLineId_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLine_pu(obj->estHandle,_IQ(0.0));
          EST_setFlag_enableRsOnLine(obj->estHandle,false);
          EST_setFlag_updateRs(obj->estHandle,false);
          EST_setRsOnLine_qFmt(obj->estHandle,EST_getRs_qFmt(obj->estHandle));
        }
    }

  return;
} // end of runRsOnLine() function

void runRsOnLineB(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  // execute Rs OnLine code
  if(gMotorVarsB.Flag_Run_Identify == true)
    {
      if(EST_getState(obj->estHandle) == EST_State_OnLine)
        {
        float_t RsError_Ohm = gMotorVarsB.RsOnLine_Ohm - gMotorVarsB.Rs_Ohm;

          EST_setFlag_enableRsOnLine(obj->estHandle,true);
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQmpy(gMotorVarsB.RsOnLineCurrent_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

          if(abs(RsError_Ohm) < (gMotorVarsB.Rs_Ohm * 0.05))
            {
              EST_setFlag_updateRs(obj->estHandle,true);
            }
        }
      else
        {
          EST_setRsOnLineId_mag_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLineId_pu(obj->estHandle,_IQ(0.0));
          EST_setRsOnLine_pu(obj->estHandle,_IQ(0.0));
          EST_setFlag_enableRsOnLine(obj->estHandle,false);
          EST_setFlag_updateRs(obj->estHandle,false);
          EST_setRsOnLine_qFmt(obj->estHandle,EST_getRs_qFmt(obj->estHandle));
        }
    }

  return;
} // end of runRsOnLine() function


void updateCPUusage(CTRL_Handle handle)
{
  uint32_t minDeltaCntObserved = CPU_USAGE_getMinDeltaCntObserved(cpu_usageHandle);
  uint32_t avgDeltaCntObserved = CPU_USAGE_getAvgDeltaCntObserved(cpu_usageHandle);
  uint32_t maxDeltaCntObserved = CPU_USAGE_getMaxDeltaCntObserved(cpu_usageHandle);
  uint16_t pwmPeriod = HAL_readPwmPeriod(halHandle,PWM_Number_1);
  float_t  cpu_usage_den = (float_t)pwmPeriod * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * 2.0;

  // calculate the minimum cpu usage percentage
  gCpuUsagePercentageMin = (float_t)minDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the average cpu usage percentage
  gCpuUsagePercentageAvg = (float_t)avgDeltaCntObserved / cpu_usage_den * 100.0;

  // calculate the maximum cpu usage percentage
  gCpuUsagePercentageMax = (float_t)maxDeltaCntObserved / cpu_usage_den * 100.0;

  return;
} // end of updateCPUusage() function


////////////////////////////////////////////////These functions are only used for debugg
void sendStringU(unsigned char s[])
{
	 int n =0;
	 while (s[n] != '\0')
	 {
	   while(!ScibRegs.SCICTL2.bit.TXRDY);
	   ScibRegs.SCITXBUF = s[n];

	   n++;
	 }
}

void ftoa(unsigned char *buf, float f) {
  unsigned int rem;
  unsigned char *s,length=0;
  int i;

  i = (int)((float)f*10);

  s = buf;
  if (i == 0){    //print 0.0 with null termination here
    *s++ = '0';
    *s++ = '.';
    *s++ = '0';
    *s=0;       //null terminate the string
  } else {
    if (i < 0) {
      *buf++ = '-';
      s = buf;
      i = -i;
    }
    //while-loop to "decode" the long integer to ASCII by append '0', string in reverse manner
    //If it is an integer of 124 -> string = {'4', '2', '1'}
    while (i) {
      ++length;
      rem = i % 10;
      *s++ = rem + '0';
      i /= 10;
    }
    //reverse the string in this for-loop, string became {'1', '2', '4'} after this for-loop
    for(rem=0; ((unsigned char)rem)<length/2; rem++) {
      *(buf+length) = *(buf+((unsigned char)rem));
      *(buf+((unsigned char)rem)) = *(buf+(length-((unsigned char)rem)-1));
      *(buf+(length-((unsigned char)rem)-1)) = *(buf+length);
    }

    /* Take care of the special case of 0.x if length ==1*/
    if(length==1) {
      *(buf+2) = *buf;
      *buf = '0';
      *(buf+1) = '.';
      *(s+2)=0;       //null terminate
    } else {
      *(buf+length) = *(buf+length-1);
      *(buf+length-1)='.';
      *(s+1)=0;       //null terminate
    }
  }
}


__interrupt void WAKE_ISR(void)
{
   //GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;  // Toggle GPIO1 in the ISR - monitored with oscilloscope
   PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}


//@} //defgroup
// end of file



Status API Training Shop Blog About
© 2016 GitHub, Inc. Terms Privacy Security Contact Help