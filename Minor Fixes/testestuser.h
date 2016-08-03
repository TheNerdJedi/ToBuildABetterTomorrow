#if (USER_MOTOR == leifMotorA)							//motor name
#define USER_MOTOR_TYPE					MOTOR_Type_Pm
#define USER_MOTOR_NUM_POLE_PAIRS		()				// Pairs not poles, Used to calc user RPM from rotor Hz 
#define USER_MOTOR_Rr					(NULL)
#define USER_MOTOR_Rs 					()				// Identified phase to neutral resistance in a Y equivalent circuit (Ohms, float)
#define USER_MOTOR_Ls_d					()				// PM, Identified average stator inductance (Henry, float)
#define USER_MOTOR_Ls_q 				()				// PM, Identified average stator inductance (Henry, float) 
#define USER_MOTOR_RATED_FLUX			()				// Identified TOTAL flux linkage between rotor and stator (V/Hz)
#define USER_MOTOR_MAGNETIZING_CURRENT	(NULL)
#define USER_MOTOR_RES_EST_CURRENT		()				// During Motor ID, maximum current (Amperes, float) used for Rs estimation, 10-20% rated current
#define USER_MOTOR_IND_EST_CURRENT		()				// During Motor ID, maximum current (negative Amperes, float) used for Ls estimation, use just enough to enable rotation
#define USER_MOTOR_MAX_CURRENT			()				// CRITICAL: Used during ID and run-time, sets a limit on the maximum current command output of the provided Speed PI Controller to the Iq controller
#define USER_MOTOR_FLUX_EST_FREQ_Hz		()				// During Motor ID, maximum commanded speed (Hz, float), ~10% rated


#if (USER_MOTOR == leifMotorB)
#define USER_MOTOR_TYPE					Motor_Type_Pm