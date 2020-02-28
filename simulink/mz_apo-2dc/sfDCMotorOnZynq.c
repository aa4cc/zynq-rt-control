/*
 * S-function to support DC Driver Board FPGA Peripheral
 *
 * Copyright (C) 2020 Lukas Cerny <cernylu6@fel.cvut.cz>
 * Copyright (C) 2015-2017 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 * Department of Control Engineering
 * Faculty of Electrical Engineering
 * Czech Technical University in Prague (CTU)
 *
 * The S-Function for ERT Linux can be distributed in compliance
 * with GNU General Public License (GPL) version 2 or later.
 * Other licence can negotiated with CTU.
 *
 * Next exception is granted in addition to GPL.
 * Instantiating or linking compiled version of this code
 * to produce an application image/executable, does not
 * by itself cause the resulting application image/executable
 * to be covered by the GNU General Public License.
 * This exception does not however invalidate any other reasons
 * why the executable file might be covered by the GNU Public License.
 * Publication of enhanced or derived S-function files is required
 * although.
 *
 * Linux ERT code is available from
 *    http://rtime.felk.cvut.cz/gitweb/ert_linux.git
 * More CTU Linux target for Simulink components are available at
 *    http://lintarget.sourceforge.net/
 *
 * sfuntmpl_basic.c by The MathWorks, Inc. has been used to accomplish
 * required S-function structure.
 */


#define S_FUNCTION_NAME  sfDCMotorOnZynq
#define S_FUNCTION_LEVEL 2

/*
 * The S-function has next parameters
 *
 * Sample time     - sample time value or -1 for inherited
 * Counter Mode    -
 * Counter Gating
 * Reset Control
 * Digital Filter
 */

#define PRM_TS(S)               (mxGetScalar(ssGetSFcnParam(S, 0)))
#define PRM_MOT_ID(S)           (mxGetScalar(ssGetSFcnParam(S, 1)))

#define PRM_COUNT                   2


#define PWORK_IDX_ZYNQDCMOTMEM_STATE       0
#define PWORK_IDX_ZYNQDCMOTPOS_STATE       1

#define PWORK_COUNT                 2

#define PWORK_ZYNQDCMOTMEM_STATE(S)        (ssGetPWork(S)[PWORK_IDX_ZYNQDCMOTMEM_STATE])
#define PWORK_ZYNQDCMOTPOS_STATE(S)        (ssGetPWork(S)[PWORK_IDX_ZYNQDCMOTPOS_STATE])

enum {
    sIn_N_MOT_PWM = 0,  /* PWM value from interval [-1, 1], dimensions: [1 x 1]  */
    sIn_N_NUM
};

/* Enumerated constants for output ports ******************************************** */
enum {
    sOut_N_IRC_POS,       /* IRC position [1 x 1] */
    sOut_N_NUM
};

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#ifndef WITHOUT_HW

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#include "mzapo_regs.h"
#include "phys_address_access.h"


#endif /*WITHOUT_HW*/

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    mdlCheckParameters verifies new parameter settings whenever parameter
   *    change or are re-evaluated during a simulation. When a simulation is
   *    running, changes to S-function parameters can occur at any time during
   *    the simulation loop.
   */
static void mdlCheckParameters(SimStruct *S)
{
    if ((PRM_TS(S) < 0) && (PRM_TS(S) != -1))
        ssSetErrorStatus(S, "Ts has to be positive or -1 for automatic step");
    if ((PRM_MOT_ID(S) < 0) || (PRM_MOT_ID(S) > 1)) {
        printf("Motor ID parameter: %d\n", PRM_MOT_ID(S));
        ssSetErrorStatus(S, "Motor ID has to be 0 or 1");
    }
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, PRM_COUNT);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S, "2-parameters required: Ts, MOT_ID");
        return;
    }

  #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
  #endif

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, sIn_N_NUM)) return;

    ssSetInputPortWidth(S, sIn_N_MOT_PWM, 1);
    /* ssSetInputPortDataType(S, sIn_N_MOT_PWM, SS_INT32); */

    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */

    if (!ssSetNumOutputPorts(S, sOut_N_NUM)) return;
    ssSetOutputPortWidth(S, sOut_N_IRC_POS, 1);
    ssSetOutputPortDataType(S, sOut_N_IRC_POS, SS_INT32);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, PWORK_COUNT);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    if (PRM_TS(S) == -1) {
        ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
        ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
    } else {
        ssSetSampleTime(S, 0, PRM_TS(S));
        ssSetOffsetTime(S, 0, 0.0);
    }
}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
static void mdlInitializeConditions(SimStruct *S)
{
  #ifndef WITHOUT_HW
    mem_address_map_t *memadrs_dcmot1 = (mem_address_map_t *)PWORK_ZYNQDCMOTMEM_STATE(S);
    int32_T *irc_pos = (int32_T *)PWORK_ZYNQDCMOTPOS_STATE(S);
    
    /* Reset IRC variable */
    *irc_pos = 0;
    
    /* Reset IRC counter (and disable DC motor PWM) */
	mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_CR_o, DCSPDRV_REG_CR_IRC_RESET_m);
	
	/* Set frequency of DC motor PWM to 20 kHz (period is given in multiples of 10 ns) */
	mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_PERIOD_o, 5000 & DCSPDRV_REG_PERIOD_MASK_m);
	
	/* Set DC motor PWM duty cycle to 0 (given in multiples of 10 ns, hence it should be less than 5000) */
	mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_DUTY_o, 0);
	
	/* Enable DC motor PWM */
	mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_CR_o, DCSPDRV_REG_CR_PWM_ENABLE_m);

  #endif /*WITHOUT_HW*/
}
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
static void mdlStart(SimStruct *S)
{
  #ifndef WITHOUT_HW
    
    /* ----- Init PWORK_ZYNQDCMOTMEM_STATE(S) ----- */
    mem_address_map_t *memadrs_dcmot1;
    PWORK_ZYNQDCMOTMEM_STATE(S) = NULL;
    
    /* Map physical address of DC motor interface to virtual address */
    if (PRM_MOT_ID(S) == 0) {
        memadrs_dcmot1 = mem_address_map_create(DCSPDRV_REG_BASE_PHYS_0, DCSPDRV_REG_SIZE, 0);
    } else {
        memadrs_dcmot1 = mem_address_map_create(DCSPDRV_REG_BASE_PHYS_1, DCSPDRV_REG_SIZE, 0);
    }
    
    /* Check for errors */
	if (memadrs_dcmot1 == NULL) {
        ssSetErrorStatus(S, "Error when accessing physical address.");
        return;
	}
    
    /* Save memory map structure to PWORK_ZYNQDCMOTMEM_STATE(S) */
    PWORK_ZYNQDCMOTMEM_STATE(S) = memadrs_dcmot1;
    
    /* ----- Init PWORK_ZYNQDCMOTPOS_STATE(S) ----- */
    int32_T *irc_pos_state;
    PWORK_ZYNQDCMOTPOS_STATE(S) = NULL;
    
    /* Alloc memory for position state */
    irc_pos_state = malloc(sizeof(*irc_pos_state));
    
    /* Check for errors */
    if (irc_pos_state == NULL) {
        ssSetErrorStatus(S, "Error when calling malloc.");
        return;
    }
    
    /* Update position */
    *irc_pos_state = 0;
    
    /* Save position to PWORK_ZYNQDCMOTPOS_STATE(S) */
    PWORK_ZYNQDCMOTPOS_STATE(S) = irc_pos_state;

  #endif /*WITHOUT_HW*/

    mdlInitializeConditions(S);
}
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int32_T *irc_pos_output = ssGetOutputPortSignal(S, sOut_N_IRC_POS);
 
  #ifndef WITHOUT_HW
    int32_T *irc_pos_state = (int32_T *)PWORK_ZYNQDCMOTPOS_STATE(S);
    *irc_pos_output = *irc_pos_state;
  #else /*WITHOUT_HW*/
    *irc_pos_output = 0;
  #endif /*WITHOUT_HW*/
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    InputRealPtrsType pwm_input = ssGetInputPortRealSignalPtrs(S, sIn_N_MOT_PWM);
    
  #ifndef WITHOUT_HW
    
    mem_address_map_t *memadrs_dcmot1 = (mem_address_map_t *)PWORK_ZYNQDCMOTMEM_STATE(S);
    int32_T *irc_pos = (int32_T *)PWORK_ZYNQDCMOTPOS_STATE(S);
    
    /* Get IRC position */
    *irc_pos = mem_address_reg_rd(memadrs_dcmot1, DCSPDRV_REG_IRC_o);
    
    /* Set PWM */
    real_T pwm;
    pwm = **(pwm_input) * 5000;
    if (pwm > 5000) pwm = 5000;
    if (pwm < -5000) pwm = -5000;
    
    if (pwm > 0) {
        mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_DUTY_o, (uint32_t)  pwm | DCSPDRV_REG_DUTY_DIR_A_m);
    } else {
        mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_DUTY_o, (uint32_t) -pwm | DCSPDRV_REG_DUTY_DIR_B_m);
    }
    
  #endif /*WITHOUT_HW*/
}
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
  #ifndef WITHOUT_HW
    
    mem_address_map_t *memadrs_dcmot1 = (mem_address_map_t *)PWORK_ZYNQDCMOTMEM_STATE(S);
    
    /* Set PWM to 0 */
    mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_DUTY_o, 0);
    
    /* Disable PWM */
    mem_address_reg_wr(memadrs_dcmot1, DCSPDRV_REG_CR_o, 0);
    
    int32_T *irc_pos = (int32_T *)PWORK_ZYNQDCMOTPOS_STATE(S);
    if (memadrs_dcmot1 != NULL) {
        PWORK_ZYNQDCMOTMEM_STATE(S) = NULL;
        free(memadrs_dcmot1);
    }
    
    if (irc_pos != NULL) {
        PWORK_ZYNQDCMOTPOS_STATE(S) = NULL;
        free(irc_pos);
    }
  #endif /*WITHOUT_HW*/
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
