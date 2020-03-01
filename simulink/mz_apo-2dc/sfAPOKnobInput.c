/*
 * S-function to Read Knob Value from MZ_APO Dial Inputs
 *
 * Copyright (C) 2020 Lukas Cerny <cernylu6@fel.cvut.cz>
 * Copyright (C) 2014-2020 Pavel Pisa <pisa@cmp.felk.cvut.cz>
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
 * The documenation for MZ_APO boards peripherals and board use
 * for Computer Architectures course
 *   https://cw.fel.cvut.cz/wiki/courses/b35apo/documentation/mz_apo/start
 * The VHDL sources of SPI connected LEDs and knobs peripheral
 *   https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top/tree/master/system/ip/spi_leds_and_enc_1.0/hdl
 * 
 * Linux ERT code is available from
 *    https://github.com/aa4cc/ert_linux
 * More CTU Linux target for Simulink components are available at
 *    http://lintarget.sourceforge.net/
 *
 * sfuntmpl_basic.c by The MathWorks, Inc. has been used to accomplish
 * required S-function structure.
 */


#define S_FUNCTION_NAME  sfAPOKnobInput
#define S_FUNCTION_LEVEL 2

/*
 * The S-function has next parameters
 *
 * Sample time     - sample time value or -1 for inherited
 * Channel         - knob 0 to 2
 * Initial Value
 */

#define PRM_TS(S)               (mxGetScalar(ssGetSFcnParam(S, 0)))
#define PRM_CHANNEL(S)          (mxGetScalar(ssGetSFcnParam(S, 1)))
#define PRM_INITIAL_VALUE(S)    (mxGetScalar(ssGetSFcnParam(S, 2)))

#define PRM_COUNT                   3

#define PWORK_IDX_KNOBMEM_STATE     0

#define PWORK_COUNT                 1

#define PWORK_KNOBMEM_STATE(S)     (ssGetPWork(S)[PWORK_IDX_KNOBMEM_STATE])

#define IWORK_IDX_CHANNEL           0
#define IWORK_IDX_VALUE_RAW         1
#define IWORK_IDX_VALUE_OFFS        2

#define IWORK_COUNT                 3

#define IWORK_CHANNEL(S)            (ssGetIWork(S)[IWORK_IDX_CHANNEL])
#define IWORK_VALUE_RAW(S)          (ssGetIWork(S)[IWORK_IDX_VALUE_RAW])
#define IWORK_VALUE_OFFS(S)         (ssGetIWork(S)[IWORK_IDX_VALUE_OFFS])

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include <limits.h>
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
    if ((PRM_CHANNEL(S) < 0) || (PRM_CHANNEL(S) > 2))
        ssSetErrorStatus(S, "valid IRC channel is 0, 1, or 2");
    if ((PRM_INITIAL_VALUE(S) < INT_MIN) || (PRM_INITIAL_VALUE(S) > INT_MAX))
        ssSetErrorStatus(S, "initial value has to be in int range");
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 0;
    int_T i;

    ssSetNumSFcnParams(S, PRM_COUNT);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S, "3-parameters requited: Ts, Channel, Initial value");
        return;
    }

  #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
  #endif

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, nInputPorts)) return;

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_INT32);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, IWORK_COUNT);
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
	int initial_value;
	int knob_value;

    /* ----- Init PWORK_KNOBMEM_STATE(S) ----- */
    mem_address_map_t *memadrs_knob;
    PWORK_KNOBMEM_STATE(S) = NULL;
    
    /* Map physical address of knobs to virtual address */
    memadrs_knob = mem_address_map_create(SPILED_REG_BASE_PHYS, SPILED_REG_SIZE, 0);
    
    /* Check for errors */
	if (memadrs_knob == NULL) {
        ssSetErrorStatus(S, "Error when accessing physical address.");
        return;
	}
    
    /* Save memory map structure to PWORK_KNOBMEM_STATE(S) */
    PWORK_KNOBMEM_STATE(S) = memadrs_knob;

    /* Read actual knobs position value from hardware */
    knob_value = mem_address_reg_rd(memadrs_knob, SPILED_REG_KNOBS_8BIT_o);

    IWORK_CHANNEL(S) = PRM_CHANNEL(S);
    initial_value = PRM_INITIAL_VALUE(S);

    knob_value >>= 8 * IWORK_CHANNEL(S);

    IWORK_VALUE_RAW(S) = (int8_t)knob_value;
    IWORK_VALUE_OFFS(S) = initial_value - knob_value;

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
    int32_T *y = ssGetOutputPortSignal(S,0);

  #ifndef WITHOUT_HW
    y[0] = IWORK_VALUE_RAW(S) + IWORK_VALUE_OFFS(S);
  #else /*WITHOUT_HW*/
    y[0] = 0;
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
  #ifndef WITHOUT_HW
	int knob_value;

    mem_address_map_t *memadrs_knob = (mem_address_map_t *)PWORK_KNOBMEM_STATE(S);

    /* Read actual knobs position value from hardware */
    knob_value = mem_address_reg_rd(memadrs_knob, SPILED_REG_KNOBS_8BIT_o);

    knob_value >>= 8 * IWORK_CHANNEL(S);
    knob_value &= 0xff;

    IWORK_VALUE_RAW(S) += (int8_t)(knob_value - IWORK_VALUE_RAW(S));

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
    mem_address_map_t *memadrs_knob = (mem_address_map_t *)PWORK_KNOBMEM_STATE(S);

    mem_address_unmap_and_free(memadrs_knob);
    
    PWORK_KNOBMEM_STATE(S) = NULL;
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
