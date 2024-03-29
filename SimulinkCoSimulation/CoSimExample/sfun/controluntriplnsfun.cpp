
/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

/*
    Time: 2023-5-5

    Triple Pendulum S-function
    1. two Input(end effector postion x,y and velocity x,y) and two Output(joint torque )

*/


#define S_FUNCTION_NAME  controluntriplnsfun
#define S_FUNCTION_LEVEL 2

#include <string>
#include <iostream>
#include <memory>

#include "controluntriplnclient.hpp"
#include "simstruc.h"

/*
    Paramemer: HOST
               PORT 
               STEP
               Input SIZE define Input Number
               OUTPUT Number
*/
#define HOST_NAME_P    0
#define PORT_NUM_P     1

#define STEP_SIZE_P    2

#define INPUT_IDX      3
#define INPUT_PARAM(S) ssGetSFcnParam(S,INPUT_IDX)

#define OUTPUT_IDX     4
#define OUTPUT_PARAM(S) ssGetSFcnParam(S,OUTPUT_IDX)

#define NUM_PRMS       5

#define INPUT_NUM  18
#define OUTPUT_NUM 2

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)

/*====================*
 * S-function methods *
 *====================*/
// mdlCheckParameters
// Validate our parameters to verify they are okay
static void mdlCheckParameters(SimStruct *S)
{
    // check host number
    if (!mxIsChar(ssGetSFcnParam(S,HOST_NAME_P))) {
        ssSetErrorStatus(S,"Host name parameter must be a char array.");
        return;
    }
    
    // check port number
    if (!mxIsChar(ssGetSFcnParam(S,PORT_NUM_P))) {
        ssSetErrorStatus(S,"Port number parameter must be a char array.");
        return;
    }
    
    //check STEP_SIZE_P is double and it must greater than 0.
    bool isValid = mxIsDouble(ssGetSFcnParam(S,STEP_SIZE_P)) &&
        mxGetNumberOfElements(ssGetSFcnParam(S,STEP_SIZE_P)) == 1 &&
        !mxIsComplex(ssGetSFcnParam(S,STEP_SIZE_P));

    if (isValid) {
        double *v = reinterpret_cast<double *>(mxGetData(ssGetSFcnParam(S,STEP_SIZE_P)));
        if (*v < 0) isValid = false;
    }
    if (!isValid) {
        ssSetErrorStatus(S,"Communication interval parameter must be a positive real scalar of double data type.");
        return;
    }

    // check Input number is unsigned int and it must greater than 0.
    isValid = mxGetNumberOfElements(INPUT_PARAM(S)) == 1 &&
        !mxIsComplex(INPUT_PARAM(S));

    if (isValid){
        unsigned int *w = reinterpret_cast<unsigned int *>(mxGetData(INPUT_PARAM(S)));
        if (*w < 0) isValid = false;
    }
    if (!isValid) {
        ssSetErrorStatus(S,"Input single number must be a positive real scalar of int data type.");
        return;
    }

    // check Output number is unsigned int and it must greater than 0.
    isValid = mxGetNumberOfElements(OUTPUT_PARAM(S)) == 1 &&
        !mxIsComplex(OUTPUT_PARAM(S));

    if (isValid){
        unsigned int *w = reinterpret_cast<unsigned int *>(mxGetData(OUTPUT_PARAM(S)));
        if (*w < 0) isValid = false;
    }
    if (!isValid) {
        ssSetErrorStatus(S,"Output single number must be a positive real scalar of int data type.");
        return;
    }

    return;
}
#endif // MDL_CHECK_PARAMETERS


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PRMS);  /* Number of expected parameters */

#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; // Parameter mismatch will be reported by Simulink
    }
#endif
    
    // set
    ssSetSFcnParamTunable(S, HOST_NAME_P, false);
    ssSetSFcnParamTunable(S, PORT_NUM_P, false);
    ssSetSFcnParamTunable(S, STEP_SIZE_P, false);
    ssSetSFcnParamTunable(S, INPUT_IDX, false);
    ssSetSFcnParamTunable(S, OUTPUT_IDX, false);

    // define the Input Port 
    {
        int_T i;

        if (!ssSetNumInputPorts(S, INPUT_NUM)) return;
        for (i = 0; i < INPUT_NUM; i++) {
            ssSetInputPortWidth(S, i, 1);
            ssSetInputPortDataType(S, i, SS_DOUBLE);
            ssSetInputPortComplexSignal(S, i, COMPLEX_NO);
            ssSetInputPortRequiredContiguous(S, i, 1);
            ssSetInputPortDirectFeedThrough(S, i, 0);
        }
     }

    // define the Output Port 
    {
        int_T i;
        if (!ssSetNumOutputPorts(S, OUTPUT_NUM)) return;
        for (i = 0; i < OUTPUT_NUM; i++) {
            ssSetOutputPortWidth(S, i, 1);
            ssSetOutputPortDataType(S, i, SS_DOUBLE);
            ssSetOutputPortComplexSignal(S, i, COMPLEX_NO);
        }
     }

    ssSetNumSampleTimes(S, 1);

    // Specify the sim state compliance to be same as Simulink built-in block
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    
    ssSetModelReferenceNormalModeSupport(S, MDL_START_AND_MDL_PROCESS_PARAMS_OK);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    double *stepSizeP = reinterpret_cast<double *>(mxGetData(ssGetSFcnParam(S,STEP_SIZE_P)));
    
    ssSetSampleTime(S, 0, *stepSizeP);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
// mdlSetWorkWidths
//
static void mdlSetWorkWidths(SimStruct *S)
{
    // Declare the PWorks vectors
    ssSetNumPWork(S, 1);

    ssSetNumDWork(S, 1);
    
    ssSetDWorkWidth(S, 0, 1);
    ssSetDWorkDataType(S, 0, SS_UINT32);
}
#endif // MDL_SET_WORK_WIDTHS

#define MDL_PROCESS_PARAMETERS
#if defined(MDL_PROCESS_PARAMETERS) && defined(MATLAB_MEX_FILE)
// mdlProcessParameters
// Update run-time parameters.
static void mdlProcessParameters(SimStruct *S)
{
    // Update Run-Time parameters
    ssUpdateAllTunableParamsAsRunTimeParams(S);
}
#endif // MDL_PROCESS_PARAMETERS

#define GET_ZM_PTR(S) ssGetPWorkValue(S,0)

auto Mx_Deleter = [](char *m) { mxFree(m); };
using mxCharUnqiuePtr = std::unique_ptr<char, decltype(Mx_Deleter)>;

static std::string host_and_port_addr(const SimStruct *S)
{
    mxCharUnqiuePtr hostStr(mxArrayToString(ssGetSFcnParam(S,HOST_NAME_P)), Mx_Deleter);
    mxCharUnqiuePtr portStr(mxArrayToString(ssGetSFcnParam(S,PORT_NUM_P)), Mx_Deleter);
    
    std::string connStr = "tcp://";
    connStr += hostStr.get();
    connStr += ":";
    connStr += portStr.get();

    return connStr;
}

#define MDL_SETUP_RUNTIME_RESOURCES
// Called at the beginning of one or multiple simulations.
void mdlSetupRuntimeResources(SimStruct *S)
{
    std::cout << "Opening connection with server" << std::endl;
    std::string connStr = host_and_port_addr(S);
    ssSetPWorkValue(S, 0, setupruntimeresources_wrapper(connStr));
}


#define MDL_START // to indicate that the S-function has mdlStart method

// mdlStart
// Called at the beginning of every simulation. Called at every Fast Restart.
static void mdlStart(SimStruct *S)
{
    unsigned int *iter = reinterpret_cast<unsigned int *>(ssGetDWork(S,0));
    // Initialize previous states and iteration counter
    start_wrapper(iter);
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 *    Send Input data, get 2 Output Data.
 */ 
static void mdlOutputs(SimStruct *S, int_T tid)
{
    double *y1_ptr = reinterpret_cast<double *>(ssGetOutputPortSignal(S,0));
    double *y2_ptr = reinterpret_cast<double *>(ssGetOutputPortSignal(S,1));
    unsigned int *iter_ptr = reinterpret_cast<unsigned int *>(ssGetDWork(S,0));

    try {
        outputs_wrapper(GET_ZM_PTR(S), iter_ptr, y1_ptr, y2_ptr);
    } catch (std::exception &e) {
        static std::string errstr(e.what());
        ssSetErrorStatus(S, errstr.c_str());
        return;
    }
    
}


#define MDL_UPDATE  /* Change to #undef to remove function */
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   * Send Input data
   */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    unsigned int *iter_ptr = reinterpret_cast<unsigned int *>(ssGetDWork(S,0));
        
    int_T port;
    double Input_data[INPUT_NUM] = {0} ; 

    for(port = 0; port < INPUT_NUM; port++)
    {
        Input_data[port] = (*(reinterpret_cast<const double *>(ssGetInputPortSignal(S,port))));
    }

    update_wrapper(GET_ZM_PTR(S), iter_ptr, Input_data, INPUT_NUM);      
}

#define MDL_CLEANUP_RUNTIME_RESOURCES /* Change to #undef to remove function */
/* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
static void mdlCleanupRuntimeResources(SimStruct *S)
{
    std::cout << "Closing connection with server" << std::endl;
    cleanupruntimeresouces_wrapper(GET_ZM_PTR(S));
}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    unsigned int *iter_ptr = reinterpret_cast<unsigned int *>(ssGetDWork(S,0));
    try {
        terminate_wrapper(GET_ZM_PTR(S), iter_ptr);
    } catch (std::exception &e) {
        static std::string errstr(e.what());
        ssSetErrorStatus(S, errstr.c_str());
        return;
    }
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
