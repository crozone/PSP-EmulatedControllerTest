// PSP-EmulatedControllerTest
// .prx plugin to demo external controller port emulation
//
// Ryan Crosby 2025

// We use our own sceCtrl_driver imports with imports.S
// Don't import <pspctrl.h> or link the pspctrl module or it will conflict!
#include "ctrl_imports.h"

#ifdef DEBUG
#include <pspdebug.h>
#include <pspdisplay.h>
#endif

#include <pspsdk.h>
#include <psptypes.h>
#include <pspkerror.h>
#include <pspkerneltypes.h>
#include <pspthreadman.h>

#include <stdbool.h>
#include <inttypes.h>


#define str(s) #s // For stringizing defines
#define xstr(s) str(s)

#ifdef DEBUG
#define DEBUG_PRINT(...) pspDebugScreenKprintf( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...) do{ } while ( 0 )
#endif

#define ONE_MSEC (1000)

#define TIMER_PERIOD (10 * ONE_MSEC)

// The controller port for which to handle input.
// Can be either:
// * SCE_CTRL_PORT_DS3
// * SCE_CTRL_PORT_UNKNOWN_2
#define CONTROLLER_PORT SCE_CTRL_PORT_DS3

// The smallest offset from the analog stick's center position defining the guaranteed
// range (center position +/- this offset) the stick returns to when being released.
//
// This is the same value used within the firmware to register analogue input and cancel the idle timer.
//
#define CTRL_ANALOG_PAD_CENTER_POS_ERROR_MARGIN (37)

// The minimum amount of stick movement from center to register as a directional input.
#define ANALOG_PAD_DIRECTION_THRESHOLD (CTRL_ANALOG_PAD_CENTER_POS_ERROR_MARGIN + 23)


#define MODULE_NAME "EmulatedControllerTest"
#define MAJOR_VER 1
#define MINOR_VER 1

#define MODULE_OK       0
#define MODULE_ERROR    1

// https://github.com/uofw/uofw/blob/7ca6ba13966a38667fa7c5c30a428ccd248186cf/include/common/errors.h
#define SCE_ERROR_OK                                0x0
#define SCE_ERROR_BUSY                              0x80000021

//
// PSP SDK
//
// We are building a kernel mode prx plugin
PSP_MODULE_INFO(MODULE_NAME, PSP_MODULE_KERNEL, MAJOR_VER, MINOR_VER);

// We don't allocate any heap memory, so set this to 0.
PSP_HEAP_SIZE_KB(0);
//PSP_MAIN_THREAD_ATTR(0);
//PSP_MAIN_THREAD_NAME(MODULE_NAME);

// We don't need a main thread since we only do basic setup during module start and won't stall module loading.
// This will make us be called from the module loader thread directly, instead of a secondary kernel thread.
PSP_NO_CREATE_MAIN_THREAD();

// We don't need any of the newlib features since we're not calling into stdio or stdlib etc
PSP_DISABLE_NEWLIB();

//
// Forward declarations
//
static int main_thread(SceSize args, void *argp);
static int start_main_thread(void);
static int stop_main_thread(void);
int module_start(SceSize args, void *argp);
int module_stop(SceSize args, void *argp);

//
// Globals
//
static SceUInt g_button_state = 0;
static SceUID g_mainThreadId = -1;

//
// Controller callback function
//
static
s32 ctrl_input_data_handler_func(void *pSrc, SceCtrlData2 *pDst)
{
    // pSrc is set up to point to g_button_state.
    // Currently g_button_state is always 0, but it is included here for extensibility.
    SceUInt* p_new_buttons = (SceUInt*)pSrc;
    SceUInt new_buttons = p_new_buttons != NULL ? *p_new_buttons : 0;

    u8 rightX = 0;
    u8 rightY = 0;

    // Demo - translate PSP analog input into DS3 directional pad buttons
    SceCtrlData pad_state;
    if(sceCtrlPeekBufferPositive(&pad_state, 1) >= 0) {

        // Test: Write right stick values as inverse of left stick
        rightX = 255 - pad_state.aX;
        rightY = 255 - pad_state.aY;

        int pad_x = pad_state.aX - SCE_CTRL_ANALOG_PAD_CENTER_VALUE;
        int pad_y = pad_state.aY - SCE_CTRL_ANALOG_PAD_CENTER_VALUE;

        SceUInt direction_buttons = 0;
        if(pad_x > ANALOG_PAD_DIRECTION_THRESHOLD) {
            direction_buttons |= SCE_CTRL_RIGHT;
        }

        if(pad_x <= -ANALOG_PAD_DIRECTION_THRESHOLD) {
            direction_buttons |= SCE_CTRL_LEFT;
        }

        if(pad_y > ANALOG_PAD_DIRECTION_THRESHOLD) {
            direction_buttons |= SCE_CTRL_DOWN;
        }

        if(pad_y <= -ANALOG_PAD_DIRECTION_THRESHOLD) {
            direction_buttons |= SCE_CTRL_UP;
        }

        new_buttons |= direction_buttons;
    }

    if(new_buttons) {
        DEBUG_PRINT("Ctrl handler timestamp: 0x%08x, buttons: 0x%08x\n", pDst->timeStamp, new_buttons);
    }

    pDst->buttons = new_buttons;
    pDst->DPadSenseA = 0;
    pDst->DPadSenseB = 0;
    pDst->GPadSenseA = 0;
    pDst->GPadSenseB = 0;
    pDst->AxisSenseA = 0;
    pDst->AxisSenseB = 0;
    pDst->TiltA = 0;
    pDst->TiltB = 0;
    pDst->aX = SCE_CTRL_ANALOG_PAD_CENTER_VALUE;
    pDst->aY = SCE_CTRL_ANALOG_PAD_CENTER_VALUE;
    pDst->rX = rightX;
    pDst->rY = rightY;
    pDst->rsrv[0] = -128;
    pDst->rsrv[1] = -128;

    // Success
    return 0;
}

// The main thread.
// * Sets up callbacks and timers
// * Sleeps and processes callbacks
// * Cleans up when awoken.
static
int main_thread(SceSize args, void *argp)
{
    SceUID ctrl_input_handler_res = -1;
    int result;

    //
    // Setup
    //

    DEBUG_PRINT("Setting controller input handler for " xstr(CONTROLLER_PORT) "\n");

    // sceCtrl_driver_6C86AF22() enables passing through controller state from a specific external controller port buffer
    // into the emulation state slot with the same index as the port.
    // This is the same emulation state set by sceCtrlSetButtonEmulation() and sceCtrlSetAnalogEmulation().
    //
    // It is effectively a thin setter function for g_ctrl.unk768.
    // For how this enables passthrough, see: https://github.com/uofw/uofw/blob/7ca6ba13966a38667fa7c5c30a428ccd248186cf/src/kd/ctrl/ctrl.c#L1782-L1803
    //
    // This allows the state to be read with the basic controller read functions that return SceCtrlData, such as
    // sceCtrlReadBufferPositive, because these functions sample the emulation state.
    //
    // The argument is a bit field with bits set corresponding to the ports.
    // 0x01 enables SCE_CTRL_PORT_DS3 passthrough
    // 0x02 enables SCE_CTRL_PORT_UNKNOWN_2
    // 0x00 disables passthrough such that it can only be read by the extended/extra functions that return SceCtrlData2,
    // such as sceCtrlReadBufferPositive2(), that takes the specific port number as an argument 
    sceCtrl_driver_6C86AF22(1 << (CONTROLLER_PORT - 1));

    // Setup sceCtrl_driver_E467BEC8() external controller port input handler.
    // This set the input data source for a controller port, similar to how the DS3 controller
    // is wired up internally to padsvc (Bluetooth -> DS3) on PSP Go.
    //
    // The copyInputData function is type SceCtrlInputDataTransferHandler and is called on every polling loop.
    SceCtrlInputDataTransferHandler controller_data_transfer_handler = {
        // GUESS: unk1 is the handler structure size. This is common in many SCE handler structures.
        .unk1 = sizeof(SceCtrlInputDataTransferHandler),
        .copyInputData = ctrl_input_data_handler_func
    };

    // sceCtrl_driver_E467BEC8(u8 externalPort, SceCtrlInputDataTransferHandler *transferHandler, void *inputSource)
    // The inputSource ptr is passed through into the handler function as the first argument, so it can be
    // used as an input buffer for controller inputs.
    ctrl_input_handler_res = sceCtrl_driver_E467BEC8(CONTROLLER_PORT, &controller_data_transfer_handler, &g_button_state);

    if(ctrl_input_handler_res != SCE_ERROR_OK) {
        DEBUG_PRINT("Failed to set controller input handler: ret 0x%08x\n", ctrl_input_handler_res);
    }

    DEBUG_PRINT("Setting controller polling mode to enable joystick\n");
    sceCtrlSetSamplingMode(SCE_CTRL_INPUT_DIGITAL_ANALOG);

    //
    // Sleep and process callbacks until we get woken up
    //
    DEBUG_PRINT("Now processing callbacks\n");
    sceKernelSleepThreadCB();

    //
    // Cleanup
    //
    if(ctrl_input_handler_res == SCE_ERROR_OK) {
        DEBUG_PRINT("Unsetting controller input handler for " xstr(CONTROLLER_PORT) "\n");

        result = sceCtrl_driver_E467BEC8(CONTROLLER_PORT, NULL, NULL);
        if(result < 0) {
            DEBUG_PRINT("Failed to unset controller input handler: ret 0x%08x\n", result);
        }
    }

    return 0;
}

static
int start_main_thread(void)
{
    int result;
    SceUID thid;

    // name, entry, initPriority, stackSize, PspThreadAttributes, SceKernelThreadOptParam
    thid = sceKernelCreateThread(MODULE_NAME "MainThread", main_thread, 0x11, 0x800, 0, 0);
    if (thid >= 0) {
        DEBUG_PRINT("Starting main thread\n");
        result = sceKernelStartThread(thid, 0, 0);
        if(result < 0) {
            DEBUG_PRINT("Failed to start main thread: ret 0x%08x\n", result);
        }

        g_mainThreadId = thid;
    }
    else {
        result = thid;
        DEBUG_PRINT("Failed to create main thread: ret 0x%08x\n", result);
    }

    return result;
}

static
int stop_main_thread(void)
{
    int result = 0;
    SceUID thid = g_mainThreadId;

    if(thid >= 0) {
        // Unblock sceKernelSleepThreadCB() and have thread begin cleanup
        result = sceKernelWakeupThread(thid);
        if(result < 0) {
            DEBUG_PRINT("Failed to wakeup main thread: ret 0x%08x\n", result);
        }

        // Wait for the main thread to clean up and exit
        DEBUG_PRINT("Waiting for main thread exit ...\n");
        result = sceKernelWaitThreadEnd(thid, NULL);
        if(result < 0) {
            // Thread did not stop, force terminate and delete it
            DEBUG_PRINT("Failed to wait for main thread exit: ret 0x%08x\n", result);
            DEBUG_PRINT("Terminating and deleting main thread\n", result);
            result = sceKernelTerminateDeleteThread(thid);
            if(result >= 0) {
                g_mainThreadId = -1;
            }
            else {
                DEBUG_PRINT("Failed to terminate delete callback thread: ret 0x%08x\n", result);
            }
        }
        else {
            DEBUG_PRINT("Deleting main thread\n");
            // Thead stopped cleanly, delete it
            result = sceKernelDeleteThread(thid);
            if(result >= 0) {
                DEBUG_PRINT("Main thread cleanup complete.\n");
                g_mainThreadId = -1;
            }
            else {
                DEBUG_PRINT("Failed to delete main thread: ret 0x%08x\n", result);
            }
        }
    }

    return result;
}

// Called during module init
int module_start(SceSize args, void *argp)
{
    int result;

    #ifdef DEBUG
    pspDebugScreenInit();
    #endif

    DEBUG_PRINT(MODULE_NAME " v" xstr(MAJOR_VER) "." xstr(MINOR_VER) " Module Start\n");

    result = start_main_thread();
    if(result < 0) {
        return MODULE_ERROR;
    }

    DEBUG_PRINT("Started.\n");

    return MODULE_OK;
}

// Called during module deinit
int module_stop(SceSize args, void *argp)
{
    int result;

    DEBUG_PRINT("Stopping ...\n");

    result = stop_main_thread();
    if(result < 0) {
        return MODULE_ERROR;
    }

    DEBUG_PRINT(MODULE_NAME " v" xstr(MAJOR_VER) "." xstr(MINOR_VER) " Module Stop\n");

    return MODULE_OK;
}
