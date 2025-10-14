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

// Maximum sleep attempts that can be blocked in a row, before sleep is allowed as a failsafe.
#define MAX_CONSECUTIVE_SLEEP_ATTEMPTS 10

#define MODULE_NAME "EmulatedControllerTest"
#define MAJOR_VER 1
#define MINOR_VER 0

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
static int timer_callback_handler(int unknown, int vtid, void *p_common);
static SceUInt vtimer_handler(SceUID vtid, SceInt64 schedule, SceInt64 actual, void *p_common);

static int main_thread(SceSize args, void *argp);
static int start_main_thread(void);
static int stop_main_thread(void);
int module_start(SceSize args, void *argp);
int module_stop(SceSize args, void *argp);

//
// Globals
//
static SceUInt g_button_state = 0;
static SceUID g_timerCallbackId = -1;
static SceUID g_mainThreadId = -1;

// Timer Callback handler
// int (*SceKernelCallbackFunction)(int arg1, int arg2, void *arg)
static
int timer_callback_handler(int unknown, int vtid, void *p_common)
{
    SceCtrlData pad_state;
    if(sceCtrlPeekBufferPositive(&pad_state, 1) >= 0) {
        int pad_x = pad_state.aX;
        int pad_y = pad_state.aY;

        SceUInt new_buttons = 0;

        if(pad_x > (SCE_CTRL_ANALOG_PAD_CENTER_VALUE + 64)) {
            new_buttons |= SCE_CTRL_RIGHT;
        }

        if(pad_x < (SCE_CTRL_ANALOG_PAD_CENTER_VALUE - 64)) {
            new_buttons |= SCE_CTRL_LEFT;
        }

        if(pad_y > (SCE_CTRL_ANALOG_PAD_CENTER_VALUE + 64)) {
            new_buttons |= SCE_CTRL_DOWN;
        }

        if(pad_y < (SCE_CTRL_ANALOG_PAD_CENTER_VALUE - 64)) {
            new_buttons |= SCE_CTRL_UP;
        }

        g_button_state = new_buttons;
    }
    else {
       DEBUG_PRINT("Failed to read button state!\n");
    }

    return 0;
}

// typedef unsigned int (*SceKernelVTimerHandlerWide)(SceUID vtid, unsigned long long *schedule, unsigned long long *actual, void *p_common);
static
SceUInt vtimer_handler(SceUID vtid, SceInt64 schedule, SceInt64 actual, void *p_common) {
    // This handler is run in user mode, so don't call any kernel mode functions from here directly.
    // Instead, notify a callback, which is run on the kernel callback handling thread we have set up.

    // We don't know what *p_common actually is yet (it's definitely not the pointer we provided to the hander registration function).
    // So, for now, pass the callback id to run into this hander using a global variable.
    int cbid = g_timerCallbackId;
    //int* cbid_ptr = (int*)p_common;
    //int cbid = cbid_ptr != NULL ? *cbid_ptr : -1;
    if(cbid >= 0) {
        int callback_ret = sceKernelNotifyCallback(cbid, vtid);

        if(callback_ret < 0) {
            DEBUG_PRINT("Callback ret %d\n", callback_ret);
            return 0;
        }
    }

    // Return time to next invocation
    return TIMER_PERIOD;
}

//
// Controller callback function based on padsvc decomp
//
static
s32 ctrl_input_data_handler_func(void *pSrc, SceCtrlData2 *pDst)
{
    SceUInt* p_new_buttons = (SceUInt*)pSrc;
    SceUInt new_buttons = p_new_buttons != NULL ? *p_new_buttons : 0;

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
    SceUID timer_cbid = -1;
    SceUID vtid = -1;
    SceUID ctrl_input_handler_id = -1;
    int slot;
    int result;

    //
    // Setup
    //

    DEBUG_PRINT("Registering controller input handler\n");

    // sceCtrl_driver_6C86AF22() enables passing through controller state from a specific external controller port buffer
    // into the global controller buffer.
    //
    // This allows it to be read with the basic controller read methods that return SceCtrlData, such as
    // sceCtrlReadBufferPositive.
    //
    // The argument is a bit field with bits set corresponding to the port.
    // 0x01 enables SCE_CTRL_PORT_DS3 passthrough
    // 0x02 enables SCE_CTRL_PORT_UNKNOWN_2
    // 0x00 disables passthrough such that it can only be read by the extended/extra methods that return SceCtrlData2,
    // such as sceCtrlReadBufferPositive2(), that takes the specific port number as an argument 
    sceCtrl_driver_6C86AF22(SCE_CTRL_PORT_DS3);

    // Setup sceCtrl_driver_E467BEC8() external controller port input handler.
    // This effectively sets up an additional source for controller input polling, similar to how the DS3 controller
    // is wired up internally to padsvc (Bluetooth -> DS3) on PSP Go.
    //
    // The copyInputData function is type SceCtrlInputDataTransferHandler and is called on every polling loop.
    SceCtrlInputDataTransferHandler controller_data_transfer_handler = {
        .unk1 = sizeof(SceCtrlInputDataTransferHandler),
        .copyInputData = ctrl_input_data_handler_func
    };

    // sceCtrl_driver_E467BEC8(u8 externalPort, SceCtrlInputDataTransferHandler *transferHandler, void *inputSource)
    // The inputSource ptr is passed through into the handler function as the first argument.
    ctrl_input_handler_id = sceCtrl_driver_E467BEC8(SCE_CTRL_PORT_DS3, &controller_data_transfer_handler, &g_button_state);

    if(ctrl_input_handler_id != SCE_ERROR_OK) {
        DEBUG_PRINT("Failed to register controller input handler: ret 0x%08x\n", ctrl_input_handler_id);
    }

    DEBUG_PRINT("Setting controller polling mode to enable joystick\n");
    sceCtrlSetSamplingMode(SCE_CTRL_INPUT_DIGITAL_ANALOG);

    DEBUG_PRINT("Creating timer callback\n");
    timer_cbid = sceKernelCreateCallback(MODULE_NAME " Timer Callback", timer_callback_handler, NULL);
    if(timer_cbid >= 0) {
        // Set a global variable and reference the global variable from the handler.
        // Hacky, but works for now.
        g_timerCallbackId = timer_cbid;

        DEBUG_PRINT("Registering timer\n");
        vtid = sceKernelCreateVTimer(MODULE_NAME " Timer", NULL);

        if(vtid >= 0) {
            int set_vtimer_handle_ret;

            // Stop the timer
            sceKernelStopVTimer(vtid);

            // Reset the timer time to zero
            sceKernelSetVTimerTimeWide(vtid, sceKernelUSec2SysClockWide(0));

            // Register callback on created vtimer.
            // Use Wide version so we get to use SceInt64 for time instead of *SceKernelSysClock
            set_vtimer_handle_ret = sceKernelSetVTimerHandlerWide(
                vtid,
                sceKernelUSec2SysClockWide(10 * ONE_MSEC),
                vtimer_handler,
                &g_timerCallbackId // TODO: Doesn't get passed to handler directly, the ptr changes.
                );

            if(set_vtimer_handle_ret < 0) {
                DEBUG_PRINT("Failed to register LED timer handler: ret 0x%08x\n", set_vtimer_handle_ret);
            }

            DEBUG_PRINT("Start timer\n");
            sceKernelStartVTimer(vtid);
                    
        }
        else {
            DEBUG_PRINT("Failed to create LED timer: ret 0x%08x\n", vtid);
        }
    }
    else {
        DEBUG_PRINT("Failed to create timer callback: ret 0x%08x\n", timer_cbid);
    }

    //
    // Sleep and process callbacks until we get woken up
    //
    DEBUG_PRINT("Now processing callbacks\n");
    sceKernelSleepThreadCB();

    //
    // Cleanup
    //
    if(vtid >= 0) {
        DEBUG_PRINT("Deleting timer\n");
        int delete_vtimer_ret = sceKernelDeleteVTimer(vtid);
        if(delete_vtimer_ret < 0) {
            DEBUG_PRINT("Failed to delete timer handler: ret 0x%08x\n", delete_vtimer_ret);
        }
    }

    if(timer_cbid >= 0) {
        DEBUG_PRINT("Deleting timer callback\n");
        result = sceKernelDeleteCallback(timer_cbid);
        if(result < 0) {
            DEBUG_PRINT("Failed to delete timer callback: ret 0x%08x\n", result);
        }
    }

    if(ctrl_input_handler_id >= 0) {
        DEBUG_PRINT("Deregistering controller input handler\n");
        result = sceCtrl_driver_E467BEC8(SCE_CTRL_PORT_DS3, NULL, NULL);
        if(result < 0) {
            DEBUG_PRINT("Failed to deregister controller input handler: ret 0x%08x\n", result);
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
