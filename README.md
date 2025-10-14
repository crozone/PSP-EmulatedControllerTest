# Emulated Controller Test

PSP CFW PRX Plugin - External controller port emulation demo

## Details

This is a Playstation Portable CFW plugin that demonstrates emulating an external controller input through software, using the same kernel interfaces that the PSP Go uses to support DS3 controller input.

The demo reads analog input from the joystick and translates it into d-pad input, as if it were coming from an externally connected DS3 controller.
If you enable this plugin in the XMB, you can finally navigate the XMB using the analog stick.

## sceCtrl_driver functions

The function [`sceCtrl_driver_E467BEC8()`](https://github.com/uofw/uofw/blob/7ca6ba13966a38667fa7c5c30a428ccd248186cf/src/kd/ctrl/ctrl.c#L902-L924) is used to register a controller input handler that provides data to the DS3 external controller port buffer. This uses the same mechanism that the PSP Go's padsvc module uses to send real DS3 input data to the controller driver.

The [`sceCtrl_driver_6C86AF22()`](https://github.com/uofw/uofw/blob/7ca6ba13966a38667fa7c5c30a428ccd248186cf/src/kd/ctrl/ctrl.c#L1180-L1184) function is used to enable reading of DS3 controller state through the standard read buffer methods (eg `sceCtrlPeekBufferPositive()`), so most applications will transparently receive the emulated data.

A vtimer runs at 100hz and samples the analog input using `sceCtrlPeekBufferPositive()`. It translates the analog input into button values and writes the bitfield into a global variable. When the controller input handler runs, it checks the global variable for any asserted buttons, and writes them into the external controller output buffer.

## Installation

* You will need a custom firmware installed on your PSP. See the [ARK-4 project](github.com/PSP-Archive/ARK-4) for details on how to install it.
* Copy emu_ctrl_test.prx into /SEPLUGINS/ on the root of your Memory Stick
* Edit `SEPLUGINS/PLUGINS.TXT`
  * To run the plugin in VSH, add the line
    ```
    vsh, ms0:/SEPLUGINS/emu_ctrl_test.prx, on
    ```
  * See [ARK-4 Plugins](https://github.com/PSP-Archive/ARK-4/wiki/Plugins) for more details. 
* Restart the PSP.

## Building

* Follow the PSPDEV toolchain [installation steps](https://pspdev.github.io/installation.html)

### For release

```bash
mkdir -p -- build/release
cd build/release
psp-cmake -DCMAKE_BUILD_TYPE=Release ../..
make
```

### For debug (with debug logs printed to the PSP display)

```bash
mkdir -p -- build/debug
cd build/debug
psp-cmake -DCMAKE_BUILD_TYPE=Debug ../..
make
```