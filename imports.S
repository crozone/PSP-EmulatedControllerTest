.set noreorder

#include "pspimport.s"

IMPORT_START "sceCtrl_driver",0x00010000
IMPORT_FUNC	"sceCtrl_driver",0x2BA616AF,sceCtrlPeekBufferPositive
IMPORT_FUNC	"sceCtrl_driver",0x5130DAE3,sceCtrlSetButtonEmulation
IMPORT_FUNC	"sceCtrl_driver",0xDB76878D,sceCtrlSetAnalogEmulation
IMPORT_FUNC	"sceCtrl_driver",0xF6E94EA3,sceCtrlSetSamplingMode
IMPORT_FUNC	"sceCtrl_driver",0xE467BEC8,sceCtrl_driver_E467BEC8
IMPORT_FUNC	"sceCtrl_driver",0x6C86AF22,sceCtrl_driver_6C86AF22