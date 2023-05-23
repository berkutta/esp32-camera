#ifndef _ADV7180_SETTINGS_H_
#define _ADV7180_SETTINGS_H_

#include <stdint.h>

#define inputControl 		0x00
#define videoSelection    	0x01
#define outputControl     	0x03
#define extOutputControl  	0x04
#define autoDetectEnable    0x07
#define contrast            0x08
#define brightness          0x0a
#define hue                 0x0b
#define defaultValueY       0x0c
#define defaultValueC       0x0d
#define adiControl          0x0e
#define powerManagement     0x0f
#define analogClampControl  0x14
#define digitalClampCtl_1   0x15
#define shapingFilterCtl_1  0x17
#define shapingFilterCtl_2  0x18
#define combFilterCtl       0x19
#define adiCtl_2            0x1d
#define pixelDelayCtl       0x27
#define miscGainCtl         0x2B
#define agcModeCtl          0x2c
#define chromaGainCtl_1_WO  0x2d
#define chromaGainCtl_2_WO  0x2e
#define lumaGainCtl_1_WO    0x2f
#define lumaGainCtl_2_WO    0x30
#define vs_fieldCtl_1       0x31
#define vs_fieldCtl_2       0x32
#define vs_fieldCtl_3       0x33
#define hs_positionCtl_1    0x34
#define hs_positionCtl_2    0x35
#define hs_positionCtl_3    0x36
#define polarity            0x37
#define ntscCombCtl         0x38
#define palCombCtl          0x39
#define adcCtl              0x3a
#define manualWindowCtl     0x3d
#define resampleCtl         0x41
#define gemstarCtl_1        0x48
#define gemstarCtl_2        0x49
#define gemstarCtl_3        0x4a
#define gemstarCtl_4        0x4b
#define gemstarCtl_5        0x4c
#define ctiDnrCtl_1         0x4d
#define ctiDnrCtl_2         0x4e
#define ctiDnrCtl_3         0x50
#define lockCount           0x51
#define cvbsTrim            0x52
#define vs_fieldPinctl      0x58
#define GPO                 0x59
#define freeRunLineLen      0x8f
#define crcEnable           0xb2
#define adcSwitch_1         0xc3
#define adcSwitch_2         0xc4
#define letterBoxCtl_1      0xdc
#define letterBoxCtl_2      0xdd
#define sdOffsetCb          0xe1
#define sdOffsetCr          0xe2
#define sdSaturationCb      0xe3
#define sdSaturationCr      0xe4
#define ntscVBitBegin       0xe5
#define ntscVBitEnd         0xe6
#define ntscVBitToggle      0xe7
#define palVBitBegin        0xe8
#define palVBitEnd          0xe9
#define palFBitToggle       0xea
#define vBlankCtl_1         0xeb
#define vBlankCtl_2         0xec
#define afeCtl_1            0xf3
#define driveStrenght       0xf4
#define IFCompCtl           0xf8
#define vsModeCtl           0xf9
#define peakingCtl          0xfb
#define coringTreshhold     0xfc

#endif