/*
 * SC031GS driver.
 * 
 * Copyright 2022-2023 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "sccb.h"
#include "xclk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "adv7180.h"
#include "adv7180_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "adv7180";
#endif

//#define SC031GS_MAX_FRAME_WIDTH       (640)
//#define SC031GS_MAX_FRAME_HIGH        (480)

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = SCCB_Read(sensor->slv_addr, reg & 0xFFFF);
    if(ret > 0){
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0;
    ret = SCCB_Read16(sensor->slv_addr, reg & 0xFFFF);
    if(ret < 0){
        return ret;
    }
    value = (ret & ~mask) | (value & mask);
    ret = SCCB_Write16(sensor->slv_addr, reg & 0xFFFF, value);
    return ret;
}

/*
static int set_reg_bits(sensor_t *sensor, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    int ret = 0;
    ret = SCCB_Read16(sensor->slv_addr, reg);
    if(ret < 0){
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (ret & ~mask) | ((value << offset) & mask);
    ret = SCCB_Write16(sensor->slv_addr, reg, value);
    return ret;
}
*/

/*
static int write_regs(uint8_t slv_addr, const struct sc031gs_regval *regs)
{
    int i = 0, ret = 0;
    while (!ret && regs[i].addr != REG_NULL) {
        if (regs[i].addr == REG_DELAY) {
            vTaskDelay(regs[i].val / portTICK_PERIOD_MS);
        } else {
            ret = SCCB_Write16(slv_addr, regs[i].addr, regs[i].val);
        }
        i++;
    }
    return ret;
}

#define WRITE_REGS_OR_RETURN(regs) ret = write_regs(slv_addr, regs); if(ret){return ret;}
#define WRITE_REG_OR_RETURN(reg, val) ret = set_reg(sensor, reg, 0xFF, val); if(ret){return ret;}
#define SET_REG_BITS_OR_RETURN(reg, offset, length, val) ret = set_reg_bits(sensor, reg, offset, length, val); if(ret){return ret;}
*/

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
/*
    SET_REG_BITS_OR_RETURN(0x4501, 3, 1, enable & 0x01); // enable test pattern mode
    SET_REG_BITS_OR_RETURN(0x3902, 6, 1, 1); // enable auto BLC, disable auto BLC if set to 0
    SET_REG_BITS_OR_RETURN(0x3e06, 0, 2, 3); // digital gain: 00->1x, 01->2x, 03->4x.
*/
    return ret;
}

int set_bpc(sensor_t *sensor, int enable) // // For sc03ags sensor, This API used to control BLC
{
    int ret = 0;
/*
    SET_REG_BITS_OR_RETURN(0x3900, 0, 1, enable & 0x01);
    SET_REG_BITS_OR_RETURN(0x3902, 6, 1, enable & 0x01);
*/
    return ret;
}

static int reset(sensor_t *sensor)
{
    //int ret = write_regs(sensor->slv_addr, sc031gs_default_init_regs);
    int ret = 0;

    if (ret) {
        ESP_LOGE(TAG, "reset fail");
    }
    // printf("reg 0x3d04=%02x\r\n", get_reg(sensor, 0x3d04, 0xff));
    // set_colorbar(sensor, 1);
    return ret;
}

// static int set_output_window(sensor_t *sensor, int offset_x, int offset_y, int w, int h)
// {
//     int ret = 0;
//     //sc:H_start={0x3212[1:0],0x3213},H_length={0x3208[1:0],0x3209},
//     // printf("%d, %d, %d, %d\r\n", ((offset_x>>8) & 0x03), offset_x & 0xff, ((w>>8) & 0x03), w & 0xff);

// /*
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_START_X_H_REG, 0x0); // For now, we use x_start is 0x04
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_START_X_L_REG, 0x04);
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_WIDTH_H_REG, ((w>>8) & 0x03));
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_WIDTH_L_REG, w & 0xff);

//     //sc:V_start={0x3210[1:0],0x3211},V_length={0x320a[1:0],0x320b},
//     // printf("%d, %d, %d, %d\r\n", ((offset_y>>8) & 0x03), offset_y & 0xff, ((h>>8) & 0x03), h & 0xff);
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_START_Y_H_REG, 0x0); // For now, we use y_start is 0x08
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_START_Y_L_REG, 0x08);
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_HIGH_H_REG, ((h>>8) & 0x03));
//     WRITE_REG_OR_RETURN(SC031GS_OUTPUT_WINDOW_HIGH_L_REG, h & 0xff);
// */

//     vTaskDelay(10 / portTICK_PERIOD_MS);

//     return ret;
// }

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    // if(w > 720 || h > 576) {
    //     goto err; 
    // }

    printf("Width of %d is %d \n", 22, resolution[22].width);
    printf("Width of %d is %d \n", 8, resolution[8].width);

    printf("Passing %d \n", framesize);

    printf("Setting frame size W: %d, H: %d \n", w, h);

/*
    if(w != 200 || h != 200) {
        ESP_LOGE(TAG, "Only support 200*200 for now, contact us if you want to use other resolutions");
        goto err; 
    }
*/

/*
    uint16_t offset_x = (640-w) /2 + 4;   
    uint16_t offset_y = (480-h) /2 + 4;
    
    if(set_output_window(sensor, offset_x, offset_y, w, h)) {
        goto err; 
    }
*/
    
    sensor->status.framesize = framesize;
    return 0;
// err:
//     ESP_LOGE(TAG, "frame size err");
//     return -1;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret=0;
    sensor->pixformat = pixformat;

    switch (pixformat) {
    case PIXFORMAT_YUV422:
    break;
    default:
        ESP_LOGE(TAG, "Only support GRAYSCALE(Y8)");
        return -1;
    }

    return ret;
}

static int init_status(sensor_t *sensor)
{
    return 0;
}

static int set_dummy(sensor_t *sensor, int val){ return -1; }


int adv7180_detect(int slv_addr, sensor_id_t *id)
{
    if (ADV7180_SCCB_ADDR == slv_addr) {
        /*
        uint8_t MIDL = SCCB_Read16(slv_addr, SC031GS_PID_HIGH_REG);
        uint8_t MIDH = SCCB_Read16(slv_addr, SC031GS_PID_LOW_REG);
        uint16_t PID = MIDH << 8 | MIDL;
        */

#if 0
        SCCB_Write(slv_addr, 0x00, 0x00);
        SCCB_Write(slv_addr, 0x04, 0x57);
        SCCB_Write(slv_addr, 0x17, 0x41);
        SCCB_Write(slv_addr, 0x31, 0x02);
        SCCB_Write(slv_addr, 0x3D, 0xA2);
        SCCB_Write(slv_addr, 0x3E, 0x6A);
        SCCB_Write(slv_addr, 0x3F, 0xA0);
        SCCB_Write(slv_addr, 0x0E, 0x80);
        SCCB_Write(slv_addr, 0x55, 0x81);
        SCCB_Write(slv_addr, 0x0E, 0x00);
#else
        SCCB_Write(slv_addr, inputControl, 					0x04); 	/*select ain 0, autodetect composite standart*/
        SCCB_Write(slv_addr, autoDetectEnable, 				0xff); 	/*enable detecting of all composite standarts*/
        SCCB_Write(slv_addr, defaultValueY, 					0xf2); 	/*enable free run and output blue screen*/
        SCCB_Write(slv_addr, defaultValueC, 					0x70); 	/*enable free run an blue screen output*/
        SCCB_Write(slv_addr, vs_fieldCtl_1, 					0x12);  /*default*/
        SCCB_Write(slv_addr, vs_fieldCtl_2, 					0x41);  /*default*/
        SCCB_Write(slv_addr, vs_fieldCtl_3, 					0x84);  /*default*/
        SCCB_Write(slv_addr, hs_positionCtl_1, 				0x21);  /*default*/
        SCCB_Write(slv_addr, hs_positionCtl_2, 				0x00);  /*default*/
        SCCB_Write(slv_addr, hs_positionCtl_3, 				0x00);  /*default*/
        SCCB_Write(slv_addr, polarity, 						0x01);  /*default*/
		SCCB_Write(slv_addr, vs_fieldPinctl, 					0x01);  /*default*/
        SCCB_Write(slv_addr, lockCount, 					    0x36);  /*default*/
        SCCB_Write(slv_addr, miscGainCtl, 					0xa0);  /*default*/
        SCCB_Write(slv_addr, cvbsTrim, 					    0x0d);  /*default*/
        SCCB_Write(slv_addr, sdOffsetCb,   					0x00);  /*default*/
        SCCB_Write(slv_addr, sdOffsetCr   , 					0x00);  /*default*/
        SCCB_Write(slv_addr, ntscCombCtl, 					0x80);  /*default*/
        SCCB_Write(slv_addr, palCombCtl, 						0xc0);  /*default*/
        SCCB_Write(slv_addr, adcCtl, 							0x10);  /*default*/
        SCCB_Write(slv_addr, manualWindowCtl, 				0x02);  /*default*/
		SCCB_Write(slv_addr, pixelDelayCtl, 				    0x80);  /*default*/
#endif

        printf("Working on ADV7180 probe \n");

        // FAKE IT..
        uint16_t PID = 0x1C;

        //uint8_t ident = SCCB_Read(slv_addr, 0x11);
        //printf("IDENT is: %x \n", ident);

        if (ADV7180_PID == PID) {
            id->PID = PID;
            return PID;
        } else {
            ESP_LOGI(TAG, "Mismatch PID=0x%x", PID);
        }
    }
    return 0;
}

int adv7180_init(sensor_t *sensor)
{
    // Set function pointers
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    
    sensor->set_colorbar = set_colorbar;
    sensor->set_hmirror = set_dummy;
    sensor->set_vflip = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_aec_value = set_dummy;
    sensor->set_special_effect = set_dummy;
    
    //not supported
    sensor->set_awb_gain = set_dummy;
    sensor->set_contrast = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_saturation= set_dummy;
    sensor->set_denoise = set_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;
    
    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_xclk = set_dummy;
    
    ESP_LOGD(TAG, "sc031gs Attached");

    return 0;
}