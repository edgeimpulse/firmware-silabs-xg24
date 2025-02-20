/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ei_environment_sensor.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "sl_si70xx.h"
#include "sl_bmp3xx.h"
#include "sl_i2cspm_instances.h"
#include <cstdint>
#include <cstdlib>

static float sensor_value[ENV_AXIS_SAMPLED];

bool ei_environment_sensor_init(void)
{
    sl_status_t ret = SL_STATUS_OK;

    ret = sl_si70xx_init(sl_i2cspm_sensor, SI7021_ADDR);
    if(ret != SL_STATUS_OK) {
        ei_printf("ERR: failed to init RHT sensor (0x%04lx)\n", ret);
        return false;
    }

    ret = sl_bmp3xx_init(sl_i2cspm_sensor);
    if(ret != SL_STATUS_OK) {
        ei_printf("ERR: failed to init pressure sensor (0x%04lx)\n", ret);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
        ei_printf("ERR: failed to register Environmental sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_environment_sensor_read_data(int n_samples)
{
    uint32_t bufRH = 0;
    int32_t bufT = 0;
    sl_status_t ret = SL_STATUS_OK;

    ret = sl_si70xx_measure_rh_and_temp(sl_i2cspm_sensor, SI7021_ADDR, &bufRH, &bufT);
    if(ret == SL_STATUS_OK) {
        // temperature is reported in miliCelsius
        sensor_value[0] = bufT / 1000.0f;
        // humidity is reported in percent multiplied by 1000
        sensor_value[1] = bufRH / 1000.0f;
    }
    else {
        sensor_value[0] = 0.0f;
        sensor_value[1] = 0.0f;
    }

    ret = sl_bmp3xx_measure_pressure(sl_i2cspm_sensor, &sensor_value[2]);
    if(ret != SL_STATUS_OK) {
        sensor_value[2] = 0.0f;
    }
    else {
        sensor_value[2] /= 1000.0f;
    }

    return sensor_value;
}