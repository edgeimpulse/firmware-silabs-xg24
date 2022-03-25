/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
