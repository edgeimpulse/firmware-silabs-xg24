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

#include "ei_ambient_sensor.h"
extern "C" {
#include "sl_veml6035.h"
};
#include "sl_i2cspm_instances.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <cstdint>
#include <cstdlib>

static float als_value;

bool ei_ambient_sensor_init(void)
{
    sl_status_t ret = SL_STATUS_OK;

    ret = sl_veml6035_init(sl_i2cspm_sensor, false);
    if(ret != SL_STATUS_OK) {
        ei_printf("ERR: failed to init ambient sensor (0x%04lx)\n", ret);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(ambient_sensor) == false) {
        ei_printf("ERR: failed to register Ambient Light sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_ambient_sensor_read_data(int n_samples)
{
    sl_status_t ret = SL_STATUS_OK;

    ret = sl_veml6035_get_als_lux(sl_i2cspm_sensor, &als_value);
    if(ret != SL_STATUS_OK) {
        als_value = 0.0f;
    }

    return &als_value;
}