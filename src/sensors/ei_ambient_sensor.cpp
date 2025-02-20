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