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

#include <stdint.h>
#include <stdlib.h>
#include "ei_inertial_sensor.h"
#include "ei_sampler.h"
#include "ei_device_xg24.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "sl_imu.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

static float imu_data[INERTIAL_AXIS_SAMPLED];

bool ei_inertial_init(void)
{
    sl_status_t ret = 0;

    ret = sl_imu_init();
    if(ret != SL_STATUS_OK) {
        ei_printf("ERR: IMU init failed (0x%04lx)!\n", ret);
        return false;
    }

    sl_imu_configure(200);
    sl_imu_calibrate_gyro();

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_inertial_read_data(int n_samples)
{
    if(sl_imu_is_data_ready()) {
        sl_imu_update();
        sl_imu_get_acceleration_raw_data(&imu_data[0]);
        imu_data[0] *= CONVERT_G_TO_MS2;
        imu_data[1] *= CONVERT_G_TO_MS2;
        imu_data[2] *= CONVERT_G_TO_MS2;

        if (n_samples > 3) {
            sl_imu_get_gyro_raw_data(&imu_data[3]);
        }
        else {
            imu_data[3] = 0.0f;
            imu_data[4] = 0.0f;
            imu_data[5] = 0.0f;
        }
    }
    else {
        ei_printf("ERR: no IMU data!\n");
        imu_data[0] = 0.0f;
        imu_data[1] = 0.0f;
        imu_data[2] = 0.0f;
        imu_data[3] = 0.0f;
        imu_data[4] = 0.0f;
        imu_data[5] = 0.0f;
    }

    return imu_data;
}
