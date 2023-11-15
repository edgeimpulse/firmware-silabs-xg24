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
