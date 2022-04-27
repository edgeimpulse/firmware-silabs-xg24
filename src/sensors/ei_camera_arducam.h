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

#ifndef EI_CAMERA_ARDUCAM_H
#define EI_CAMERA_ARDUCAM_H
#include "firmware-sdk/ei_camera_interface.h"
//TODO: only for ei_device_snapshot_resolutions_t
#include "firmware-sdk/ei_device_info_lib.h"
#include "ArduCAM_OV2640/ArduCAM.h"

class EiCameraArduCam : public EiCamera {
private:
    ArduCAM *cam;
    static ei_device_snapshot_resolutions_t resolutions[];
    bool stream_active;
    uint32_t width;
    uint32_t height;
    uint32_t output_width;
    uint32_t output_height;
    bool stream_do_resize;
    uint8_t *stream_buffer;
    uint32_t stream_buffer_size;
    bool camera_present;

public:
    EiCameraArduCam();
    bool init(uint16_t width, uint16_t height);
    bool ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size, uint16_t width, uint16_t height);
    bool ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size);
    bool set_resolution(const ei_device_snapshot_resolutions_t res);
    ei_device_snapshot_resolutions_t get_min_resolution(void);
    bool is_camera_present(void);
    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num);

    bool start_stream(uint32_t width, uint32_t height);
    bool run_stream(void);
    bool is_stream_active(void);
    bool stop_stream(void);
};

#endif /* EI_CAMERA_ARDUCAM_H */