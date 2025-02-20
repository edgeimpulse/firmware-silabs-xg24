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

#include "ei_camera_arducam.h"
#include "ei_device_xg24.h"
#include "jpegdec/jpegdec.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "firmware-sdk/at_base64_lib.h"
#include <cmath>


ei_device_snapshot_resolutions_t EiCameraArduCam::resolutions[] = {
        { .width = 160, .height = 120 },
        { .width = 176, .height = 144 }
    };

EiCameraArduCam::EiCameraArduCam()
{
    static ArduCAM camera_object;

    this->cam = &camera_object;

    stream_active = false;

    camera_present = this->cam->detect_camera();

    this->cam->resetFirmware();
    this->cam->set_format(JPEG);
    this->cam->InitCAM();
    //TODO: add API to change that resolution?
    this->cam->OV2640_set_JPEG_size(OV2640_160x120);
}

bool EiCameraArduCam::init(uint16_t width, uint16_t height)
{
    // try to set required resolution, returned is possible
    ei_device_snapshot_resolutions_t sensor_res = this->search_resolution(width, height);

    if(set_resolution(sensor_res) == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    return true;
}

bool EiCameraArduCam::set_resolution(const ei_device_snapshot_resolutions_t res)
{
    bool ret = false;

    if(res.width == 160 && res.height == 120) {
        cam->OV2640_set_JPEG_size(OV2640_160x120);
        ret = true;
    }
    else if(res.width == 176 && res.height == 144) {
        cam->OV2640_set_JPEG_size(OV2640_176x144);
        ret = true;
    }
    else if(res.width == 320 && res.height == 240) {
        cam->OV2640_set_JPEG_size(OV2640_320x240);
        ret = true;
    }

    if(ret == true) {
        width = res.width;
        height = res.height;
    }
    else {
        width = 0;
        height = 0;
    }

    return ret;
}

ei_device_snapshot_resolutions_t EiCameraArduCam::get_min_resolution(void)
{
    return resolutions[0];
}

EiCamera* EiCamera::get_camera(void)
{
    static EiCameraArduCam cam;

    return &cam;
}

bool EiCameraArduCam::is_camera_present(void)
{
    return camera_present;
}

bool EiCameraArduCam::ei_camera_capture_jpeg(uint8_t **image, uint32_t *image_size, uint16_t width, uint16_t height)
{
    //TODO: change resolution if needed
    // Remember to free image buffer!
    *image_size = this->cam->capture_frame(image);
    if(*image == nullptr || *image_size == 0) {
        ei_printf("Error getting snapshot!\n");
        return false;
    }

    return true;
}

bool EiCameraArduCam::ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size)
{
    uint8_t *jpeg_image = nullptr;
    uint32_t jpeg_image_size;
    bool result;
    
    //TODO: change resolution if needed
    jpeg_image_size = this->cam->capture_frame(&jpeg_image);
    if(jpeg_image == nullptr || jpeg_image_size == 0) {
        ei_printf("Error getting snapshot!\n");
        return false;
    }

    //TODO: calculate resize dimension????
    result = jpeg_decode_static(jpeg_image, jpeg_image_size, image, image_size);
    ei_free(jpeg_image);

    return result;
}

void EiCameraArduCam::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num)
{
    *res = &EiCameraArduCam::resolutions[0];
    *res_num = sizeof(EiCameraArduCam::resolutions) / sizeof(ei_device_snapshot_resolutions_t);
}

bool EiCameraArduCam::start_stream(uint32_t width, uint32_t height)
{
    // try to set required resolution, returned is what has been set
    ei_device_snapshot_resolutions_t sensor_res = this->search_resolution(width, height);

    if(set_resolution(sensor_res) == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    // store required output res
    this->output_width = width;
    this->output_height = height;

    // check if we have to do resize/crop
    this->stream_do_resize = this->width != width || this->height != height;

    // get bigger image resolution (snapshot or output) to allocate big enough buffer
    //TODO: get color depth (here 3 bytes) from camera props
    this->stream_buffer_size = std::max(this->width * this->height, this->output_width * this->output_height) * 3;
    this->stream_buffer = (uint8_t*)ei_malloc(stream_buffer_size);
    if(this->stream_buffer == nullptr) {
        ei_printf("ERR: Failed to allocate stream buffer!\n");
        return false;
    }

    this->stream_active = true;
    return true;
}

bool EiCameraArduCam::run_stream(void)
{
    if(stream_active == false) {
        return false;
    }

    ei_camera_capture_rgb888_packed_big_endian(this->stream_buffer, this->stream_buffer_size);

    if (this->stream_do_resize) {
        // interpolate in place
        ei::image::processing::crop_and_interpolate_rgb888(
            this->stream_buffer,
            this->width,
            this->height,
            this->stream_buffer,
            this->output_width,
            this->output_height);
    }

    //TODO: use camera color_depth size instead of fixed 3
    base64_encode((char*)this->stream_buffer,
        this->output_height * this->output_width * 3,
        ei_putchar);
    ei_printf("\r\n");

    return true;
}

bool EiCameraArduCam::is_stream_active(void)
{
    return stream_active;
}

bool EiCameraArduCam::stop_stream(void)
{
    auto dev = EiDeviceXG24::get_device();

    ei_printf("OK\r\n");
    // // we can call it even if the baudrate wasn't changed
    dev->set_default_data_output_baudrate();
    ei_sleep(100);
    ei_printf("Snapshot streaming stopped by user\n");
    ei_printf("OK\n");
    ei_free(this->stream_buffer);

    stream_active = false;

    return true;
}
