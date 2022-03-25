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

#include "ei_camera_arducam.h"
#include "jpegdec/jpegdec.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "ei_device_xg24.h"
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
    *image_size = this->cam->capture_frame(image);
    if(*image == nullptr || *image_size == 0) {
        ei_printf("Error getting snapshot!\n");
        return false;
    }

    return true;
}

bool EiCameraArduCam::ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size, uint16_t width, uint16_t height)
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
    ei_device_snapshot_resolutions_t snapshot_res = this->try_set_resolution(width, height);
    
    if(snapshot_res.width == 0) {
        ei_printf("ERR: Failed to set camera resolution!\n");
        return false;
    }

    // store required ouptut res
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

    ei_camera_capture_rgb888_packed_big_endian(this->stream_buffer, this->stream_buffer_size, this->width, this->height);

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
