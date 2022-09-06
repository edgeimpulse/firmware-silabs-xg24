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

#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include "ei_device_xg24.h"
#include "em_system.h"
#include "flash_memory.h"
#include "ei_device_xg24.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_iostream_eusart_vcom_config.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
extern "C" {
#include "em_eusart.h"
};
//TODO: move out of device and register through EiDevice API
#include "sensors/ei_microphone.h"
#include "sensors/ei_inertial_sensor.h"
#include "sensors/ei_camera_arducam.h"

static const sl_led_t *led_red = &sl_led_led0;
static const sl_led_t *led_green = &sl_led_led1;
static const sl_led_t *led_blue = &sl_led_led2;

using namespace std;

static void on_sample_timer(sl_sleeptimer_timer_handle_t *handle, void *data)
{
    EiDeviceXG24* dev = static_cast<EiDeviceXG24*>(data);
    dev->sample_read_callback();
}

static void on_led_timer(sl_sleeptimer_timer_handle_t *handle, void *data)
{
    EiDeviceXG24* dev = static_cast<EiDeviceXG24*>(data);
    static uint8_t animation = 0;

    switch(dev->get_state())
    {
        case eiStateErasingFlash:
            dev->toggle_led(led_red);
            break;
        case eiStateSampling:
            dev->toggle_led(led_blue);
            break;
        case eiStateUploading:
            dev->toggle_led(led_green);
            break;
        case eiStateFinished:
            if(animation == 0) {
                animation = 10;
            }
            break;
        default:
            break;
    }

    if(animation == 0) {
        return;
    }

    switch(animation) {
        case 10:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 0);
            break;
        case 9:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 1);
            break;
        case 8:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 1);
            dev->set_led(led_blue, 0);
            break;
        case 7:
            dev->set_led(led_red, 1);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 0);
            break;
        case 6:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 1);
            break;
        case 5:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 1);
            dev->set_led(led_blue, 0);
            break;
        case 4:
            dev->set_led(led_red, 1);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 0);
            break;
        case 3:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 0);
            dev->set_led(led_blue, 1);
            break;
        case 2:
            dev->set_led(led_red, 0);
            dev->set_led(led_green, 1);
            dev->set_led(led_blue, 0);
            break;
        case 1:
            dev->set_state(eiStateIdle);
            break;
    }
    animation--;
}

EiDeviceXG24::EiDeviceXG24(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();

    load_config();

    device_type = "SILABS_XG24";

    cam = static_cast<EiCameraArduCam*>(EiCameraArduCam::get_camera());
    camera_present = cam->is_camera_present();
    if(camera_present == true) {
        led_red = nullptr;
    }

    standalone_sensor_list[0].name = "Microphone";
    standalone_sensor_list[0].frequencies[0] = 16000.0f;
    standalone_sensor_list[0].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t));
}

EiDeviceXG24::~EiDeviceXG24()
{

}

string EiDeviceXG24::get_mac_address(void)
{
    return mac_address;
}

bool EiDeviceXG24::is_camera_present(void)
{
    return camera_present;
}

void EiDeviceXG24::init_device_id(void)
{
    uint64_t id;
    char temp[18];

    id = SYSTEM_GetUnique();

    snprintf(temp, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
            (uint8_t)(id>>56),
            (uint8_t)(id>>48),
            (uint8_t)(id>>40),
            (uint8_t)(id>>16),
            (uint8_t)(id>>8),
            (uint8_t)(id));

    device_id = string(temp);
    mac_address = string(temp);
}

/**
 * @brief get_device is a static method of EiDeviceInfo class
 * It is used to implement singleton paradigm, so we are returning
 * here pointer always to the same object (dev)
 * 
 * @return EiDeviceInfo* 
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceXG24 dev(&memory);

    return &dev;
}

void EiDeviceXG24::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

bool EiDeviceXG24::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    sl_status_t ret;

    this->sample_read_callback = sample_read_cb;
    ret = sl_sleeptimer_start_periodic_timer_ms(&this->sample_timer,
                                          sample_interval_ms,
                                          on_sample_timer,
                                          this,
                                          0,
                                          SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

    return ret == SL_STATUS_OK;
}

bool EiDeviceXG24::stop_sample_thread(void)
{
    sl_status_t ret;

    ret = sl_sleeptimer_stop_timer(&this->sample_timer);

    return ret == SL_STATUS_OK;
}

void EiDeviceXG24::set_state(EiState state)
{
    this->state = state;

    set_led(led_red, 0);
    set_led(led_green, 0);
    set_led(led_blue, 0);

    switch(state) {
        case eiStateErasingFlash:
        case eiStateSampling:
        case eiStateUploading:
        case eiStateFinished:
            sl_sleeptimer_start_periodic_timer_ms(&this->led_timer,
                                                250,
                                                on_led_timer,
                                                this,
                                                0,
                                                SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
            break;
        case eiStateIdle:
        default:
            sl_sleeptimer_stop_timer(&this->led_timer);
            set_led(led_red, 0);
            set_led(led_green, 0);
            set_led(led_blue, 0);
    }
}

void EiDeviceXG24::set_led(const sl_led_t *led_handle, uint8_t state)
{
    if(led_handle == nullptr) {
        return;
    }

    if(state == 0) {
        sl_led_turn_off(led_handle);
    }
    else {
        sl_led_turn_on(led_handle);
    }
}

void EiDeviceXG24::toggle_led(const sl_led_t *led_handle)
{
    if(led_handle == nullptr) {
        return;
    }

    sl_led_toggle(led_handle);
}

EiState EiDeviceXG24::get_state(void)
{
    return this->state;
}

bool EiDeviceXG24::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;

    return true;
}

EiSnapshotProperties EiDeviceXG24::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res;
    uint8_t res_num = 0;

    //TODO: move the getting of snapshot to camera device
    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = 0,
        .resolutions = res
    };

    if(this->cam->is_camera_present() == true) {
        this->cam->get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }

    return props;
}

uint32_t EiDeviceXG24::get_data_output_baudrate(void)
{
    return 115200;
}
