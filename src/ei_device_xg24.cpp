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
// #include "sensors/ei_inertial_sensor.h"
#include "sensors/ei_camera_arducam.h"

static const sl_led_t *led_red = &sl_led_led0;
static const sl_led_t *led_green = &sl_led_led1;
static const sl_led_t *led_blue = &sl_led_led2;

using namespace std;

static uint32_t calc_best_timer(uint32_t time1, uint32_t time2);

static void on_sample_timer(sl_sleeptimer_timer_handle_t *handle, void *data)
{
    EiDeviceXG24* dev = static_cast<EiDeviceXG24*>(data);

#if MULTI_FREQ_ENABLED == 1
    if (dev->get_fusioning() == 1){
        dev->sample_read_callback();
    }
    else {
        uint8_t flag = 0;
        uint8_t i = 0;
        dev->actual_timer += dev->get_sample_interval();

        for (i = 0; i < dev->get_fusioning(); i++){
            if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {
                flag |= (1<<i);
            }
        }
        if (dev->sample_multi_read_callback != nullptr)
        {
            dev->sample_multi_read_callback(flag);

        }
    }
#else
    dev->sample_read_callback();
#endif

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

#if MULTI_FREQ_ENABLED == 1
    this->fusioning = 1;
#endif
    ret = sl_sleeptimer_start_periodic_timer_ms(&this->sample_timer,
                                          sample_interval_ms,
                                          on_sample_timer,
                                          this,
                                          0,
                                          SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

    return ret == SL_STATUS_OK;
}

#if MULTI_FREQ_ENABLED == 1
/**
 *
 * @param sample_multi_read_cb
 * @param multi_sample_interval_ms
 * @param num_fusioned
 * @return
 */
bool EiDeviceXG24::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
  sl_status_t ret;

  uint8_t i;
  uint8_t flag = 0;

  this->sample_multi_read_callback = sample_multi_read_cb;
  this->fusioning = num_fusioned;
  this->multi_sample_interval.clear();

  this->sample_multi_read_callback = sample_multi_read_cb;

  for (i = 0; i < num_fusioned; i++){
    this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
  }
  /* to improve, we consider just a 2 sensors case for now */
  this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

      /* force first reading */
  for (i = 0; i < this->fusioning; i++){
    flag |= (1<<i);
  }
  this->sample_multi_read_callback(flag);

  this->actual_timer = 0;

  ret = sl_sleeptimer_start_periodic_timer_ms(&this->sample_timer,
                                              this->sample_interval,
                                        on_sample_timer,
                                        this,
                                        0,
                                        SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

  return ret == SL_STATUS_OK;
}
#endif

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

/**
 *
 * @param freq1
 * @param freq2
 * @return
 */
static uint32_t calc_best_timer(uint32_t time1, uint32_t time2)
{
    uint32_t temp = 1;

    if (time2 > time1){
        temp = time2;
        time2 = time1;
        time1 = temp;
    }

    while(time2 != 0){
        temp = time1 % time2;
        time1 = time2;
        time2 = temp;
    }


    return time1;
}
