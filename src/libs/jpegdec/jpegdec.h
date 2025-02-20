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

#ifndef JPEGDEC_H
#define JPEGDEC_H
#include <cstdint>

/**
 * @brief Decode JPEG image into raw RGB or Grayscale array.
 * It is wrapper for picojpeg https://github.com/richgel999/picojpeg
 * The function is using ei_malloc to allocate buffer for decoded image.
 * The resulting output image buffer will have size width * height * (3 for RGB, 1 for Greyscale)
 * It is user responsible to free the output_image buffer.
 * 
 * @param input_jpeg_image
 * @param input_jpeg_image_size
 * @param output_image just a pointer to pointer, don't pass any buffer as function is doing allocation
 * @return uint32_t size of allocated buffer for output_image, 0 if something went wrong
 */
uint32_t
jpeg_decode_dynamic(uint8_t *input_jpeg_image, uint32_t input_jpeg_image_size, uint8_t **output_image);

/**
 * @brief Decode JPEG image into raw RGB or Grayscale array.
 * It is wrapper for picojpeg https://github.com/richgel999/picojpeg
 * The function is using statically allocated buffer by the user. It's user responsibility to 
 * provide big enough buffer. If provided buffer is too small, function return false and print error.
 * 
 * @param input_jpeg_image
 * @param input_jpeg_image_size
 * @param output_image buffer for decoded image
 * @param output_image_size size of the output_image buffer
 * @return true if image decoded succesfully
 * @return false if error occured
 */
bool
jpeg_decode_static(uint8_t *input_jpeg_image, uint32_t input_jpeg_image_size, uint8_t *output_image, uint32_t output_image_size);

#endif /* JPEGDEC_H */
