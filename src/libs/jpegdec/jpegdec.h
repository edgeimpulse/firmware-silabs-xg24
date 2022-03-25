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
