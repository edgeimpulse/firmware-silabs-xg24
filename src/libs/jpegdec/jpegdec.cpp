#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "picojpeg.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#define jpg_min(a, b) (((a) < (b)) ? (a) : (b))

// global buffers used by pjpeg_need_bytes_callback
static uint8_t *jpeg_image;
static uint32_t jpeg_image_size;
static uint32_t jpeg_image_offset;

static uint8_t pjpeg_need_bytes_callback(
    uint8_t *pBuf,
    uint8_t buf_size,
    uint8_t *pBytes_actually_read,
    void *pCallback_data)
{
    uint8_t n;
    n = jpg_min(jpeg_image_size - jpeg_image_offset, buf_size);
    for (int i = 0; i < n; i++) {
        pBuf[i] = *(jpeg_image++);
    }
    *pBytes_actually_read = (uint8_t)(n);
    jpeg_image_offset += n;

    return 0;
}

uint32_t
jpeg_decode(uint8_t *input_jpeg_image, uint32_t input_jpeg_image_size, uint8_t **output_image, uint32_t output_image_len = 0)
{
    pjpeg_image_info_t image_info;
    uint8_t status;
    uint32_t output_image_size;
    int mcu_x = 0;
    int mcu_y = 0;

    // store pinters globally, picojpeg callback uses them
    jpeg_image = input_jpeg_image;
    jpeg_image_size = input_jpeg_image_size;
    jpeg_image_offset = 0;

    status = pjpeg_decode_init(&image_info, pjpeg_need_bytes_callback, NULL, 0);
    if (status) {
        ei_printf("ERR: pjpeg_decode_init failed = 0x%x\n", status);
        return 0;
    }

    unsigned int row_pitch = image_info.m_width * image_info.m_comps;
    if(output_image_len == 0) {
        *output_image = (uint8_t *)ei_malloc(row_pitch * image_info.m_height);
        if (output_image == nullptr) {
            ei_printf("ERR: Failed to allocate output image buffer!\n");
            return 0;
        }
    }
    output_image_size = row_pitch * image_info.m_height;
    if(output_image_len != 0 && output_image_len < output_image_size) {
        ei_printf("ERR: provided buffer size = %lu, required = %lu\n", output_image_len, output_image_size);
        return 0;
    }

    while (pjpeg_decode_mcu() == 0) {
        if (mcu_y >= image_info.m_MCUSPerCol) {
            ei_printf("ERR: Too many MCUs in JPEG file!\n");
            if(output_image_len == 0) {
                ei_free(*output_image);
            }
            return 0;
        }
        uint8_t *pDst_row = *output_image + (mcu_y * image_info.m_MCUHeight) * row_pitch +
            (mcu_x * image_info.m_MCUWidth * image_info.m_comps);
        for (int y = 0; y < image_info.m_MCUHeight; y += 8) {
            const int by_limit =
                jpg_min(8, image_info.m_height - (mcu_y * image_info.m_MCUHeight + y));
            for (int x = 0; x < image_info.m_MCUWidth; x += 8) {
                uint8_t *pDst_block = pDst_row + x * image_info.m_comps;
                // Compute source byte offset of the block in the decoder's MCU buffer.
                unsigned int src_ofs = (x * 8U) + (y * 16U);
                const uint8_t *pSrcR = image_info.m_pMCUBufR + src_ofs;
                const uint8_t *pSrcG = image_info.m_pMCUBufG + src_ofs;
                const uint8_t *pSrcB = image_info.m_pMCUBufB + src_ofs;
                const int bx_limit =
                    jpg_min(8, image_info.m_width - (mcu_x * image_info.m_MCUWidth + x));
                if (image_info.m_scanType == PJPG_GRAYSCALE) {
                    int bx, by;
                    for (by = 0; by < by_limit; by++) {
                        uint8_t *pDst = pDst_block;
                        for (bx = 0; bx < bx_limit; bx++) {
                            *pDst++ = *pSrcR++;
                        }
                        pSrcR += (8 - bx_limit);
                        pDst_block += row_pitch;
                    }
                }
                else {
                    int bx, by;
                    for (by = 0; by < by_limit; by++) {
                        uint8_t *pDst = pDst_block;
                        for (bx = 0; bx < bx_limit; bx++) {
                            pDst[0] = *pSrcR++;
                            pDst[1] = *pSrcG++;
                            pDst[2] = *pSrcB++;
                            pDst += 3;
                        }
                        pSrcR += (8 - bx_limit);
                        pSrcG += (8 - bx_limit);
                        pSrcB += (8 - bx_limit);
                        pDst_block += row_pitch;
                    }
                }
            }
            pDst_row += (row_pitch * 8);
        }
        mcu_x++;
        if (mcu_x == image_info.m_MCUSPerRow) {
            mcu_x = 0;
            mcu_y++;
        }
    }
    return output_image_size;
}

uint32_t
jpeg_decode_dynamic(uint8_t *input_jpeg_image, uint32_t input_jpeg_image_size, uint8_t **output_image)
{
    return jpeg_decode(input_jpeg_image, input_jpeg_image_size, output_image, 0);
}


bool
jpeg_decode_static(uint8_t *input_jpeg_image, uint32_t input_jpeg_image_size, uint8_t *output_image, uint32_t output_image_size)
{
    uint32_t ret;

    ret = jpeg_decode(input_jpeg_image, input_jpeg_image_size, &output_image, output_image_size);

    return ret == output_image_size;
}
