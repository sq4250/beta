/**
 * hal_encoder.c — 双后轮编码器 (正交解码)
 */
#include "hal_encoder.h"
#include "config.h"
#include "zf_common_headfile.h"

// 硬件引脚 (根据实际接线修改)
#define ENC_L_CH  TC_CH09_ENCODER
#define ENC_L_CH1 TC_CH09_ENCODER_CH1_P05_0
#define ENC_L_CH2 TC_CH09_ENCODER_CH2_P05_1
#define ENC_R_CH  TC_CH07_ENCODER
#define ENC_R_CH1 TC_CH07_ENCODER_CH1_P02_0
#define ENC_R_CH2 TC_CH07_ENCODER_CH2_P02_1

void hal_encoder_init(void) {
    encoder_quad_init(ENC_L_CH, ENC_L_CH1, ENC_L_CH2);
    encoder_quad_init(ENC_R_CH, ENC_R_CH1, ENC_R_CH2);
}

void hal_encoder_get(encoder_t *enc) {
    enc->left  =  (f32)encoder_get_count(ENC_L_CH) * ENCODER_SCALE;
    enc->right = -(f32)encoder_get_count(ENC_R_CH) * ENCODER_SCALE;
    encoder_clear_count(ENC_L_CH);
    encoder_clear_count(ENC_R_CH);
}
