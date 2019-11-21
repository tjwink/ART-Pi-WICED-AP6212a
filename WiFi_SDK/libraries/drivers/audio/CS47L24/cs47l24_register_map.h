/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define CS47L24_SOFTWARE_RESET                   0x00
#define CS47L24_DEVICE_REVISION                  0x01
#define CS47L24_SOFTWARE_REVISION                0x02
#define CS47L24_CTRL_IF_SPI_CFG_1                0x08
#define CS47L24_WRITE_SEQUENCER_CTRL_0           0x16
#define CS47L24_WRITE_SEQUENCER_CTRL_1           0x17
#define CS47L24_WRITE_SEQUENCER_CTRL_2           0x18
#define CS47L24_TONE_GENERATOR_1                 0x20
#define CS47L24_TONE_GENERATOR_2                 0x21
#define CS47L24_TONE_GENERATOR_3                 0x22
#define CS47L24_TONE_GENERATOR_4                 0x23
#define CS47L24_TONE_GENERATOR_5                 0x24
#define CS47L24_PWM_DRIVE_1                      0x30
#define CS47L24_PWM_DRIVE_2                      0x31
#define CS47L24_PWM_DRIVE_3                      0x32
#define CS47L24_SEQUENCE_CONTROL                 0x41
#define CS47L24_SAMPLE_RATE_SEQUENCE_SELECT_1    0x61
#define CS47L24_SAMPLE_RATE_SEQUENCE_SELECT_2    0x62
#define CS47L24_SAMPLE_RATE_SEQUENCE_SELECT_3    0x63
#define CS47L24_SAMPLE_RATE_SEQUENCE_SELECT_4    0x64
#define CS47L24_GPIO_TRIGGERS_SEQUENCE_SELECT_1  0x68
#define CS47L24_GPIO_TRIGGERS_SEQUENCE_SELECT_2  0x69
#define CS47L24_GPIO_TRIGGERS_SEQUENCE_SELECT_3  0x6A
#define CS47L24_GPIO_TRIGGERS_SEQUENCE_SELECT_4  0x6B
#define CS47L24_TRIGGER_SEQUENCE_SELECT_32       0x6E
#define CS47L24_TRIGGER_SEQUENCE_SELECT_33       0x6F
#define CS47L24_COMFORT_NOISE_GENERATOR          0x70
#define CS47L24_HAPTICS_CONTROL_1                0x90
#define CS47L24_HAPTICS_CONTROL_2                0x91
#define CS47L24_HAPTICS_PHASE_1_INTENSITY        0x92
#define CS47L24_HAPTICS_PHASE_1_DURATION         0x93
#define CS47L24_HAPTICS_PHASE_2_INTENSITY        0x94
#define CS47L24_HAPTICS_PHASE_2_DURATION         0x95
#define CS47L24_HAPTICS_PHASE_3_INTENSITY        0x96
#define CS47L24_HAPTICS_PHASE_3_DURATION         0x97
#define CS47L24_HAPTICS_STATUS                   0x98
#define CS47L24_CLOCK_32K_1                      0x100
#define CS47L24_SYSTEM_CLOCK_1                   0x101
#define CS47L24_SAMPLE_RATE_1                    0x102
#define CS47L24_SAMPLE_RATE_2                    0x103
#define CS47L24_SAMPLE_RATE_3                    0x104
#define CS47L24_SAMPLE_RATE_1_STATUS             0x10A
#define CS47L24_SAMPLE_RATE_2_STATUS             0x10B
#define CS47L24_SAMPLE_RATE_3_STATUS             0x10C
#define CS47L24_ASYNC_CLOCK_1                    0x112
#define CS47L24_ASYNC_SAMPLE_RATE_1              0x113
#define CS47L24_ASYNC_SAMPLE_RATE_2              0x114
#define CS47L24_ASYNC_SAMPLE_RATE_1_STATUS       0x11B
#define CS47L24_ASYNC_SAMPLE_RATE_2_STATUS       0x11C
#define CS47L24_OUTPUT_SYSTEM_CLOCK              0x149
#define CS47L24_OUTPUT_ASYNC_CLOCK               0x14A
#define CS47L24_RATE_ESTIMATOR_1                 0x152
#define CS47L24_RATE_ESTIMATOR_2                 0x153
#define CS47L24_RATE_ESTIMATOR_3                 0x154
#define CS47L24_RATE_ESTIMATOR_4                 0x155
#define CS47L24_RATE_ESTIMATOR_5                 0x156
#define CS47L24_FLL1_CONTROL_1                   0x171
#define CS47L24_FLL1_CONTROL_2                   0x172
#define CS47L24_FLL1_CONTROL_3                   0x173
#define CS47L24_FLL1_CONTROL_4                   0x174
#define CS47L24_FLL1_CONTROL_5                   0x175
#define CS47L24_FLL1_CONTROL_6                   0x176
#define CS47L24_FLL1_LOOP_FILTER_TEST_1          0x177
#define CS47L24_FLL1_NCO_TEST_0                  0x178
#define CS47L24_FLL1_CONTROL_7                   0x179
#define CS47L24_FLL1_SYNCHRONISER_1              0x181
#define CS47L24_FLL1_SYNCHRONISER_2              0x182
#define CS47L24_FLL1_SYNCHRONISER_3              0x183
#define CS47L24_FLL1_SYNCHRONISER_4              0x184
#define CS47L24_FLL1_SYNCHRONISER_5              0x185
#define CS47L24_FLL1_SYNCHRONISER_6              0x186
#define CS47L24_FLL1_SYNCHRONISER_7              0x187
#define CS47L24_FLL1_SPREAD_SPECTRUM             0x189
#define CS47L24_FLL1_GPIO_CLOCK                  0x18A
#define CS47L24_FLL2_CONTROL_1                   0x191
#define CS47L24_FLL2_CONTROL_2                   0x192
#define CS47L24_FLL2_CONTROL_3                   0x193
#define CS47L24_FLL2_CONTROL_4                   0x194
#define CS47L24_FLL2_CONTROL_5                   0x195
#define CS47L24_FLL2_CONTROL_6                   0x196
#define CS47L24_FLL2_LOOP_FILTER_TEST_1          0x197
#define CS47L24_FLL2_NCO_TEST_0                  0x198
#define CS47L24_FLL2_CONTROL_7                   0x199
#define CS47L24_FLL2_SYNCHRONISER_1              0x1A1
#define CS47L24_FLL2_SYNCHRONISER_2              0x1A2
#define CS47L24_FLL2_SYNCHRONISER_3              0x1A3
#define CS47L24_FLL2_SYNCHRONISER_4              0x1A4
#define CS47L24_FLL2_SYNCHRONISER_5              0x1A5
#define CS47L24_FLL2_SYNCHRONISER_6              0x1A6
#define CS47L24_FLL2_SYNCHRONISER_7              0x1A7
#define CS47L24_FLL2_SPREAD_SPECTRUM             0x1A9
#define CS47L24_FLL2_GPIO_CLOCK                  0x1AA
#define CS47L24_MIC_BIAS_CTRL_1                  0x218
#define CS47L24_MIC_BIAS_CTRL_2                  0x219
#define CS47L24_INPUT_ENABLES                    0x300
#define CS47L24_INPUT_ENABLES_STATUS             0x301
#define CS47L24_INPUT_RATE                       0x308
#define CS47L24_INPUT_VOLUME_RAMP                0x309
#define CS47L24_HPF_CONTROL                      0x30C
#define CS47L24_IN1L_CONTROL                     0x310
#define CS47L24_ADC_DIGITAL_VOLUME_1L            0x311
#define CS47L24_DMIC1L_CONTROL                   0x312
#define CS47L24_IN1R_CONTROL                     0x314
#define CS47L24_ADC_DIGITAL_VOLUME_1R            0x315
#define CS47L24_DMIC1R_CONTROL                   0x316
#define CS47L24_IN2L_CONTROL                     0x318
#define CS47L24_ADC_DIGITAL_VOLUME_2L            0x319
#define CS47L24_DMIC2L_CONTROL                   0x31A
#define CS47L24_IN2R_CONTROL                     0x31C
#define CS47L24_ADC_DIGITAL_VOLUME_2R            0x31D
#define CS47L24_DMIC2R_CONTROL                   0x31E
#define CS47L24_OUTPUT_ENABLES_1                 0x400
#define CS47L24_OUTPUT_STATUS_1                  0x401
#define CS47L24_RAW_OUTPUT_STATUS_1              0x406
#define CS47L24_OUTPUT_RATE_1                    0x408
#define CS47L24_OUTPUT_VOLUME_RAMP               0x409
#define CS47L24_OUTPUT_PATH_CONFIG_1L            0x410
#define CS47L24_DAC_DIGITAL_VOLUME_1L            0x411
#define CS47L24_DAC_VOLUME_LIMIT_1L              0x412
#define CS47L24_NOISE_GATE_SELECT_1L             0x413
#define CS47L24_DAC_DIGITAL_VOLUME_1R            0x415
#define CS47L24_DAC_VOLUME_LIMIT_1R              0x416
#define CS47L24_NOISE_GATE_SELECT_1R             0x417
#define CS47L24_DAC_DIGITAL_VOLUME_4L            0x429
#define CS47L24_OUT_VOLUME_4L                    0x42A
#define CS47L24_NOISE_GATE_SELECT_4L             0x42B
#define CS47L24_DAC_AEC_CONTROL_1                0x450
#define CS47L24_NOISE_GATE_CONTROL               0x458
#define CS47L24_HP1_SHORT_CIRCUIT_CTRL           0x4A0
#define CS47L24_AIF1_BCLK_CTRL                   0x500
#define CS47L24_AIF1_TX_PIN_CTRL                 0x501
#define CS47L24_AIF1_RX_PIN_CTRL                 0x502
#define CS47L24_AIF1_RATE_CTRL                   0x503
#define CS47L24_AIF1_FORMAT                      0x504
#define CS47L24_AIF1_RX_BCLK_RATE                0x506
#define CS47L24_AIF1_FRAME_CTRL_1                0x507
#define CS47L24_AIF1_FRAME_CTRL_2                0x508
#define CS47L24_AIF1_FRAME_CTRL_3                0x509
#define CS47L24_AIF1_FRAME_CTRL_4                0x50A
#define CS47L24_AIF1_FRAME_CTRL_5                0x50B
#define CS47L24_AIF1_FRAME_CTRL_6                0x50C
#define CS47L24_AIF1_FRAME_CTRL_7                0x50D
#define CS47L24_AIF1_FRAME_CTRL_8                0x50E
#define CS47L24_AIF1_FRAME_CTRL_9                0x50F
#define CS47L24_AIF1_FRAME_CTRL_10               0x510
#define CS47L24_AIF1_FRAME_CTRL_11               0x511
#define CS47L24_AIF1_FRAME_CTRL_12               0x512
#define CS47L24_AIF1_FRAME_CTRL_13               0x513
#define CS47L24_AIF1_FRAME_CTRL_14               0x514
#define CS47L24_AIF1_FRAME_CTRL_15               0x515
#define CS47L24_AIF1_FRAME_CTRL_16               0x516
#define CS47L24_AIF1_FRAME_CTRL_17               0x517
#define CS47L24_AIF1_FRAME_CTRL_18               0x518
#define CS47L24_AIF1_TX_ENABLES                  0x519
#define CS47L24_AIF1_RX_ENABLES                  0x51A
#define CS47L24_AIF2_BCLK_CTRL                   0x540
#define CS47L24_AIF2_TX_PIN_CTRL                 0x541
#define CS47L24_AIF2_RX_PIN_CTRL                 0x542
#define CS47L24_AIF2_RATE_CTRL                   0x543
#define CS47L24_AIF2_FORMAT                      0x544
#define CS47L24_AIF2_RX_BCLK_RATE                0x546
#define CS47L24_AIF2_FRAME_CTRL_1                0x547
#define CS47L24_AIF2_FRAME_CTRL_2                0x548
#define CS47L24_AIF2_FRAME_CTRL_3                0x549
#define CS47L24_AIF2_FRAME_CTRL_4                0x54A
#define CS47L24_AIF2_FRAME_CTRL_5                0x54B
#define CS47L24_AIF2_FRAME_CTRL_6                0x54C
#define CS47L24_AIF2_FRAME_CTRL_7                0x54D
#define CS47L24_AIF2_FRAME_CTRL_8                0x54E
#define CS47L24_AIF2_FRAME_CTRL_9                0x54F
#define CS47L24_AIF2_FRAME_CTRL_10               0x550
#define CS47L24_AIF2_FRAME_CTRL_11               0x551
#define CS47L24_AIF2_FRAME_CTRL_12               0x552
#define CS47L24_AIF2_FRAME_CTRL_13               0x553
#define CS47L24_AIF2_FRAME_CTRL_14               0x554
#define CS47L24_AIF2_FRAME_CTRL_15               0x555
#define CS47L24_AIF2_FRAME_CTRL_16               0x556
#define CS47L24_AIF2_TX_ENABLES                  0x559
#define CS47L24_AIF2_RX_ENABLES                  0x55A
#define CS47L24_AIF3_BCLK_CTRL                   0x580
#define CS47L24_AIF3_TX_PIN_CTRL                 0x581
#define CS47L24_AIF3_RX_PIN_CTRL                 0x582
#define CS47L24_AIF3_RATE_CTRL                   0x583
#define CS47L24_AIF3_FORMAT                      0x584
#define CS47L24_AIF3_RX_BCLK_RATE                0x586
#define CS47L24_AIF3_FRAME_CTRL_1                0x587
#define CS47L24_AIF3_FRAME_CTRL_2                0x588
#define CS47L24_AIF3_FRAME_CTRL_3                0x589
#define CS47L24_AIF3_FRAME_CTRL_4                0x58A
#define CS47L24_AIF3_FRAME_CTRL_11               0x591
#define CS47L24_AIF3_FRAME_CTRL_12               0x592
#define CS47L24_AIF3_TX_ENABLES                  0x599
#define CS47L24_AIF3_RX_ENABLES                  0x59A
#define CS47L24_PWM1MIX_INPUT_1_SOURCE           0x640
#define CS47L24_PWM1MIX_INPUT_1_VOLUME           0x641
#define CS47L24_PWM1MIX_INPUT_2_SOURCE           0x642
#define CS47L24_PWM1MIX_INPUT_2_VOLUME           0x643
#define CS47L24_PWM1MIX_INPUT_3_SOURCE           0x644
#define CS47L24_PWM1MIX_INPUT_3_VOLUME           0x645
#define CS47L24_PWM1MIX_INPUT_4_SOURCE           0x646
#define CS47L24_PWM1MIX_INPUT_4_VOLUME           0x647
#define CS47L24_PWM2MIX_INPUT_1_SOURCE           0x648
#define CS47L24_PWM2MIX_INPUT_1_VOLUME           0x649
#define CS47L24_PWM2MIX_INPUT_2_SOURCE           0x64A
#define CS47L24_PWM2MIX_INPUT_2_VOLUME           0x64B
#define CS47L24_PWM2MIX_INPUT_3_SOURCE           0x64C
#define CS47L24_PWM2MIX_INPUT_3_VOLUME           0x64D
#define CS47L24_PWM2MIX_INPUT_4_SOURCE           0x64E
#define CS47L24_PWM2MIX_INPUT_4_VOLUME           0x64F
#define CS47L24_OUT1LMIX_INPUT_1_SOURCE          0x680
#define CS47L24_OUT1LMIX_INPUT_1_VOLUME          0x681
#define CS47L24_OUT1LMIX_INPUT_2_SOURCE          0x682
#define CS47L24_OUT1LMIX_INPUT_2_VOLUME          0x683
#define CS47L24_OUT1LMIX_INPUT_3_SOURCE          0x684
#define CS47L24_OUT1LMIX_INPUT_3_VOLUME          0x685
#define CS47L24_OUT1LMIX_INPUT_4_SOURCE          0x686
#define CS47L24_OUT1LMIX_INPUT_4_VOLUME          0x687
#define CS47L24_OUT1RMIX_INPUT_1_SOURCE          0x688
#define CS47L24_OUT1RMIX_INPUT_1_VOLUME          0x689
#define CS47L24_OUT1RMIX_INPUT_2_SOURCE          0x68A
#define CS47L24_OUT1RMIX_INPUT_2_VOLUME          0x68B
#define CS47L24_OUT1RMIX_INPUT_3_SOURCE          0x68C
#define CS47L24_OUT1RMIX_INPUT_3_VOLUME          0x68D
#define CS47L24_OUT1RMIX_INPUT_4_SOURCE          0x68E
#define CS47L24_OUT1RMIX_INPUT_4_VOLUME          0x68F
#define CS47L24_OUT4LMIX_INPUT_1_SOURCE          0x6B0
#define CS47L24_OUT4LMIX_INPUT_1_VOLUME          0x6B1
#define CS47L24_OUT4LMIX_INPUT_2_SOURCE          0x6B2
#define CS47L24_OUT4LMIX_INPUT_2_VOLUME          0x6B3
#define CS47L24_OUT4LMIX_INPUT_3_SOURCE          0x6B4
#define CS47L24_OUT4LMIX_INPUT_3_VOLUME          0x6B5
#define CS47L24_OUT4LMIX_INPUT_4_SOURCE          0x6B6
#define CS47L24_OUT4LMIX_INPUT_4_VOLUME          0x6B7
#define CS47L24_AIF1TX1MIX_INPUT_1_SOURCE        0x700
#define CS47L24_AIF1TX1MIX_INPUT_1_VOLUME        0x701
#define CS47L24_AIF1TX1MIX_INPUT_2_SOURCE        0x702
#define CS47L24_AIF1TX1MIX_INPUT_2_VOLUME        0x703
#define CS47L24_AIF1TX1MIX_INPUT_3_SOURCE        0x704
#define CS47L24_AIF1TX1MIX_INPUT_3_VOLUME        0x705
#define CS47L24_AIF1TX1MIX_INPUT_4_SOURCE        0x706
#define CS47L24_AIF1TX1MIX_INPUT_4_VOLUME        0x707
#define CS47L24_AIF1TX2MIX_INPUT_1_SOURCE        0x708
#define CS47L24_AIF1TX2MIX_INPUT_1_VOLUME        0x709
#define CS47L24_AIF1TX2MIX_INPUT_2_SOURCE        0x70A
#define CS47L24_AIF1TX2MIX_INPUT_2_VOLUME        0x70B
#define CS47L24_AIF1TX2MIX_INPUT_3_SOURCE        0x70C
#define CS47L24_AIF1TX2MIX_INPUT_3_VOLUME        0x70D
#define CS47L24_AIF1TX2MIX_INPUT_4_SOURCE        0x70E
#define CS47L24_AIF1TX2MIX_INPUT_4_VOLUME        0x70F
#define CS47L24_AIF1TX3MIX_INPUT_1_SOURCE        0x710
#define CS47L24_AIF1TX3MIX_INPUT_1_VOLUME        0x711
#define CS47L24_AIF1TX3MIX_INPUT_2_SOURCE        0x712
#define CS47L24_AIF1TX3MIX_INPUT_2_VOLUME        0x713
#define CS47L24_AIF1TX3MIX_INPUT_3_SOURCE        0x714
#define CS47L24_AIF1TX3MIX_INPUT_3_VOLUME        0x715
#define CS47L24_AIF1TX3MIX_INPUT_4_SOURCE        0x716
#define CS47L24_AIF1TX3MIX_INPUT_4_VOLUME        0x717
#define CS47L24_AIF1TX4MIX_INPUT_1_SOURCE        0x718
#define CS47L24_AIF1TX4MIX_INPUT_1_VOLUME        0x719
#define CS47L24_AIF1TX4MIX_INPUT_2_SOURCE        0x71A
#define CS47L24_AIF1TX4MIX_INPUT_2_VOLUME        0x71B
#define CS47L24_AIF1TX4MIX_INPUT_3_SOURCE        0x71C
#define CS47L24_AIF1TX4MIX_INPUT_3_VOLUME        0x71D
#define CS47L24_AIF1TX4MIX_INPUT_4_SOURCE        0x71E
#define CS47L24_AIF1TX4MIX_INPUT_4_VOLUME        0x71F
#define CS47L24_AIF1TX5MIX_INPUT_1_SOURCE        0x720
#define CS47L24_AIF1TX5MIX_INPUT_1_VOLUME        0x721
#define CS47L24_AIF1TX5MIX_INPUT_2_SOURCE        0x722
#define CS47L24_AIF1TX5MIX_INPUT_2_VOLUME        0x723
#define CS47L24_AIF1TX5MIX_INPUT_3_SOURCE        0x724
#define CS47L24_AIF1TX5MIX_INPUT_3_VOLUME        0x725
#define CS47L24_AIF1TX5MIX_INPUT_4_SOURCE        0x726
#define CS47L24_AIF1TX5MIX_INPUT_4_VOLUME        0x727
#define CS47L24_AIF1TX6MIX_INPUT_1_SOURCE        0x728
#define CS47L24_AIF1TX6MIX_INPUT_1_VOLUME        0x729
#define CS47L24_AIF1TX6MIX_INPUT_2_SOURCE        0x72A
#define CS47L24_AIF1TX6MIX_INPUT_2_VOLUME        0x72B
#define CS47L24_AIF1TX6MIX_INPUT_3_SOURCE        0x72C
#define CS47L24_AIF1TX6MIX_INPUT_3_VOLUME        0x72D
#define CS47L24_AIF1TX6MIX_INPUT_4_SOURCE        0x72E
#define CS47L24_AIF1TX6MIX_INPUT_4_VOLUME        0x72F
#define CS47L24_AIF1TX7MIX_INPUT_1_SOURCE        0x730
#define CS47L24_AIF1TX7MIX_INPUT_1_VOLUME        0x731
#define CS47L24_AIF1TX7MIX_INPUT_2_SOURCE        0x732
#define CS47L24_AIF1TX7MIX_INPUT_2_VOLUME        0x733
#define CS47L24_AIF1TX7MIX_INPUT_3_SOURCE        0x734
#define CS47L24_AIF1TX7MIX_INPUT_3_VOLUME        0x735
#define CS47L24_AIF1TX7MIX_INPUT_4_SOURCE        0x736
#define CS47L24_AIF1TX7MIX_INPUT_4_VOLUME        0x737
#define CS47L24_AIF1TX8MIX_INPUT_1_SOURCE        0x738
#define CS47L24_AIF1TX8MIX_INPUT_1_VOLUME        0x739
#define CS47L24_AIF1TX8MIX_INPUT_2_SOURCE        0x73A
#define CS47L24_AIF1TX8MIX_INPUT_2_VOLUME        0x73B
#define CS47L24_AIF1TX8MIX_INPUT_3_SOURCE        0x73C
#define CS47L24_AIF1TX8MIX_INPUT_3_VOLUME        0x73D
#define CS47L24_AIF1TX8MIX_INPUT_4_SOURCE        0x73E
#define CS47L24_AIF1TX8MIX_INPUT_4_VOLUME        0x73F
#define CS47L24_AIF2TX1MIX_INPUT_1_SOURCE        0x740
#define CS47L24_AIF2TX1MIX_INPUT_1_VOLUME        0x741
#define CS47L24_AIF2TX1MIX_INPUT_2_SOURCE        0x742
#define CS47L24_AIF2TX1MIX_INPUT_2_VOLUME        0x743
#define CS47L24_AIF2TX1MIX_INPUT_3_SOURCE        0x744
#define CS47L24_AIF2TX1MIX_INPUT_3_VOLUME        0x745
#define CS47L24_AIF2TX1MIX_INPUT_4_SOURCE        0x746
#define CS47L24_AIF2TX1MIX_INPUT_4_VOLUME        0x747
#define CS47L24_AIF2TX2MIX_INPUT_1_SOURCE        0x748
#define CS47L24_AIF2TX2MIX_INPUT_1_VOLUME        0x749
#define CS47L24_AIF2TX2MIX_INPUT_2_SOURCE        0x74A
#define CS47L24_AIF2TX2MIX_INPUT_2_VOLUME        0x74B
#define CS47L24_AIF2TX2MIX_INPUT_3_SOURCE        0x74C
#define CS47L24_AIF2TX2MIX_INPUT_3_VOLUME        0x74D
#define CS47L24_AIF2TX2MIX_INPUT_4_SOURCE        0x74E
#define CS47L24_AIF2TX2MIX_INPUT_4_VOLUME        0x74F
#define CS47L24_AIF2TX3MIX_INPUT_1_SOURCE        0x750
#define CS47L24_AIF2TX3MIX_INPUT_1_VOLUME        0x751
#define CS47L24_AIF2TX3MIX_INPUT_2_SOURCE        0x752
#define CS47L24_AIF2TX3MIX_INPUT_2_VOLUME        0x753
#define CS47L24_AIF2TX3MIX_INPUT_3_SOURCE        0x754
#define CS47L24_AIF2TX3MIX_INPUT_3_VOLUME        0x755
#define CS47L24_AIF2TX3MIX_INPUT_4_SOURCE        0x756
#define CS47L24_AIF2TX3MIX_INPUT_4_VOLUME        0x757
#define CS47L24_AIF2TX4MIX_INPUT_1_SOURCE        0x758
#define CS47L24_AIF2TX4MIX_INPUT_1_VOLUME        0x759
#define CS47L24_AIF2TX4MIX_INPUT_2_SOURCE        0x75A
#define CS47L24_AIF2TX4MIX_INPUT_2_VOLUME        0x75B
#define CS47L24_AIF2TX4MIX_INPUT_3_SOURCE        0x75C
#define CS47L24_AIF2TX4MIX_INPUT_3_VOLUME        0x75D
#define CS47L24_AIF2TX4MIX_INPUT_4_SOURCE        0x75E
#define CS47L24_AIF2TX4MIX_INPUT_4_VOLUME        0x75F
#define CS47L24_AIF2TX5MIX_INPUT_1_SOURCE        0x760
#define CS47L24_AIF2TX5MIX_INPUT_1_VOLUME        0x761
#define CS47L24_AIF2TX5MIX_INPUT_2_SOURCE        0x762
#define CS47L24_AIF2TX5MIX_INPUT_2_VOLUME        0x763
#define CS47L24_AIF2TX5MIX_INPUT_3_SOURCE        0x764
#define CS47L24_AIF2TX5MIX_INPUT_3_VOLUME        0x765
#define CS47L24_AIF2TX5MIX_INPUT_4_SOURCE        0x766
#define CS47L24_AIF2TX5MIX_INPUT_4_VOLUME        0x767
#define CS47L24_AIF2TX6MIX_INPUT_1_SOURCE        0x768
#define CS47L24_AIF2TX6MIX_INPUT_1_VOLUME        0x769
#define CS47L24_AIF2TX6MIX_INPUT_2_SOURCE        0x76A
#define CS47L24_AIF2TX6MIX_INPUT_2_VOLUME        0x76B
#define CS47L24_AIF2TX6MIX_INPUT_3_SOURCE        0x76C
#define CS47L24_AIF2TX6MIX_INPUT_3_VOLUME        0x76D
#define CS47L24_AIF2TX6MIX_INPUT_4_SOURCE        0x76E
#define CS47L24_AIF2TX6MIX_INPUT_4_VOLUME        0x76F
#define CS47L24_AIF3TX1MIX_INPUT_1_SOURCE        0x780
#define CS47L24_AIF3TX1MIX_INPUT_1_VOLUME        0x781
#define CS47L24_AIF3TX1MIX_INPUT_2_SOURCE        0x782
#define CS47L24_AIF3TX1MIX_INPUT_2_VOLUME        0x783
#define CS47L24_AIF3TX1MIX_INPUT_3_SOURCE        0x784
#define CS47L24_AIF3TX1MIX_INPUT_3_VOLUME        0x785
#define CS47L24_AIF3TX1MIX_INPUT_4_SOURCE        0x786
#define CS47L24_AIF3TX1MIX_INPUT_4_VOLUME        0x787
#define CS47L24_AIF3TX2MIX_INPUT_1_SOURCE        0x788
#define CS47L24_AIF3TX2MIX_INPUT_1_VOLUME        0x789
#define CS47L24_AIF3TX2MIX_INPUT_2_SOURCE        0x78A
#define CS47L24_AIF3TX2MIX_INPUT_2_VOLUME        0x78B
#define CS47L24_AIF3TX2MIX_INPUT_3_SOURCE        0x78C
#define CS47L24_AIF3TX2MIX_INPUT_3_VOLUME        0x78D
#define CS47L24_AIF3TX2MIX_INPUT_4_SOURCE        0x78E
#define CS47L24_AIF3TX2MIX_INPUT_4_VOLUME        0x78F
#define CS47L24_EQ1MIX_INPUT_1_SOURCE            0x880
#define CS47L24_EQ1MIX_INPUT_1_VOLUME            0x881
#define CS47L24_EQ1MIX_INPUT_2_SOURCE            0x882
#define CS47L24_EQ1MIX_INPUT_2_VOLUME            0x883
#define CS47L24_EQ1MIX_INPUT_3_SOURCE            0x884
#define CS47L24_EQ1MIX_INPUT_3_VOLUME            0x885
#define CS47L24_EQ1MIX_INPUT_4_SOURCE            0x886
#define CS47L24_EQ1MIX_INPUT_4_VOLUME            0x887
#define CS47L24_EQ2MIX_INPUT_1_SOURCE            0x888
#define CS47L24_EQ2MIX_INPUT_1_VOLUME            0x889
#define CS47L24_EQ2MIX_INPUT_2_SOURCE            0x88A
#define CS47L24_EQ2MIX_INPUT_2_VOLUME            0x88B
#define CS47L24_EQ2MIX_INPUT_3_SOURCE            0x88C
#define CS47L24_EQ2MIX_INPUT_3_VOLUME            0x88D
#define CS47L24_EQ2MIX_INPUT_4_SOURCE            0x88E
#define CS47L24_EQ2MIX_INPUT_4_VOLUME            0x88F
#define CS47L24_DRC1LMIX_INPUT_1_SOURCE          0x8C0
#define CS47L24_DRC1LMIX_INPUT_1_VOLUME          0x8C1
#define CS47L24_DRC1LMIX_INPUT_2_SOURCE          0x8C2
#define CS47L24_DRC1LMIX_INPUT_2_VOLUME          0x8C3
#define CS47L24_DRC1LMIX_INPUT_3_SOURCE          0x8C4
#define CS47L24_DRC1LMIX_INPUT_3_VOLUME          0x8C5
#define CS47L24_DRC1LMIX_INPUT_4_SOURCE          0x8C6
#define CS47L24_DRC1LMIX_INPUT_4_VOLUME          0x8C7
#define CS47L24_DRC1RMIX_INPUT_1_SOURCE          0x8C8
#define CS47L24_DRC1RMIX_INPUT_1_VOLUME          0x8C9
#define CS47L24_DRC1RMIX_INPUT_2_SOURCE          0x8CA
#define CS47L24_DRC1RMIX_INPUT_2_VOLUME          0x8CB
#define CS47L24_DRC1RMIX_INPUT_3_SOURCE          0x8CC
#define CS47L24_DRC1RMIX_INPUT_3_VOLUME          0x8CD
#define CS47L24_DRC1RMIX_INPUT_4_SOURCE          0x8CE
#define CS47L24_DRC1RMIX_INPUT_4_VOLUME          0x8CF
#define CS47L24_DRC2LMIX_INPUT_1_SOURCE          0x8D0
#define CS47L24_DRC2LMIX_INPUT_1_VOLUME          0x8D1
#define CS47L24_DRC2LMIX_INPUT_2_SOURCE          0x8D2
#define CS47L24_DRC2LMIX_INPUT_2_VOLUME          0x8D3
#define CS47L24_DRC2LMIX_INPUT_3_SOURCE          0x8D4
#define CS47L24_DRC2LMIX_INPUT_3_VOLUME          0x8D5
#define CS47L24_DRC2LMIX_INPUT_4_SOURCE          0x8D6
#define CS47L24_DRC2LMIX_INPUT_4_VOLUME          0x8D7
#define CS47L24_DRC2RMIX_INPUT_1_SOURCE          0x8D8
#define CS47L24_DRC2RMIX_INPUT_1_VOLUME          0x8D9
#define CS47L24_DRC2RMIX_INPUT_2_SOURCE          0x8DA
#define CS47L24_DRC2RMIX_INPUT_2_VOLUME          0x8DB
#define CS47L24_DRC2RMIX_INPUT_3_SOURCE          0x8DC
#define CS47L24_DRC2RMIX_INPUT_3_VOLUME          0x8DD
#define CS47L24_DRC2RMIX_INPUT_4_SOURCE          0x8DE
#define CS47L24_DRC2RMIX_INPUT_4_VOLUME          0x8DF
#define CS47L24_HPLP1MIX_INPUT_1_SOURCE          0x900
#define CS47L24_HPLP1MIX_INPUT_1_VOLUME          0x901
#define CS47L24_HPLP1MIX_INPUT_2_SOURCE          0x902
#define CS47L24_HPLP1MIX_INPUT_2_VOLUME          0x903
#define CS47L24_HPLP1MIX_INPUT_3_SOURCE          0x904
#define CS47L24_HPLP1MIX_INPUT_3_VOLUME          0x905
#define CS47L24_HPLP1MIX_INPUT_4_SOURCE          0x906
#define CS47L24_HPLP1MIX_INPUT_4_VOLUME          0x907
#define CS47L24_HPLP2MIX_INPUT_1_SOURCE          0x908
#define CS47L24_HPLP2MIX_INPUT_1_VOLUME          0x909
#define CS47L24_HPLP2MIX_INPUT_2_SOURCE          0x90A
#define CS47L24_HPLP2MIX_INPUT_2_VOLUME          0x90B
#define CS47L24_HPLP2MIX_INPUT_3_SOURCE          0x90C
#define CS47L24_HPLP2MIX_INPUT_3_VOLUME          0x90D
#define CS47L24_HPLP2MIX_INPUT_4_SOURCE          0x90E
#define CS47L24_HPLP2MIX_INPUT_4_VOLUME          0x90F
#define CS47L24_HPLP3MIX_INPUT_1_SOURCE          0x910
#define CS47L24_HPLP3MIX_INPUT_1_VOLUME          0x911
#define CS47L24_HPLP3MIX_INPUT_2_SOURCE          0x912
#define CS47L24_HPLP3MIX_INPUT_2_VOLUME          0x913
#define CS47L24_HPLP3MIX_INPUT_3_SOURCE          0x914
#define CS47L24_HPLP3MIX_INPUT_3_VOLUME          0x915
#define CS47L24_HPLP3MIX_INPUT_4_SOURCE          0x916
#define CS47L24_HPLP3MIX_INPUT_4_VOLUME          0x917
#define CS47L24_HPLP4MIX_INPUT_1_SOURCE          0x918
#define CS47L24_HPLP4MIX_INPUT_1_VOLUME          0x919
#define CS47L24_HPLP4MIX_INPUT_2_SOURCE          0x91A
#define CS47L24_HPLP4MIX_INPUT_2_VOLUME          0x91B
#define CS47L24_HPLP4MIX_INPUT_3_SOURCE          0x91C
#define CS47L24_HPLP4MIX_INPUT_3_VOLUME          0x91D
#define CS47L24_HPLP4MIX_INPUT_4_SOURCE          0x91E
#define CS47L24_HPLP4MIX_INPUT_4_VOLUME          0x91F
#define CS47L24_DSP2LMIX_INPUT_1_SOURCE          0x980
#define CS47L24_DSP2LMIX_INPUT_1_VOLUME          0x981
#define CS47L24_DSP2LMIX_INPUT_2_SOURCE          0x982
#define CS47L24_DSP2LMIX_INPUT_2_VOLUME          0x983
#define CS47L24_DSP2LMIX_INPUT_3_SOURCE          0x984
#define CS47L24_DSP2LMIX_INPUT_3_VOLUME          0x985
#define CS47L24_DSP2LMIX_INPUT_4_SOURCE          0x986
#define CS47L24_DSP2LMIX_INPUT_4_VOLUME          0x987
#define CS47L24_DSP2RMIX_INPUT_1_SOURCE          0x988
#define CS47L24_DSP2RMIX_INPUT_1_VOLUME          0x989
#define CS47L24_DSP2RMIX_INPUT_2_SOURCE          0x98A
#define CS47L24_DSP2RMIX_INPUT_2_VOLUME          0x98B
#define CS47L24_DSP2RMIX_INPUT_3_SOURCE          0x98C
#define CS47L24_DSP2RMIX_INPUT_3_VOLUME          0x98D
#define CS47L24_DSP2RMIX_INPUT_4_SOURCE          0x98E
#define CS47L24_DSP2RMIX_INPUT_4_VOLUME          0x98F
#define CS47L24_DSP2AUX1MIX_INPUT_1_SOURCE       0x990
#define CS47L24_DSP2AUX2MIX_INPUT_1_SOURCE       0x998
#define CS47L24_DSP2AUX3MIX_INPUT_1_SOURCE       0x9A0
#define CS47L24_DSP2AUX4MIX_INPUT_1_SOURCE       0x9A8
#define CS47L24_DSP2AUX5MIX_INPUT_1_SOURCE       0x9B0
#define CS47L24_DSP2AUX6MIX_INPUT_1_SOURCE       0x9B8
#define CS47L24_DSP3LMIX_INPUT_1_SOURCE          0x9C0
#define CS47L24_DSP3LMIX_INPUT_1_VOLUME          0x9C1
#define CS47L24_DSP3LMIX_INPUT_2_SOURCE          0x9C2
#define CS47L24_DSP3LMIX_INPUT_2_VOLUME          0x9C3
#define CS47L24_DSP3LMIX_INPUT_3_SOURCE          0x9C4
#define CS47L24_DSP3LMIX_INPUT_3_VOLUME          0x9C5
#define CS47L24_DSP3LMIX_INPUT_4_SOURCE          0x9C6
#define CS47L24_DSP3LMIX_INPUT_4_VOLUME          0x9C7
#define CS47L24_DSP3RMIX_INPUT_1_SOURCE          0x9C8
#define CS47L24_DSP3RMIX_INPUT_1_VOLUME          0x9C9
#define CS47L24_DSP3RMIX_INPUT_2_SOURCE          0x9CA
#define CS47L24_DSP3RMIX_INPUT_2_VOLUME          0x9CB
#define CS47L24_DSP3RMIX_INPUT_3_SOURCE          0x9CC
#define CS47L24_DSP3RMIX_INPUT_3_VOLUME          0x9CD
#define CS47L24_DSP3RMIX_INPUT_4_SOURCE          0x9CE
#define CS47L24_DSP3RMIX_INPUT_4_VOLUME          0x9CF
#define CS47L24_DSP3AUX1MIX_INPUT_1_SOURCE       0x9D0
#define CS47L24_DSP3AUX2MIX_INPUT_1_SOURCE       0x9D8
#define CS47L24_DSP3AUX3MIX_INPUT_1_SOURCE       0x9E0
#define CS47L24_DSP3AUX4MIX_INPUT_1_SOURCE       0x9E8
#define CS47L24_DSP3AUX5MIX_INPUT_1_SOURCE       0x9F0
#define CS47L24_DSP3AUX6MIX_INPUT_1_SOURCE       0x9F8
#define CS47L24_ASRC1LMIX_INPUT_1_SOURCE         0xA80
#define CS47L24_ASRC1RMIX_INPUT_1_SOURCE         0xA88
#define CS47L24_ASRC2LMIX_INPUT_1_SOURCE         0xA90
#define CS47L24_ASRC2RMIX_INPUT_1_SOURCE         0xA98
#define CS47L24_ISRC1DEC1MIX_INPUT_1_SOURCE      0xB00
#define CS47L24_ISRC1DEC2MIX_INPUT_1_SOURCE      0xB08
#define CS47L24_ISRC1DEC3MIX_INPUT_1_SOURCE      0xB10
#define CS47L24_ISRC1DEC4MIX_INPUT_1_SOURCE      0xB18
#define CS47L24_ISRC1INT1MIX_INPUT_1_SOURCE      0xB20
#define CS47L24_ISRC1INT2MIX_INPUT_1_SOURCE      0xB28
#define CS47L24_ISRC1INT3MIX_INPUT_1_SOURCE      0xB30
#define CS47L24_ISRC1INT4MIX_INPUT_1_SOURCE      0xB38
#define CS47L24_ISRC2DEC1MIX_INPUT_1_SOURCE      0xB40
#define CS47L24_ISRC2DEC2MIX_INPUT_1_SOURCE      0xB48
#define CS47L24_ISRC2DEC3MIX_INPUT_1_SOURCE      0xB50
#define CS47L24_ISRC2DEC4MIX_INPUT_1_SOURCE      0xB58
#define CS47L24_ISRC2INT1MIX_INPUT_1_SOURCE      0xB60
#define CS47L24_ISRC2INT2MIX_INPUT_1_SOURCE      0xB68
#define CS47L24_ISRC2INT3MIX_INPUT_1_SOURCE      0xB70
#define CS47L24_ISRC2INT4MIX_INPUT_1_SOURCE      0xB78
#define CS47L24_ISRC3DEC1MIX_INPUT_1_SOURCE      0xB80
#define CS47L24_ISRC3DEC2MIX_INPUT_1_SOURCE      0xB88
#define CS47L24_ISRC3DEC3MIX_INPUT_1_SOURCE      0xB90
#define CS47L24_ISRC3DEC4MIX_INPUT_1_SOURCE      0xB98
#define CS47L24_ISRC3INT1MIX_INPUT_1_SOURCE      0xBA0
#define CS47L24_ISRC3INT2MIX_INPUT_1_SOURCE      0xBA8
#define CS47L24_ISRC3INT3MIX_INPUT_1_SOURCE      0xBB0
#define CS47L24_ISRC3INT4MIX_INPUT_1_SOURCE      0xBB8
#define CS47L24_GPIO1_CTRL                       0xC00
#define CS47L24_GPIO2_CTRL                       0xC01
#define CS47L24_IRQ_CTRL_1                       0xC0F
#define CS47L24_GPIO_DEBOUNCE_CONFIG             0xC10
#define CS47L24_MISC_PAD_CTRL_1                  0xC20
#define CS47L24_MISC_PAD_CTRL_2                  0xC21
#define CS47L24_MISC_PAD_CTRL_3                  0xC22
#define CS47L24_MISC_PAD_CTRL_4                  0xC23
#define CS47L24_MISC_PAD_CTRL_5                  0xC24
#define CS47L24_MISC_PAD_CTRL_6                  0xC25
#define CS47L24_MISC_PAD_CTRL_7                  0xC30
#define CS47L24_MISC_PAD_CTRL_9                  0xC32
#define CS47L24_MISC_PAD_CTRL_10                 0xC33
#define CS47L24_MISC_PAD_CTRL_11                 0xC34
#define CS47L24_MISC_PAD_CTRL_12                 0xC35
#define CS47L24_MISC_PAD_CTRL_13                 0xC36
#define CS47L24_MISC_PAD_CTRL_14                 0xC37
#define CS47L24_MISC_PAD_CTRL_15                 0xC38
#define CS47L24_MISC_PAD_CTRL_16                 0xC39
#define CS47L24_INTERRUPT_STATUS_1               0xD00
#define CS47L24_INTERRUPT_STATUS_2               0xD01
#define CS47L24_INTERRUPT_STATUS_3               0xD02
#define CS47L24_INTERRUPT_STATUS_4               0xD03
#define CS47L24_INTERRUPT_STATUS_5               0xD04
#define CS47L24_INTERRUPT_STATUS_6               0xD05
#define CS47L24_INTERRUPT_STATUS_1_MASK          0xD08
#define CS47L24_INTERRUPT_STATUS_2_MASK          0xD09
#define CS47L24_INTERRUPT_STATUS_3_MASK          0xD0A
#define CS47L24_INTERRUPT_STATUS_4_MASK          0xD0B
#define CS47L24_INTERRUPT_STATUS_5_MASK          0xD0C
#define CS47L24_INTERRUPT_STATUS_6_MASK          0xD0D
#define CS47L24_INTERRUPT_CONTROL                0xD0F
#define CS47L24_IRQ2_STATUS_1                    0xD10
#define CS47L24_IRQ2_STATUS_2                    0xD11
#define CS47L24_IRQ2_STATUS_3                    0xD12
#define CS47L24_IRQ2_STATUS_4                    0xD13
#define CS47L24_IRQ2_STATUS_5                    0xD14
#define CS47L24_IRQ2_STATUS_6                    0xD15
#define CS47L24_IRQ2_STATUS_1_MASK               0xD18
#define CS47L24_IRQ2_STATUS_2_MASK               0xD19
#define CS47L24_IRQ2_STATUS_3_MASK               0xD1A
#define CS47L24_IRQ2_STATUS_4_MASK               0xD1B
#define CS47L24_IRQ2_STATUS_5_MASK               0xD1C
#define CS47L24_IRQ2_STATUS_6_MASK               0xD1D
#define CS47L24_IRQ2_CONTROL                     0xD1F
#define CS47L24_INTERRUPT_RAW_STATUS_1           0xD20
#define CS47L24_INTERRUPT_RAW_STATUS_2           0xD21
#define CS47L24_INTERRUPT_RAW_STATUS_3           0xD22
#define CS47L24_INTERRUPT_RAW_STATUS_4           0xD23
#define CS47L24_INTERRUPT_RAW_STATUS_5           0xD24
#define CS47L24_INTERRUPT_RAW_STATUS_6           0xD25
#define CS47L24_INTERRUPT_RAW_STATUS_7           0xD26
#define CS47L24_INTERRUPT_RAW_STATUS_8           0xD28
#define CS47L24_IRQ_PIN_STATUS                   0xD40
#define CS47L24_ADSP2_IRQ0                       0xD41
#define CS47L24_ADSP2_IRQ1                       0xD42
#define CS47L24_ADSP2_IRQ2                       0xD43
#define CS47L24_ADSP2_IRQ3                       0xD44
#define CS47L24_FX_CTRL1                         0xE00
#define CS47L24_FX_CTRL2                         0xE01
#define CS47L24_EQ1_1                            0xE10
#define CS47L24_EQ1_2                            0xE11
#define CS47L24_EQ1_3                            0xE12
#define CS47L24_EQ1_4                            0xE13
#define CS47L24_EQ1_5                            0xE14
#define CS47L24_EQ1_6                            0xE15
#define CS47L24_EQ1_7                            0xE16
#define CS47L24_EQ1_8                            0xE17
#define CS47L24_EQ1_9                            0xE18
#define CS47L24_EQ1_10                           0xE19
#define CS47L24_EQ1_11                           0xE1A
#define CS47L24_EQ1_12                           0xE1B
#define CS47L24_EQ1_13                           0xE1C
#define CS47L24_EQ1_14                           0xE1D
#define CS47L24_EQ1_15                           0xE1E
#define CS47L24_EQ1_16                           0xE1F
#define CS47L24_EQ1_17                           0xE20
#define CS47L24_EQ1_18                           0xE21
#define CS47L24_EQ1_19                           0xE22
#define CS47L24_EQ1_20                           0xE23
#define CS47L24_EQ1_21                           0xE24
#define CS47L24_EQ2_1                            0xE26
#define CS47L24_EQ2_2                            0xE27
#define CS47L24_EQ2_3                            0xE28
#define CS47L24_EQ2_4                            0xE29
#define CS47L24_EQ2_5                            0xE2A
#define CS47L24_EQ2_6                            0xE2B
#define CS47L24_EQ2_7                            0xE2C
#define CS47L24_EQ2_8                            0xE2D
#define CS47L24_EQ2_9                            0xE2E
#define CS47L24_EQ2_10                           0xE2F
#define CS47L24_EQ2_11                           0xE30
#define CS47L24_EQ2_12                           0xE31
#define CS47L24_EQ2_13                           0xE32
#define CS47L24_EQ2_14                           0xE33
#define CS47L24_EQ2_15                           0xE34
#define CS47L24_EQ2_16                           0xE35
#define CS47L24_EQ2_17                           0xE36
#define CS47L24_EQ2_18                           0xE37
#define CS47L24_EQ2_19                           0xE38
#define CS47L24_EQ2_20                           0xE39
#define CS47L24_EQ2_21                           0xE3A
#define CS47L24_DRC1_CTRL1                       0xE80
#define CS47L24_DRC1_CTRL2                       0xE81
#define CS47L24_DRC1_CTRL3                       0xE82
#define CS47L24_DRC1_CTRL4                       0xE83
#define CS47L24_DRC1_CTRL5                       0xE84
#define CS47L24_DRC2_CTRL1                       0xE89
#define CS47L24_DRC2_CTRL2                       0xE8A
#define CS47L24_DRC2_CTRL3                       0xE8B
#define CS47L24_DRC2_CTRL4                       0xE8C
#define CS47L24_DRC2_CTRL5                       0xE8D
#define CS47L24_HPLPF1_1                         0xEC0
#define CS47L24_HPLPF1_2                         0xEC1
#define CS47L24_HPLPF2_1                         0xEC4
#define CS47L24_HPLPF2_2                         0xEC5
#define CS47L24_HPLPF3_1                         0xEC8
#define CS47L24_HPLPF3_2                         0xEC9
#define CS47L24_HPLPF4_1                         0xECC
#define CS47L24_HPLPF4_2                         0xECD
#define CS47L24_ASRC_ENABLE                      0xEE0
#define CS47L24_ASRC_STATUS                      0xEE1
#define CS47L24_ASRC_RATE1                       0xEE2
#define CS47L24_ASRC_RATE2                       0xEE3
#define CS47L24_ISRC_1_CTRL_1                    0xEF0
#define CS47L24_ISRC_1_CTRL_2                    0xEF1
#define CS47L24_ISRC_1_CTRL_3                    0xEF2
#define CS47L24_ISRC_2_CTRL_1                    0xEF3
#define CS47L24_ISRC_2_CTRL_2                    0xEF4
#define CS47L24_ISRC_2_CTRL_3                    0xEF5
#define CS47L24_ISRC_3_CTRL_1                    0xEF6
#define CS47L24_ISRC_3_CTRL_2                    0xEF7
#define CS47L24_ISRC_3_CTRL_3                    0xEF8
#define CS47L24_DSP2_CONTROL_1                   0x1200
#define CS47L24_DSP2_CLOCKING_1                  0x1201
#define CS47L24_DSP2_STATUS_1                    0x1204
#define CS47L24_DSP2_STATUS_2                    0x1205
#define CS47L24_DSP2_STATUS_3                    0x1206
#define CS47L24_DSP2_WDMA_BUFFER_1               0x1210
#define CS47L24_DSP2_WDMA_BUFFER_2               0x1211
#define CS47L24_DSP2_WDMA_BUFFER_3               0x1212
#define CS47L24_DSP2_WDMA_BUFFER_4               0x1213
#define CS47L24_DSP2_WDMA_BUFFER_5               0x1214
#define CS47L24_DSP2_WDMA_BUFFER_6               0x1215
#define CS47L24_DSP2_WDMA_BUFFER_7               0x1216
#define CS47L24_DSP2_WDMA_BUFFER_8               0x1217
#define CS47L24_DSP2_RDMA_BUFFER_1               0x1220
#define CS47L24_DSP2_RDMA_BUFFER_2               0x1221
#define CS47L24_DSP2_RDMA_BUFFER_3               0x1222
#define CS47L24_DSP2_RDMA_BUFFER_4               0x1223
#define CS47L24_DSP2_RDMA_BUFFER_5               0x1224
#define CS47L24_DSP2_RDMA_BUFFER_6               0x1225
#define CS47L24_DSP2_WDMA_CONFIG_1               0x1230
#define CS47L24_DSP2_WDMA_CONFIG_2               0x1231
#define CS47L24_DSP2_WDMA_OFFSET_1               0x1232
#define CS47L24_DSP2_RDMA_CONFIG_1               0x1234
#define CS47L24_DSP2_RDMA_OFFSET_1               0x1235
#define CS47L24_DSP2_EXTERNAL_START_SELECT_1     0x1238
#define CS47L24_DSP2_SCRATCH_0                   0x1240
#define CS47L24_DSP2_SCRATCH_1                   0x1241
#define CS47L24_DSP2_SCRATCH_2                   0x1242
#define CS47L24_DSP2_SCRATCH_3                   0x1243
#define CS47L24_DSP3_CONTROL_1                   0x1300
#define CS47L24_DSP3_CLOCKING_1                  0x1301
#define CS47L24_DSP3_STATUS_1                    0x1304
#define CS47L24_DSP3_STATUS_2                    0x1305
#define CS47L24_DSP3_STATUS_3                    0x1306
#define CS47L24_DSP3_STATUS_4                    0x1307
#define CS47L24_DSP3_WDMA_BUFFER_1               0x1310
#define CS47L24_DSP3_WDMA_BUFFER_2               0x1311
#define CS47L24_DSP3_WDMA_BUFFER_3               0x1312
#define CS47L24_DSP3_WDMA_BUFFER_4               0x1313
#define CS47L24_DSP3_WDMA_BUFFER_5               0x1314
#define CS47L24_DSP3_WDMA_BUFFER_6               0x1315
#define CS47L24_DSP3_WDMA_BUFFER_7               0x1316
#define CS47L24_DSP3_WDMA_BUFFER_8               0x1317
#define CS47L24_DSP3_RDMA_BUFFER_1               0x1320
#define CS47L24_DSP3_RDMA_BUFFER_2               0x1321
#define CS47L24_DSP3_RDMA_BUFFER_3               0x1322
#define CS47L24_DSP3_RDMA_BUFFER_4               0x1323
#define CS47L24_DSP3_RDMA_BUFFER_5               0x1324
#define CS47L24_DSP3_RDMA_BUFFER_6               0x1325
#define CS47L24_DSP3_WDMA_CONFIG_1               0x1330
#define CS47L24_DSP3_WDMA_CONFIG_2               0x1331
#define CS47L24_DSP3_WDMA_OFFSET_1               0x1332
#define CS47L24_DSP3_RDMA_CONFIG_1               0x1334
#define CS47L24_DSP3_RDMA_OFFSET_1               0x1335
#define CS47L24_DSP3_EXTERNAL_START_SELECT_1     0x1338
#define CS47L24_DSP3_SCRATCH_0                   0x1340
#define CS47L24_DSP3_SCRATCH_1                   0x1341
#define CS47L24_DSP3_SCRATCH_2                   0x1342
#define CS47L24_DSP3_SCRATCH_3                   0x1343
#define CS47L24_WSEQ_SEQUENCE_1                  0x3000
#define CS47L24_WSEQ_SEQUENCE_2                  0x3002
#define CS47L24_WSEQ_SEQUENCE_3                  0x3004
#define CS47L24_WSEQ_SEQUENCE_4                  0x3006
#define CS47L24_WSEQ_SEQUENCE_5                  0x3008
#define CS47L24_WSEQ_SEQUENCE_6                  0x300A
#define CS47L24_WSEQ_SEQUENCE_7                  0x300C
#define CS47L24_WSEQ_SEQUENCE_8                  0x300E
#define CS47L24_WSEQ_SEQUENCE_9                  0x3010
#define CS47L24_WSEQ_SEQUENCE_10                 0x3012
#define CS47L24_WSEQ_SEQUENCE_11                 0x3014
#define CS47L24_WSEQ_SEQUENCE_12                 0x3016
#define CS47L24_WSEQ_SEQUENCE_13                 0x3018
#define CS47L24_WSEQ_SEQUENCE_14                 0x301A
#define CS47L24_WSEQ_SEQUENCE_15                 0x301C
#define CS47L24_WSEQ_SEQUENCE_16                 0x301E
#define CS47L24_WSEQ_SEQUENCE_17                 0x3020
#define CS47L24_WSEQ_SEQUENCE_18                 0x3022
#define CS47L24_WSEQ_SEQUENCE_19                 0x3024
#define CS47L24_WSEQ_SEQUENCE_20                 0x3026
#define CS47L24_WSEQ_SEQUENCE_21                 0x3028
#define CS47L24_WSEQ_SEQUENCE_22                 0x302A
#define CS47L24_WSEQ_SEQUENCE_23                 0x302C
#define CS47L24_WSEQ_SEQUENCE_24                 0x302E
#define CS47L24_WSEQ_SEQUENCE_25                 0x3030
#define CS47L24_WSEQ_SEQUENCE_26                 0x3032
#define CS47L24_WSEQ_SEQUENCE_27                 0x3034
#define CS47L24_WSEQ_SEQUENCE_28                 0x3036
#define CS47L24_WSEQ_SEQUENCE_29                 0x3038
#define CS47L24_WSEQ_SEQUENCE_30                 0x303A
#define CS47L24_WSEQ_SEQUENCE_31                 0x303C
#define CS47L24_WSEQ_SEQUENCE_32                 0x303E
#define CS47L24_WSEQ_SEQUENCE_33                 0x3040
#define CS47L24_WSEQ_SEQUENCE_34                 0x3042
#define CS47L24_WSEQ_SEQUENCE_35                 0x3044
#define CS47L24_WSEQ_SEQUENCE_36                 0x3046
#define CS47L24_WSEQ_SEQUENCE_37                 0x3048
#define CS47L24_WSEQ_SEQUENCE_38                 0x304A
#define CS47L24_WSEQ_SEQUENCE_39                 0x304C
#define CS47L24_WSEQ_SEQUENCE_40                 0x304E
#define CS47L24_WSEQ_SEQUENCE_41                 0x3050
#define CS47L24_WSEQ_SEQUENCE_42                 0x3052
#define CS47L24_WSEQ_SEQUENCE_43                 0x3054
#define CS47L24_WSEQ_SEQUENCE_44                 0x3056
#define CS47L24_WSEQ_SEQUENCE_45                 0x3058
#define CS47L24_WSEQ_SEQUENCE_46                 0x305A
#define CS47L24_WSEQ_SEQUENCE_47                 0x305C
#define CS47L24_WSEQ_SEQUENCE_48                 0x305E
#define CS47L24_WSEQ_SEQUENCE_49                 0x3060
#define CS47L24_WSEQ_SEQUENCE_50                 0x3062
#define CS47L24_WSEQ_SEQUENCE_51                 0x3064
#define CS47L24_WSEQ_SEQUENCE_52                 0x3066
#define CS47L24_WSEQ_SEQUENCE_53                 0x3068
#define CS47L24_WSEQ_SEQUENCE_54                 0x306A
#define CS47L24_WSEQ_SEQUENCE_55                 0x306C
#define CS47L24_WSEQ_SEQUENCE_56                 0x306E
#define CS47L24_WSEQ_SEQUENCE_57                 0x3070
#define CS47L24_WSEQ_SEQUENCE_58                 0x3072
#define CS47L24_WSEQ_SEQUENCE_59                 0x3074
#define CS47L24_WSEQ_SEQUENCE_60                 0x3076
#define CS47L24_WSEQ_SEQUENCE_61                 0x3078
#define CS47L24_WSEQ_SEQUENCE_62                 0x307A
#define CS47L24_WSEQ_SEQUENCE_63                 0x307C
#define CS47L24_WSEQ_SEQUENCE_64                 0x307E
#define CS47L24_WSEQ_SEQUENCE_65                 0x3080
#define CS47L24_WSEQ_SEQUENCE_66                 0x3082
#define CS47L24_WSEQ_SEQUENCE_67                 0x3084
#define CS47L24_WSEQ_SEQUENCE_68                 0x3086
#define CS47L24_WSEQ_SEQUENCE_69                 0x3088
#define CS47L24_WSEQ_SEQUENCE_70                 0x308A
#define CS47L24_WSEQ_SEQUENCE_71                 0x308C
#define CS47L24_WSEQ_SEQUENCE_72                 0x308E
#define CS47L24_WSEQ_SEQUENCE_73                 0x3090
#define CS47L24_WSEQ_SEQUENCE_74                 0x3092
#define CS47L24_WSEQ_SEQUENCE_75                 0x3094
#define CS47L24_WSEQ_SEQUENCE_76                 0x3096
#define CS47L24_WSEQ_SEQUENCE_77                 0x3098
#define CS47L24_WSEQ_SEQUENCE_78                 0x309A
#define CS47L24_WSEQ_SEQUENCE_79                 0x309C
#define CS47L24_WSEQ_SEQUENCE_80                 0x309E
#define CS47L24_WSEQ_SEQUENCE_81                 0x30A0
#define CS47L24_WSEQ_SEQUENCE_82                 0x30A2
#define CS47L24_WSEQ_SEQUENCE_83                 0x30A4
#define CS47L24_WSEQ_SEQUENCE_84                 0x30A6
#define CS47L24_WSEQ_SEQUENCE_85                 0x30A8
#define CS47L24_WSEQ_SEQUENCE_86                 0x30AA
#define CS47L24_WSEQ_SEQUENCE_87                 0x30AC
#define CS47L24_WSEQ_SEQUENCE_88                 0x30AE
#define CS47L24_WSEQ_SEQUENCE_89                 0x30B0
#define CS47L24_WSEQ_SEQUENCE_90                 0x30B2
#define CS47L24_WSEQ_SEQUENCE_91                 0x30B4
#define CS47L24_WSEQ_SEQUENCE_92                 0x30B6
#define CS47L24_WSEQ_SEQUENCE_93                 0x30B8
#define CS47L24_WSEQ_SEQUENCE_94                 0x30BA
#define CS47L24_WSEQ_SEQUENCE_95                 0x30BC
#define CS47L24_WSEQ_SEQUENCE_96                 0x30BE
#define CS47L24_WSEQ_SEQUENCE_97                 0x30C0
#define CS47L24_WSEQ_SEQUENCE_98                 0x30C2
#define CS47L24_WSEQ_SEQUENCE_99                 0x30C4
#define CS47L24_WSEQ_SEQUENCE_100                0x30C6
#define CS47L24_WSEQ_SEQUENCE_101                0x30C8
#define CS47L24_WSEQ_SEQUENCE_102                0x30CA
#define CS47L24_WSEQ_SEQUENCE_103                0x30CC
#define CS47L24_WSEQ_SEQUENCE_104                0x30CE
#define CS47L24_WSEQ_SEQUENCE_105                0x30D0
#define CS47L24_WSEQ_SEQUENCE_106                0x30D2
#define CS47L24_WSEQ_SEQUENCE_107                0x30D4
#define CS47L24_WSEQ_SEQUENCE_108                0x30D6
#define CS47L24_WSEQ_SEQUENCE_109                0x30D8
#define CS47L24_WSEQ_SEQUENCE_110                0x30DA
#define CS47L24_WSEQ_SEQUENCE_111                0x30DC
#define CS47L24_WSEQ_SEQUENCE_112                0x30DE
#define CS47L24_WSEQ_SEQUENCE_113                0x30E0
#define CS47L24_WSEQ_SEQUENCE_114                0x30E2
#define CS47L24_WSEQ_SEQUENCE_115                0x30E4
#define CS47L24_WSEQ_SEQUENCE_116                0x30E6
#define CS47L24_WSEQ_SEQUENCE_117                0x30E8
#define CS47L24_WSEQ_SEQUENCE_118                0x30EA
#define CS47L24_WSEQ_SEQUENCE_119                0x30EC
#define CS47L24_WSEQ_SEQUENCE_120                0x30EE
#define CS47L24_WSEQ_SEQUENCE_121                0x30F0
#define CS47L24_WSEQ_SEQUENCE_122                0x30F2
#define CS47L24_WSEQ_SEQUENCE_123                0x30F4
#define CS47L24_WSEQ_SEQUENCE_124                0x30F6
#define CS47L24_WSEQ_SEQUENCE_125                0x30F8
#define CS47L24_WSEQ_SEQUENCE_126                0x30FA
#define CS47L24_WSEQ_SEQUENCE_127                0x30FC
#define CS47L24_WSEQ_SEQUENCE_128                0x30FE
#define CS47L24_WSEQ_SEQUENCE_129                0x3100
#define CS47L24_WSEQ_SEQUENCE_130                0x3102
#define CS47L24_WSEQ_SEQUENCE_131                0x3104
#define CS47L24_WSEQ_SEQUENCE_132                0x3106
#define CS47L24_WSEQ_SEQUENCE_133                0x3108
#define CS47L24_WSEQ_SEQUENCE_134                0x310A
#define CS47L24_WSEQ_SEQUENCE_135                0x310C
#define CS47L24_WSEQ_SEQUENCE_136                0x310E
#define CS47L24_WSEQ_SEQUENCE_137                0x3110
#define CS47L24_WSEQ_SEQUENCE_138                0x3112
#define CS47L24_WSEQ_SEQUENCE_139                0x3114
#define CS47L24_WSEQ_SEQUENCE_140                0x3116
#define CS47L24_WSEQ_SEQUENCE_141                0x3118
#define CS47L24_WSEQ_SEQUENCE_142                0x311A
#define CS47L24_WSEQ_SEQUENCE_143                0x311C
#define CS47L24_WSEQ_SEQUENCE_144                0x311E
#define CS47L24_WSEQ_SEQUENCE_145                0x3120
#define CS47L24_WSEQ_SEQUENCE_146                0x3122
#define CS47L24_WSEQ_SEQUENCE_147                0x3124
#define CS47L24_WSEQ_SEQUENCE_148                0x3126
#define CS47L24_WSEQ_SEQUENCE_149                0x3128
#define CS47L24_WSEQ_SEQUENCE_150                0x312A
#define CS47L24_WSEQ_SEQUENCE_151                0x312C
#define CS47L24_WSEQ_SEQUENCE_152                0x312E
#define CS47L24_WSEQ_SEQUENCE_153                0x3130
#define CS47L24_WSEQ_SEQUENCE_154                0x3132
#define CS47L24_WSEQ_SEQUENCE_155                0x3134
#define CS47L24_WSEQ_SEQUENCE_156                0x3136
#define CS47L24_WSEQ_SEQUENCE_157                0x3138
#define CS47L24_WSEQ_SEQUENCE_158                0x313A
#define CS47L24_WSEQ_SEQUENCE_159                0x313C
#define CS47L24_WSEQ_SEQUENCE_160                0x313E
#define CS47L24_WSEQ_SEQUENCE_161                0x3140
#define CS47L24_WSEQ_SEQUENCE_162                0x3142
#define CS47L24_WSEQ_SEQUENCE_163                0x3144
#define CS47L24_WSEQ_SEQUENCE_164                0x3146
#define CS47L24_WSEQ_SEQUENCE_165                0x3148
#define CS47L24_WSEQ_SEQUENCE_166                0x314A
#define CS47L24_WSEQ_SEQUENCE_167                0x314C
#define CS47L24_WSEQ_SEQUENCE_168                0x314E
#define CS47L24_WSEQ_SEQUENCE_169                0x3150
#define CS47L24_WSEQ_SEQUENCE_170                0x3152
#define CS47L24_WSEQ_SEQUENCE_171                0x3154
#define CS47L24_WSEQ_SEQUENCE_172                0x3156
#define CS47L24_WSEQ_SEQUENCE_173                0x3158
#define CS47L24_WSEQ_SEQUENCE_174                0x315A
#define CS47L24_WSEQ_SEQUENCE_175                0x315C
#define CS47L24_WSEQ_SEQUENCE_176                0x315E
#define CS47L24_WSEQ_SEQUENCE_177                0x3160
#define CS47L24_WSEQ_SEQUENCE_178                0x3162
#define CS47L24_WSEQ_SEQUENCE_179                0x3164
#define CS47L24_WSEQ_SEQUENCE_180                0x3166
#define CS47L24_WSEQ_SEQUENCE_181                0x3168
#define CS47L24_WSEQ_SEQUENCE_182                0x316A
#define CS47L24_WSEQ_SEQUENCE_183                0x316C
#define CS47L24_WSEQ_SEQUENCE_184                0x316E
#define CS47L24_WSEQ_SEQUENCE_185                0x3170
#define CS47L24_WSEQ_SEQUENCE_186                0x3172
#define CS47L24_WSEQ_SEQUENCE_187                0x3174
#define CS47L24_WSEQ_SEQUENCE_188                0x3176
#define CS47L24_WSEQ_SEQUENCE_189                0x3178
#define CS47L24_WSEQ_SEQUENCE_190                0x317A
#define CS47L24_WSEQ_SEQUENCE_191                0x317C
#define CS47L24_WSEQ_SEQUENCE_192                0x317E
#define CS47L24_WSEQ_SEQUENCE_193                0x3180
#define CS47L24_WSEQ_SEQUENCE_194                0x3182
#define CS47L24_WSEQ_SEQUENCE_195                0x3184
#define CS47L24_WSEQ_SEQUENCE_196                0x3186
#define CS47L24_WSEQ_SEQUENCE_197                0x3188
#define CS47L24_WSEQ_SEQUENCE_198                0x318A
#define CS47L24_WSEQ_SEQUENCE_199                0x318C
#define CS47L24_WSEQ_SEQUENCE_200                0x318E
#define CS47L24_WSEQ_SEQUENCE_201                0x3190
#define CS47L24_WSEQ_SEQUENCE_202                0x3192
#define CS47L24_WSEQ_SEQUENCE_203                0x3194
#define CS47L24_WSEQ_SEQUENCE_204                0x3196
#define CS47L24_WSEQ_SEQUENCE_205                0x3198
#define CS47L24_WSEQ_SEQUENCE_206                0x319A
#define CS47L24_WSEQ_SEQUENCE_207                0x319C
#define CS47L24_WSEQ_SEQUENCE_208                0x319E
#define CS47L24_WSEQ_SEQUENCE_209                0x31A0
#define CS47L24_WSEQ_SEQUENCE_210                0x31A2
#define CS47L24_WSEQ_SEQUENCE_211                0x31A4
#define CS47L24_WSEQ_SEQUENCE_212                0x31A6
#define CS47L24_WSEQ_SEQUENCE_213                0x31A8
#define CS47L24_WSEQ_SEQUENCE_214                0x31AA
#define CS47L24_WSEQ_SEQUENCE_215                0x31AC
#define CS47L24_WSEQ_SEQUENCE_216                0x31AE
#define CS47L24_WSEQ_SEQUENCE_217                0x31B0
#define CS47L24_WSEQ_SEQUENCE_218                0x31B2
#define CS47L24_WSEQ_SEQUENCE_219                0x31B4
#define CS47L24_WSEQ_SEQUENCE_220                0x31B6
#define CS47L24_WSEQ_SEQUENCE_221                0x31B8
#define CS47L24_WSEQ_SEQUENCE_222                0x31BA
#define CS47L24_WSEQ_SEQUENCE_223                0x31BC
#define CS47L24_WSEQ_SEQUENCE_224                0x31BE
#define CS47L24_WSEQ_SEQUENCE_225                0x31C0
#define CS47L24_WSEQ_SEQUENCE_226                0x31C2
#define CS47L24_WSEQ_SEQUENCE_227                0x31C4
#define CS47L24_WSEQ_SEQUENCE_228                0x31C6
#define CS47L24_WSEQ_SEQUENCE_229                0x31C8
#define CS47L24_WSEQ_SEQUENCE_230                0x31CA
#define CS47L24_WSEQ_SEQUENCE_231                0x31CC
#define CS47L24_WSEQ_SEQUENCE_232                0x31CE
#define CS47L24_WSEQ_SEQUENCE_233                0x31D0
#define CS47L24_WSEQ_SEQUENCE_234                0x31D2
#define CS47L24_WSEQ_SEQUENCE_235                0x31D4
#define CS47L24_WSEQ_SEQUENCE_236                0x31D6
#define CS47L24_WSEQ_SEQUENCE_237                0x31D8
#define CS47L24_WSEQ_SEQUENCE_238                0x31DA
#define CS47L24_WSEQ_SEQUENCE_239                0x31DC
#define CS47L24_WSEQ_SEQUENCE_240                0x31DE
#define CS47L24_WSEQ_SEQUENCE_241                0x31E0
#define CS47L24_WSEQ_SEQUENCE_242                0x31E2
#define CS47L24_WSEQ_SEQUENCE_243                0x31E4
#define CS47L24_WSEQ_SEQUENCE_244                0x31E6
#define CS47L24_WSEQ_SEQUENCE_245                0x31E8
#define CS47L24_WSEQ_SEQUENCE_246                0x31EA
#define CS47L24_WSEQ_SEQUENCE_247                0x31EC
#define CS47L24_WSEQ_SEQUENCE_248                0x31EE
#define CS47L24_WSEQ_SEQUENCE_249                0x31F0
#define CS47L24_WSEQ_SEQUENCE_250                0x31F2
#define CS47L24_WSEQ_SEQUENCE_251                0x31F4
#define CS47L24_WSEQ_SEQUENCE_252                0x31F6
#define CS47L24_WSEQ_SEQUENCE_253                0x31F8
#define CS47L24_WSEQ_SEQUENCE_254                0x31FA
#define CS47L24_WSEQ_SEQUENCE_255                0x31FC
#define CS47L24_WSEQ_SEQUENCE_256                0x31FE
#define CS47L24_WSEQ_SEQUENCE_257                0x3200
#define CS47L24_WSEQ_SEQUENCE_258                0x3202
#define CS47L24_WSEQ_SEQUENCE_259                0x3204
#define CS47L24_WSEQ_SEQUENCE_260                0x3206
#define CS47L24_WSEQ_SEQUENCE_261                0x3208
#define CS47L24_WSEQ_SEQUENCE_262                0x320A
#define CS47L24_WSEQ_SEQUENCE_263                0x320C
#define CS47L24_WSEQ_SEQUENCE_264                0x320E
#define CS47L24_WSEQ_SEQUENCE_265                0x3210
#define CS47L24_WSEQ_SEQUENCE_266                0x3212
#define CS47L24_WSEQ_SEQUENCE_267                0x3214
#define CS47L24_WSEQ_SEQUENCE_268                0x3216
#define CS47L24_WSEQ_SEQUENCE_269                0x3218
#define CS47L24_WSEQ_SEQUENCE_270                0x321A
#define CS47L24_WSEQ_SEQUENCE_271                0x321C
#define CS47L24_WSEQ_SEQUENCE_272                0x321E
#define CS47L24_WSEQ_SEQUENCE_273                0x3220
#define CS47L24_WSEQ_SEQUENCE_274                0x3222
#define CS47L24_WSEQ_SEQUENCE_275                0x3224
#define CS47L24_WSEQ_SEQUENCE_276                0x3226
#define CS47L24_WSEQ_SEQUENCE_277                0x3228
#define CS47L24_WSEQ_SEQUENCE_278                0x322A
#define CS47L24_WSEQ_SEQUENCE_279                0x322C
#define CS47L24_WSEQ_SEQUENCE_280                0x322E
#define CS47L24_WSEQ_SEQUENCE_281                0x3230
#define CS47L24_WSEQ_SEQUENCE_282                0x3232
#define CS47L24_WSEQ_SEQUENCE_283                0x3234
#define CS47L24_WSEQ_SEQUENCE_284                0x3236
#define CS47L24_WSEQ_SEQUENCE_285                0x3238
#define CS47L24_WSEQ_SEQUENCE_286                0x323A
#define CS47L24_WSEQ_SEQUENCE_287                0x323C
#define CS47L24_WSEQ_SEQUENCE_288                0x323E
#define CS47L24_WSEQ_SEQUENCE_289                0x3240
#define CS47L24_WSEQ_SEQUENCE_290                0x3242
#define CS47L24_WSEQ_SEQUENCE_291                0x3244
#define CS47L24_WSEQ_SEQUENCE_292                0x3246
#define CS47L24_WSEQ_SEQUENCE_293                0x3248
#define CS47L24_WSEQ_SEQUENCE_294                0x324A
#define CS47L24_WSEQ_SEQUENCE_295                0x324C
#define CS47L24_WSEQ_SEQUENCE_296                0x324E
#define CS47L24_WSEQ_SEQUENCE_297                0x3250
#define CS47L24_WSEQ_SEQUENCE_298                0x3252
#define CS47L24_WSEQ_SEQUENCE_299                0x3254
#define CS47L24_WSEQ_SEQUENCE_300                0x3256
#define CS47L24_WSEQ_SEQUENCE_301                0x3258
#define CS47L24_WSEQ_SEQUENCE_302                0x325A
#define CS47L24_WSEQ_SEQUENCE_303                0x325C
#define CS47L24_WSEQ_SEQUENCE_304                0x325E
#define CS47L24_WSEQ_SEQUENCE_305                0x3260
#define CS47L24_WSEQ_SEQUENCE_306                0x3262
#define CS47L24_WSEQ_SEQUENCE_307                0x3264
#define CS47L24_WSEQ_SEQUENCE_308                0x3266
#define CS47L24_WSEQ_SEQUENCE_309                0x3268
#define CS47L24_WSEQ_SEQUENCE_310                0x326A
#define CS47L24_WSEQ_SEQUENCE_311                0x326C
#define CS47L24_WSEQ_SEQUENCE_312                0x326E
#define CS47L24_WSEQ_SEQUENCE_313                0x3270
#define CS47L24_WSEQ_SEQUENCE_314                0x3272
#define CS47L24_WSEQ_SEQUENCE_315                0x3274
#define CS47L24_WSEQ_SEQUENCE_316                0x3276
#define CS47L24_WSEQ_SEQUENCE_317                0x3278
#define CS47L24_WSEQ_SEQUENCE_318                0x327A
#define CS47L24_WSEQ_SEQUENCE_319                0x327C
#define CS47L24_WSEQ_SEQUENCE_320                0x327E
#define CS47L24_WSEQ_SEQUENCE_321                0x3280
#define CS47L24_WSEQ_SEQUENCE_322                0x3282
#define CS47L24_WSEQ_SEQUENCE_323                0x3284
#define CS47L24_WSEQ_SEQUENCE_324                0x3286
#define CS47L24_WSEQ_SEQUENCE_325                0x3288
#define CS47L24_WSEQ_SEQUENCE_326                0x328A
#define CS47L24_WSEQ_SEQUENCE_327                0x328C
#define CS47L24_WSEQ_SEQUENCE_328                0x328E
#define CS47L24_WSEQ_SEQUENCE_329                0x3290
#define CS47L24_WSEQ_SEQUENCE_330                0x3292
#define CS47L24_WSEQ_SEQUENCE_331                0x3294
#define CS47L24_WSEQ_SEQUENCE_332                0x3296
#define CS47L24_WSEQ_SEQUENCE_333                0x3298
#define CS47L24_WSEQ_SEQUENCE_334                0x329A
#define CS47L24_WSEQ_SEQUENCE_335                0x329C
#define CS47L24_WSEQ_SEQUENCE_336                0x329E
#define CS47L24_WSEQ_SEQUENCE_337                0x32A0
#define CS47L24_WSEQ_SEQUENCE_338                0x32A2
#define CS47L24_WSEQ_SEQUENCE_339                0x32A4
#define CS47L24_WSEQ_SEQUENCE_340                0x32A6
#define CS47L24_WSEQ_SEQUENCE_341                0x32A8
#define CS47L24_WSEQ_SEQUENCE_342                0x32AA
#define CS47L24_WSEQ_SEQUENCE_343                0x32AC
#define CS47L24_WSEQ_SEQUENCE_344                0x32AE
#define CS47L24_WSEQ_SEQUENCE_345                0x32B0
#define CS47L24_WSEQ_SEQUENCE_346                0x32B2
#define CS47L24_WSEQ_SEQUENCE_347                0x32B4
#define CS47L24_WSEQ_SEQUENCE_348                0x32B6
#define CS47L24_WSEQ_SEQUENCE_349                0x32B8
#define CS47L24_WSEQ_SEQUENCE_350                0x32BA
#define CS47L24_WSEQ_SEQUENCE_351                0x32BC
#define CS47L24_WSEQ_SEQUENCE_352                0x32BE
#define CS47L24_WSEQ_SEQUENCE_353                0x32C0
#define CS47L24_WSEQ_SEQUENCE_354                0x32C2
#define CS47L24_WSEQ_SEQUENCE_355                0x32C4
#define CS47L24_WSEQ_SEQUENCE_356                0x32C6
#define CS47L24_WSEQ_SEQUENCE_357                0x32C8
#define CS47L24_WSEQ_SEQUENCE_358                0x32CA
#define CS47L24_WSEQ_SEQUENCE_359                0x32CC
#define CS47L24_WSEQ_SEQUENCE_360                0x32CE
#define CS47L24_WSEQ_SEQUENCE_361                0x32D0
#define CS47L24_WSEQ_SEQUENCE_362                0x32D2
#define CS47L24_WSEQ_SEQUENCE_363                0x32D4
#define CS47L24_WSEQ_SEQUENCE_364                0x32D6
#define CS47L24_WSEQ_SEQUENCE_365                0x32D8
#define CS47L24_WSEQ_SEQUENCE_366                0x32DA
#define CS47L24_WSEQ_SEQUENCE_367                0x32DC
#define CS47L24_WSEQ_SEQUENCE_368                0x32DE
#define CS47L24_WSEQ_SEQUENCE_369                0x32E0
#define CS47L24_WSEQ_SEQUENCE_370                0x32E2
#define CS47L24_WSEQ_SEQUENCE_371                0x32E4
#define CS47L24_WSEQ_SEQUENCE_372                0x32E6
#define CS47L24_WSEQ_SEQUENCE_373                0x32E8
#define CS47L24_WSEQ_SEQUENCE_374                0x32EA
#define CS47L24_WSEQ_SEQUENCE_375                0x32EC
#define CS47L24_WSEQ_SEQUENCE_376                0x32EE
#define CS47L24_WSEQ_SEQUENCE_377                0x32F0
#define CS47L24_WSEQ_SEQUENCE_378                0x32F2
#define CS47L24_WSEQ_SEQUENCE_379                0x32F4
#define CS47L24_WSEQ_SEQUENCE_380                0x32F6
#define CS47L24_WSEQ_SEQUENCE_381                0x32F8
#define CS47L24_WSEQ_SEQUENCE_382                0x32FA
#define CS47L24_WSEQ_SEQUENCE_383                0x32FC
#define CS47L24_WSEQ_SEQUENCE_384                0x32FE
#define CS47L24_WSEQ_SEQUENCE_385                0x3300
#define CS47L24_WSEQ_SEQUENCE_386                0x3302
#define CS47L24_WSEQ_SEQUENCE_387                0x3304
#define CS47L24_WSEQ_SEQUENCE_388                0x3306
#define CS47L24_WSEQ_SEQUENCE_389                0x3308
#define CS47L24_WSEQ_SEQUENCE_390                0x330A
#define CS47L24_WSEQ_SEQUENCE_391                0x330C
#define CS47L24_WSEQ_SEQUENCE_392                0x330E
#define CS47L24_WSEQ_SEQUENCE_393                0x3310
#define CS47L24_WSEQ_SEQUENCE_394                0x3312
#define CS47L24_WSEQ_SEQUENCE_395                0x3314
#define CS47L24_WSEQ_SEQUENCE_396                0x3316
#define CS47L24_WSEQ_SEQUENCE_397                0x3318
#define CS47L24_WSEQ_SEQUENCE_398                0x331A
#define CS47L24_WSEQ_SEQUENCE_399                0x331C
#define CS47L24_WSEQ_SEQUENCE_400                0x331E
#define CS47L24_WSEQ_SEQUENCE_401                0x3320
#define CS47L24_WSEQ_SEQUENCE_402                0x3322
#define CS47L24_WSEQ_SEQUENCE_403                0x3324
#define CS47L24_WSEQ_SEQUENCE_404                0x3326
#define CS47L24_WSEQ_SEQUENCE_405                0x3328
#define CS47L24_WSEQ_SEQUENCE_406                0x332A
#define CS47L24_WSEQ_SEQUENCE_407                0x332C
#define CS47L24_WSEQ_SEQUENCE_408                0x332E
#define CS47L24_WSEQ_SEQUENCE_409                0x3330
#define CS47L24_WSEQ_SEQUENCE_410                0x3332
#define CS47L24_WSEQ_SEQUENCE_411                0x3334
#define CS47L24_WSEQ_SEQUENCE_412                0x3336
#define CS47L24_WSEQ_SEQUENCE_413                0x3338
#define CS47L24_WSEQ_SEQUENCE_414                0x333A
#define CS47L24_WSEQ_SEQUENCE_415                0x333C
#define CS47L24_WSEQ_SEQUENCE_416                0x333E
#define CS47L24_WSEQ_SEQUENCE_417                0x3340
#define CS47L24_WSEQ_SEQUENCE_418                0x3342
#define CS47L24_WSEQ_SEQUENCE_419                0x3344
#define CS47L24_WSEQ_SEQUENCE_420                0x3346
#define CS47L24_WSEQ_SEQUENCE_421                0x3348
#define CS47L24_WSEQ_SEQUENCE_422                0x334A
#define CS47L24_WSEQ_SEQUENCE_423                0x334C
#define CS47L24_WSEQ_SEQUENCE_424                0x334E
#define CS47L24_WSEQ_SEQUENCE_425                0x3350
#define CS47L24_WSEQ_SEQUENCE_426                0x3352
#define CS47L24_WSEQ_SEQUENCE_427                0x3354
#define CS47L24_WSEQ_SEQUENCE_428                0x3356
#define CS47L24_WSEQ_SEQUENCE_429                0x3358
#define CS47L24_WSEQ_SEQUENCE_430                0x335A
#define CS47L24_WSEQ_SEQUENCE_431                0x335C
#define CS47L24_WSEQ_SEQUENCE_432                0x335E
#define CS47L24_WSEQ_SEQUENCE_433                0x3360
#define CS47L24_WSEQ_SEQUENCE_434                0x3362
#define CS47L24_WSEQ_SEQUENCE_435                0x3364
#define CS47L24_WSEQ_SEQUENCE_436                0x3366
#define CS47L24_WSEQ_SEQUENCE_437                0x3368
#define CS47L24_WSEQ_SEQUENCE_438                0x336A
#define CS47L24_WSEQ_SEQUENCE_439                0x336C
#define CS47L24_WSEQ_SEQUENCE_440                0x336E
#define CS47L24_WSEQ_SEQUENCE_441                0x3370
#define CS47L24_WSEQ_SEQUENCE_442                0x3372
#define CS47L24_WSEQ_SEQUENCE_443                0x3374
#define CS47L24_WSEQ_SEQUENCE_444                0x3376
#define CS47L24_WSEQ_SEQUENCE_445                0x3378
#define CS47L24_WSEQ_SEQUENCE_446                0x337A
#define CS47L24_WSEQ_SEQUENCE_447                0x337C
#define CS47L24_WSEQ_SEQUENCE_448                0x337E
#define CS47L24_WSEQ_SEQUENCE_449                0x3380
#define CS47L24_WSEQ_SEQUENCE_450                0x3382
#define CS47L24_WSEQ_SEQUENCE_451                0x3384
#define CS47L24_WSEQ_SEQUENCE_452                0x3386
#define CS47L24_WSEQ_SEQUENCE_453                0x3388
#define CS47L24_WSEQ_SEQUENCE_454                0x338A
#define CS47L24_WSEQ_SEQUENCE_455                0x338C
#define CS47L24_WSEQ_SEQUENCE_456                0x338E
#define CS47L24_WSEQ_SEQUENCE_457                0x3390
#define CS47L24_WSEQ_SEQUENCE_458                0x3392
#define CS47L24_WSEQ_SEQUENCE_459                0x3394
#define CS47L24_WSEQ_SEQUENCE_460                0x3396
#define CS47L24_WSEQ_SEQUENCE_461                0x3398
#define CS47L24_WSEQ_SEQUENCE_462                0x339A
#define CS47L24_WSEQ_SEQUENCE_463                0x339C
#define CS47L24_WSEQ_SEQUENCE_464                0x339E
#define CS47L24_WSEQ_SEQUENCE_465                0x33A0
#define CS47L24_WSEQ_SEQUENCE_466                0x33A2
#define CS47L24_WSEQ_SEQUENCE_467                0x33A4
#define CS47L24_WSEQ_SEQUENCE_468                0x33A6
#define CS47L24_WSEQ_SEQUENCE_469                0x33A8
#define CS47L24_WSEQ_SEQUENCE_470                0x33AA
#define CS47L24_WSEQ_SEQUENCE_471                0x33AC
#define CS47L24_WSEQ_SEQUENCE_472                0x33AE
#define CS47L24_WSEQ_SEQUENCE_473                0x33B0
#define CS47L24_WSEQ_SEQUENCE_474                0x33B2
#define CS47L24_WSEQ_SEQUENCE_475                0x33B4
#define CS47L24_WSEQ_SEQUENCE_476                0x33B6
#define CS47L24_WSEQ_SEQUENCE_477                0x33B8
#define CS47L24_WSEQ_SEQUENCE_478                0x33BA
#define CS47L24_WSEQ_SEQUENCE_479                0x33BC
#define CS47L24_WSEQ_SEQUENCE_480                0x33BE
#define CS47L24_WSEQ_SEQUENCE_481                0x33C0
#define CS47L24_WSEQ_SEQUENCE_482                0x33C2
#define CS47L24_WSEQ_SEQUENCE_483                0x33C4
#define CS47L24_WSEQ_SEQUENCE_484                0x33C6
#define CS47L24_WSEQ_SEQUENCE_485                0x33C8
#define CS47L24_WSEQ_SEQUENCE_486                0x33CA
#define CS47L24_WSEQ_SEQUENCE_487                0x33CC
#define CS47L24_WSEQ_SEQUENCE_488                0x33CE
#define CS47L24_WSEQ_SEQUENCE_489                0x33D0
#define CS47L24_WSEQ_SEQUENCE_490                0x33D2
#define CS47L24_WSEQ_SEQUENCE_491                0x33D4
#define CS47L24_WSEQ_SEQUENCE_492                0x33D6
#define CS47L24_WSEQ_SEQUENCE_493                0x33D8
#define CS47L24_WSEQ_SEQUENCE_494                0x33DA
#define CS47L24_WSEQ_SEQUENCE_495                0x33DC
#define CS47L24_WSEQ_SEQUENCE_496                0x33DE
#define CS47L24_WSEQ_SEQUENCE_497                0x33E0
#define CS47L24_WSEQ_SEQUENCE_498                0x33E2
#define CS47L24_WSEQ_SEQUENCE_499                0x33E4
#define CS47L24_WSEQ_SEQUENCE_500                0x33E6
#define CS47L24_WSEQ_SEQUENCE_501                0x33E8
#define CS47L24_WSEQ_SEQUENCE_502                0x33EA
#define CS47L24_WSEQ_SEQUENCE_503                0x33EC
#define CS47L24_WSEQ_SEQUENCE_504                0x33EE
#define CS47L24_WSEQ_SEQUENCE_505                0x33F0
#define CS47L24_WSEQ_SEQUENCE_506                0x33F2
#define CS47L24_WSEQ_SEQUENCE_507                0x33F4
#define CS47L24_WSEQ_SEQUENCE_508                0x33F6
#define CS47L24_WSEQ_SEQUENCE_509                0x33F8
#define CS47L24_WSEQ_SEQUENCE_510                0x33FA


/*
 * Field Definitions.
 */

/*
 * R0 (0x00) - software reset
 */
#define CS47L24_SW_RST_DEV_ID1_MASK              0xFFFF  /* SW_RST_DEV_ID1 - [15:0] */
#define CS47L24_SW_RST_DEV_ID1_SHIFT                  0  /* SW_RST_DEV_ID1 - [15:0] */
#define CS47L24_SW_RST_DEV_ID1_WIDTH                 16  /* SW_RST_DEV_ID1 - [15:0] */

/*
 * R1 (0x01) - Device Revision
 */
#define CS47L24_DEVICE_REVISION_MASK             0x00FF  /* DEVICE_REVISION - [7:0] */
#define CS47L24_DEVICE_REVISION_SHIFT                 0  /* DEVICE_REVISION - [7:0] */
#define CS47L24_DEVICE_REVISION_WIDTH                 8  /* DEVICE_REVISION - [7:0] */

/*
 * R8 (0x08) - Ctrl IF SPI CFG 1
 */
#define CS47L24_SPI_CFG                          0x0010  /* SPI_CFG */
#define CS47L24_SPI_CFG_MASK                     0x0010  /* SPI_CFG */
#define CS47L24_SPI_CFG_SHIFT                         4  /* SPI_CFG */
#define CS47L24_SPI_CFG_WIDTH                         1  /* SPI_CFG */
#define CS47L24_SPI_4WIRE                        0x0008  /* SPI_4WIRE */
#define CS47L24_SPI_4WIRE_MASK                   0x0008  /* SPI_4WIRE */
#define CS47L24_SPI_4WIRE_SHIFT                       3  /* SPI_4WIRE */
#define CS47L24_SPI_4WIRE_WIDTH                       1  /* SPI_4WIRE */
#define CS47L24_SPI_AUTO_INC_MASK                0x0003  /* SPI_AUTO_INC - [1:0] */
#define CS47L24_SPI_AUTO_INC_SHIFT                    0  /* SPI_AUTO_INC - [1:0] */
#define CS47L24_SPI_AUTO_INC_WIDTH                    2  /* SPI_AUTO_INC - [1:0] */

/*
 * R22 (0x16) - Write Sequencer Ctrl 0
 */
#define CS47L24_WSEQ_ABORT                       0x0800  /* WSEQ_ABORT */
#define CS47L24_WSEQ_ABORT_MASK                  0x0800  /* WSEQ_ABORT */
#define CS47L24_WSEQ_ABORT_SHIFT                     11  /* WSEQ_ABORT */
#define CS47L24_WSEQ_ABORT_WIDTH                      1  /* WSEQ_ABORT */
#define CS47L24_WSEQ_START                       0x0400  /* WSEQ_START */
#define CS47L24_WSEQ_START_MASK                  0x0400  /* WSEQ_START */
#define CS47L24_WSEQ_START_SHIFT                     10  /* WSEQ_START */
#define CS47L24_WSEQ_START_WIDTH                      1  /* WSEQ_START */
#define CS47L24_WSEQ_ENA                         0x0200  /* WSEQ_ENA */
#define CS47L24_WSEQ_ENA_MASK                    0x0200  /* WSEQ_ENA */
#define CS47L24_WSEQ_ENA_SHIFT                        9  /* WSEQ_ENA */
#define CS47L24_WSEQ_ENA_WIDTH                        1  /* WSEQ_ENA */
#define CS47L24_WSEQ_START_INDEX_MASK            0x01FF  /* WSEQ_START_INDEX - [8:0] */
#define CS47L24_WSEQ_START_INDEX_SHIFT                0  /* WSEQ_START_INDEX - [8:0] */
#define CS47L24_WSEQ_START_INDEX_WIDTH                9  /* WSEQ_START_INDEX - [8:0] */

/*
 * R23 (0x17) - Write Sequencer Ctrl 1
 */
#define CS47L24_WSEQ_BUSY                        0x0200  /* WSEQ_BUSY */
#define CS47L24_WSEQ_BUSY_MASK                   0x0200  /* WSEQ_BUSY */
#define CS47L24_WSEQ_BUSY_SHIFT                       9  /* WSEQ_BUSY */
#define CS47L24_WSEQ_BUSY_WIDTH                       1  /* WSEQ_BUSY */
#define CS47L24_WSEQ_CURRENT_INDEX_MASK          0x01FF  /* WSEQ_CURRENT_INDEX - [8:0] */
#define CS47L24_WSEQ_CURRENT_INDEX_SHIFT              0  /* WSEQ_CURRENT_INDEX - [8:0] */
#define CS47L24_WSEQ_CURRENT_INDEX_WIDTH              9  /* WSEQ_CURRENT_INDEX - [8:0] */

/*
 * R24 (0x18) - Write Sequencer Ctrl 2
 */
#define CS47L24_LOAD_DEFAULTS                    0x0002  /* LOAD_DEFAULTS */
#define CS47L24_LOAD_DEFAULTS_MASK               0x0002  /* LOAD_DEFAULTS */
#define CS47L24_LOAD_DEFAULTS_SHIFT                   1  /* LOAD_DEFAULTS */
#define CS47L24_LOAD_DEFAULTS_WIDTH                   1  /* LOAD_DEFAULTS */
#define CS47L24_WSEQ_LOAD_MEM                    0x0001  /* WSEQ_LOAD_MEM */
#define CS47L24_WSEQ_LOAD_MEM_MASK               0x0001  /* WSEQ_LOAD_MEM */
#define CS47L24_WSEQ_LOAD_MEM_SHIFT                   0  /* WSEQ_LOAD_MEM */
#define CS47L24_WSEQ_LOAD_MEM_WIDTH                   1  /* WSEQ_LOAD_MEM */

/*
 * R32 (0x20) - Tone Generator 1
 */
#define CS47L24_TONE_RATE_MASK                   0x7800  /* TONE_RATE - [14:11] */
#define CS47L24_TONE_RATE_SHIFT                      11  /* TONE_RATE - [14:11] */
#define CS47L24_TONE_RATE_WIDTH                       4  /* TONE_RATE - [14:11] */
#define CS47L24_TONE_OFFSET_MASK                 0x0300  /* TONE_OFFSET - [9:8] */
#define CS47L24_TONE_OFFSET_SHIFT                     8  /* TONE_OFFSET - [9:8] */
#define CS47L24_TONE_OFFSET_WIDTH                     2  /* TONE_OFFSET - [9:8] */
#define CS47L24_TONE2_OVD                        0x0020  /* TONE2_OVD */
#define CS47L24_TONE2_OVD_MASK                   0x0020  /* TONE2_OVD */
#define CS47L24_TONE2_OVD_SHIFT                       5  /* TONE2_OVD */
#define CS47L24_TONE2_OVD_WIDTH                       1  /* TONE2_OVD */
#define CS47L24_TONE1_OVD                        0x0010  /* TONE1_OVD */
#define CS47L24_TONE1_OVD_MASK                   0x0010  /* TONE1_OVD */
#define CS47L24_TONE1_OVD_SHIFT                       4  /* TONE1_OVD */
#define CS47L24_TONE1_OVD_WIDTH                       1  /* TONE1_OVD */
#define CS47L24_TONE2_ENA                        0x0002  /* TONE2_ENA */
#define CS47L24_TONE2_ENA_MASK                   0x0002  /* TONE2_ENA */
#define CS47L24_TONE2_ENA_SHIFT                       1  /* TONE2_ENA */
#define CS47L24_TONE2_ENA_WIDTH                       1  /* TONE2_ENA */
#define CS47L24_TONE1_ENA                        0x0001  /* TONE1_ENA */
#define CS47L24_TONE1_ENA_MASK                   0x0001  /* TONE1_ENA */
#define CS47L24_TONE1_ENA_SHIFT                       0  /* TONE1_ENA */
#define CS47L24_TONE1_ENA_WIDTH                       1  /* TONE1_ENA */

/*
 * R33 (0x21) - Tone Generator 2
 */
#define CS47L24_TONE1_LVL_0_MASK                 0xFFFF  /* TONE1_LVL - [15:0] */
#define CS47L24_TONE1_LVL_0_SHIFT                     0  /* TONE1_LVL - [15:0] */
#define CS47L24_TONE1_LVL_0_WIDTH                    16  /* TONE1_LVL - [15:0] */

/*
 * R34 (0x22) - Tone Generator 3
 */
#define CS47L24_TONE1_LVL_MASK                   0x00FF  /* TONE1_LVL - [7:0] */
#define CS47L24_TONE1_LVL_SHIFT                       0  /* TONE1_LVL - [7:0] */
#define CS47L24_TONE1_LVL_WIDTH                       8  /* TONE1_LVL - [7:0] */

/*
 * R35 (0x23) - Tone Generator 4
 */
#define CS47L24_TONE2_LVL_0_MASK                 0xFFFF  /* TONE2_LVL - [15:0] */
#define CS47L24_TONE2_LVL_0_SHIFT                     0  /* TONE2_LVL - [15:0] */
#define CS47L24_TONE2_LVL_0_WIDTH                    16  /* TONE2_LVL - [15:0] */

/*
 * R36 (0x24) - Tone Generator 5
 */
#define CS47L24_TONE2_LVL_MASK                   0x00FF  /* TONE2_LVL - [7:0] */
#define CS47L24_TONE2_LVL_SHIFT                       0  /* TONE2_LVL - [7:0] */
#define CS47L24_TONE2_LVL_WIDTH                       8  /* TONE2_LVL - [7:0] */

/*
 * R48 (0x30) - PWM Drive 1
 */
#define CS47L24_PWM_RATE_MASK                    0x7800  /* PWM_RATE - [14:11] */
#define CS47L24_PWM_RATE_SHIFT                       11  /* PWM_RATE - [14:11] */
#define CS47L24_PWM_RATE_WIDTH                        4  /* PWM_RATE - [14:11] */
#define CS47L24_PWM_CLK_SEL_MASK                 0x0700  /* PWM_CLK_SEL - [10:8] */
#define CS47L24_PWM_CLK_SEL_SHIFT                     8  /* PWM_CLK_SEL - [10:8] */
#define CS47L24_PWM_CLK_SEL_WIDTH                     3  /* PWM_CLK_SEL - [10:8] */
#define CS47L24_PWM2_OVD                         0x0020  /* PWM2_OVD */
#define CS47L24_PWM2_OVD_MASK                    0x0020  /* PWM2_OVD */
#define CS47L24_PWM2_OVD_SHIFT                        5  /* PWM2_OVD */
#define CS47L24_PWM2_OVD_WIDTH                        1  /* PWM2_OVD */
#define CS47L24_PWM1_OVD                         0x0010  /* PWM1_OVD */
#define CS47L24_PWM1_OVD_MASK                    0x0010  /* PWM1_OVD */
#define CS47L24_PWM1_OVD_SHIFT                        4  /* PWM1_OVD */
#define CS47L24_PWM1_OVD_WIDTH                        1  /* PWM1_OVD */
#define CS47L24_PWM2_ENA                         0x0002  /* PWM2_ENA */
#define CS47L24_PWM2_ENA_MASK                    0x0002  /* PWM2_ENA */
#define CS47L24_PWM2_ENA_SHIFT                        1  /* PWM2_ENA */
#define CS47L24_PWM2_ENA_WIDTH                        1  /* PWM2_ENA */
#define CS47L24_PWM1_ENA                         0x0001  /* PWM1_ENA */
#define CS47L24_PWM1_ENA_MASK                    0x0001  /* PWM1_ENA */
#define CS47L24_PWM1_ENA_SHIFT                        0  /* PWM1_ENA */
#define CS47L24_PWM1_ENA_WIDTH                        1  /* PWM1_ENA */

/*
 * R49 (0x31) - PWM Drive 2
 */
#define CS47L24_PWM1_LVL_MASK                    0x03FF  /* PWM1_LVL - [9:0] */
#define CS47L24_PWM1_LVL_SHIFT                        0  /* PWM1_LVL - [9:0] */
#define CS47L24_PWM1_LVL_WIDTH                       10  /* PWM1_LVL - [9:0] */

/*
 * R50 (0x32) - PWM Drive 3
 */
#define CS47L24_PWM2_LVL_MASK                    0x03FF  /* PWM2_LVL - [9:0] */
#define CS47L24_PWM2_LVL_SHIFT                        0  /* PWM2_LVL - [9:0] */
#define CS47L24_PWM2_LVL_WIDTH                       10  /* PWM2_LVL - [9:0] */

/*
 * R65 (0x41) - Sequence control
 */
#define CS47L24_WSEQ_ENA_GP5_FALL                0x0020  /* WSEQ_ENA_GP5_FALL */
#define CS47L24_WSEQ_ENA_GP5_FALL_MASK           0x0020  /* WSEQ_ENA_GP5_FALL */
#define CS47L24_WSEQ_ENA_GP5_FALL_SHIFT               5  /* WSEQ_ENA_GP5_FALL */
#define CS47L24_WSEQ_ENA_GP5_FALL_WIDTH               1  /* WSEQ_ENA_GP5_FALL */
#define CS47L24_WSEQ_ENA_GP5_RISE                0x0010  /* WSEQ_ENA_GP5_RISE */
#define CS47L24_WSEQ_ENA_GP5_RISE_MASK           0x0010  /* WSEQ_ENA_GP5_RISE */
#define CS47L24_WSEQ_ENA_GP5_RISE_SHIFT               4  /* WSEQ_ENA_GP5_RISE */
#define CS47L24_WSEQ_ENA_GP5_RISE_WIDTH               1  /* WSEQ_ENA_GP5_RISE */
#define CS47L24_WSEQ_ENA_JD1_FALL                0x0008  /* WSEQ_ENA_JD1_FALL */
#define CS47L24_WSEQ_ENA_JD1_FALL_MASK           0x0008  /* WSEQ_ENA_JD1_FALL */
#define CS47L24_WSEQ_ENA_JD1_FALL_SHIFT               3  /* WSEQ_ENA_JD1_FALL */
#define CS47L24_WSEQ_ENA_JD1_FALL_WIDTH               1  /* WSEQ_ENA_JD1_FALL */
#define CS47L24_WSEQ_ENA_JD1_RISE                0x0004  /* WSEQ_ENA_JD1_RISE */
#define CS47L24_WSEQ_ENA_JD1_RISE_MASK           0x0004  /* WSEQ_ENA_JD1_RISE */
#define CS47L24_WSEQ_ENA_JD1_RISE_SHIFT               2  /* WSEQ_ENA_JD1_RISE */
#define CS47L24_WSEQ_ENA_JD1_RISE_WIDTH               1  /* WSEQ_ENA_JD1_RISE */
#define CS47L24_WSEQ_ENA_JD2_FALL                0x0002  /* WSEQ_ENA_JD2_FALL */
#define CS47L24_WSEQ_ENA_JD2_FALL_MASK           0x0002  /* WSEQ_ENA_JD2_FALL */
#define CS47L24_WSEQ_ENA_JD2_FALL_SHIFT               1  /* WSEQ_ENA_JD2_FALL */
#define CS47L24_WSEQ_ENA_JD2_FALL_WIDTH               1  /* WSEQ_ENA_JD2_FALL */
#define CS47L24_WSEQ_ENA_JD2_RISE                0x0001  /* WSEQ_ENA_JD2_RISE */
#define CS47L24_WSEQ_ENA_JD2_RISE_MASK           0x0001  /* WSEQ_ENA_JD2_RISE */
#define CS47L24_WSEQ_ENA_JD2_RISE_SHIFT               0  /* WSEQ_ENA_JD2_RISE */
#define CS47L24_WSEQ_ENA_JD2_RISE_WIDTH               1  /* WSEQ_ENA_JD2_RISE */

/*
 * R97 (0x61) - Sample Rate Sequence Select 1
 */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR_MASK 0x01FF  /* WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR_SHIFT      0  /* WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR_WIDTH      9  /* WSEQ_SAMPLE_RATE_DETECT_A_SEQ_ADDR - [8:0] */

/*
 * R98 (0x62) - Sample Rate Sequence Select 2
 */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR_MASK 0x01FF  /* WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR_SHIFT      0  /* WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR_WIDTH      9  /* WSEQ_SAMPLE_RATE_DETECT_B_SEQ_ADDR - [8:0] */

/*
 * R99 (0x63) - Sample Rate Sequence Select 3
 */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR_MASK 0x01FF  /* WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR_SHIFT      0  /* WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR_WIDTH      9  /* WSEQ_SAMPLE_RATE_DETECT_C_SEQ_ADDR - [8:0] */

/*
 * R100 (0x64) - Sample Rate Sequence Select 4
 */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR_MASK 0x01FF  /* WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR_SHIFT      0  /* WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR - [8:0] */
#define CS47L24_WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR_WIDTH      9  /* WSEQ_SAMPLE_RATE_DETECT_D_SEQ_ADDR - [8:0] */

/*
 * R112 (0x70) - Comfort Noise Generator
 */
#define CS47L24_NOISE_GEN_RATE_MASK              0x7800  /* NOISE_GEN_RATE - [14:11] */
#define CS47L24_NOISE_GEN_RATE_SHIFT                 11  /* NOISE_GEN_RATE - [14:11] */
#define CS47L24_NOISE_GEN_RATE_WIDTH                  4  /* NOISE_GEN_RATE - [14:11] */
#define CS47L24_NOISE_GEN_ENA                    0x0020  /* NOISE_GEN_ENA */
#define CS47L24_NOISE_GEN_ENA_MASK               0x0020  /* NOISE_GEN_ENA */
#define CS47L24_NOISE_GEN_ENA_SHIFT                   5  /* NOISE_GEN_ENA */
#define CS47L24_NOISE_GEN_ENA_WIDTH                   1  /* NOISE_GEN_ENA */
#define CS47L24_NOISE_GEN_GAIN_MASK              0x001F  /* NOISE_GEN_GAIN - [4:0] */
#define CS47L24_NOISE_GEN_GAIN_SHIFT                  0  /* NOISE_GEN_GAIN - [4:0] */
#define CS47L24_NOISE_GEN_GAIN_WIDTH                  5  /* NOISE_GEN_GAIN - [4:0] */

/*
 * R144 (0x90) - Haptics Control 1
 */
#define CS47L24_HAP_RATE_MASK                    0x7800  /* HAP_RATE - [14:11] */
#define CS47L24_HAP_RATE_SHIFT                       11  /* HAP_RATE - [14:11] */
#define CS47L24_HAP_RATE_WIDTH                        4  /* HAP_RATE - [14:11] */
#define CS47L24_ONESHOT_TRIG                     0x0010  /* ONESHOT_TRIG */
#define CS47L24_ONESHOT_TRIG_MASK                0x0010  /* ONESHOT_TRIG */
#define CS47L24_ONESHOT_TRIG_SHIFT                    4  /* ONESHOT_TRIG */
#define CS47L24_ONESHOT_TRIG_WIDTH                    1  /* ONESHOT_TRIG */
#define CS47L24_HAP_CTRL_MASK                    0x000C  /* HAP_CTRL - [3:2] */
#define CS47L24_HAP_CTRL_SHIFT                        2  /* HAP_CTRL - [3:2] */
#define CS47L24_HAP_CTRL_WIDTH                        2  /* HAP_CTRL - [3:2] */
#define CS47L24_HAP_ACT                          0x0002  /* HAP_ACT */
#define CS47L24_HAP_ACT_MASK                     0x0002  /* HAP_ACT */
#define CS47L24_HAP_ACT_SHIFT                         1  /* HAP_ACT */
#define CS47L24_HAP_ACT_WIDTH                         1  /* HAP_ACT */

/*
 * R145 (0x91) - Haptics Control 2
 */
#define CS47L24_LRA_FREQ_MASK                    0x7FFF  /* LRA_FREQ - [14:0] */
#define CS47L24_LRA_FREQ_SHIFT                        0  /* LRA_FREQ - [14:0] */
#define CS47L24_LRA_FREQ_WIDTH                       15  /* LRA_FREQ - [14:0] */

/*
 * R146 (0x92) - Haptics phase 1 intensity
 */
#define CS47L24_PHASE1_INTENSITY_MASK            0x00FF  /* PHASE1_INTENSITY - [7:0] */
#define CS47L24_PHASE1_INTENSITY_SHIFT                0  /* PHASE1_INTENSITY - [7:0] */
#define CS47L24_PHASE1_INTENSITY_WIDTH                8  /* PHASE1_INTENSITY - [7:0] */

/*
 * R147 (0x93) - Haptics phase 1 duration
 */
#define CS47L24_PHASE1_DURATION_MASK             0x01FF  /* PHASE1_DURATION - [8:0] */
#define CS47L24_PHASE1_DURATION_SHIFT                 0  /* PHASE1_DURATION - [8:0] */
#define CS47L24_PHASE1_DURATION_WIDTH                 9  /* PHASE1_DURATION - [8:0] */

/*
 * R148 (0x94) - Haptics phase 2 intensity
 */
#define CS47L24_PHASE2_INTENSITY_MASK            0x00FF  /* PHASE2_INTENSITY - [7:0] */
#define CS47L24_PHASE2_INTENSITY_SHIFT                0  /* PHASE2_INTENSITY - [7:0] */
#define CS47L24_PHASE2_INTENSITY_WIDTH                8  /* PHASE2_INTENSITY - [7:0] */

/*
 * R149 (0x95) - Haptics phase 2 duration
 */
#define CS47L24_PHASE2_DURATION_MASK             0x07FF  /* PHASE2_DURATION - [10:0] */
#define CS47L24_PHASE2_DURATION_SHIFT                 0  /* PHASE2_DURATION - [10:0] */
#define CS47L24_PHASE2_DURATION_WIDTH                11  /* PHASE2_DURATION - [10:0] */

/*
 * R150 (0x96) - Haptics phase 3 intensity
 */
#define CS47L24_PHASE3_INTENSITY_MASK            0x00FF  /* PHASE3_INTENSITY - [7:0] */
#define CS47L24_PHASE3_INTENSITY_SHIFT                0  /* PHASE3_INTENSITY - [7:0] */
#define CS47L24_PHASE3_INTENSITY_WIDTH                8  /* PHASE3_INTENSITY - [7:0] */

/*
 * R151 (0x97) - Haptics phase 3 duration
 */
#define CS47L24_PHASE3_DURATION_MASK             0x01FF  /* PHASE3_DURATION - [8:0] */
#define CS47L24_PHASE3_DURATION_SHIFT                 0  /* PHASE3_DURATION - [8:0] */
#define CS47L24_PHASE3_DURATION_WIDTH                 9  /* PHASE3_DURATION - [8:0] */

/*
 * R152 (0x98) - Haptics Status
 */
#define CS47L24_ONESHOT_STS                      0x0001  /* ONESHOT_STS */
#define CS47L24_ONESHOT_STS_MASK                 0x0001  /* ONESHOT_STS */
#define CS47L24_ONESHOT_STS_SHIFT                     0  /* ONESHOT_STS */
#define CS47L24_ONESHOT_STS_WIDTH                     1  /* ONESHOT_STS */

/*
 * R256 (0x100) - Clock 32k 1
 */
#define CS47L24_CLK_32K_ENA                      0x0040  /* CLK_32K_ENA */
#define CS47L24_CLK_32K_ENA_MASK                 0x0040  /* CLK_32K_ENA */
#define CS47L24_CLK_32K_ENA_SHIFT                     6  /* CLK_32K_ENA */
#define CS47L24_CLK_32K_ENA_WIDTH                     1  /* CLK_32K_ENA */
#define CS47L24_CLK_32K_SRC_MASK                 0x0003  /* CLK_32K_SRC - [1:0] */
#define CS47L24_CLK_32K_SRC_SHIFT                     0  /* CLK_32K_SRC - [1:0] */
#define CS47L24_CLK_32K_SRC_WIDTH                     2  /* CLK_32K_SRC - [1:0] */

/*
 * R257 (0x101) - System Clock 1
 */
#define CS47L24_SYSCLK_FRAC                      0x8000  /* SYSCLK_FRAC */
#define CS47L24_SYSCLK_FRAC_MASK                 0x8000  /* SYSCLK_FRAC */
#define CS47L24_SYSCLK_FRAC_SHIFT                    15  /* SYSCLK_FRAC */
#define CS47L24_SYSCLK_FRAC_WIDTH                     1  /* SYSCLK_FRAC */
#define CS47L24_SYSCLK_FREQ_MASK                 0x0700  /* SYSCLK_FREQ - [10:8] */
#define CS47L24_SYSCLK_FREQ_SHIFT                     8  /* SYSCLK_FREQ - [10:8] */
#define CS47L24_SYSCLK_FREQ_WIDTH                     3  /* SYSCLK_FREQ - [10:8] */
#define CS47L24_SYSCLK_ENA                       0x0040  /* SYSCLK_ENA */
#define CS47L24_SYSCLK_ENA_MASK                  0x0040  /* SYSCLK_ENA */
#define CS47L24_SYSCLK_ENA_SHIFT                      6  /* SYSCLK_ENA */
#define CS47L24_SYSCLK_ENA_WIDTH                      1  /* SYSCLK_ENA */
#define CS47L24_SYSCLK_SRC_MASK                  0x000F  /* SYSCLK_SRC - [3:0] */
#define CS47L24_SYSCLK_SRC_SHIFT                      0  /* SYSCLK_SRC - [3:0] */
#define CS47L24_SYSCLK_SRC_WIDTH                      4  /* SYSCLK_SRC - [3:0] */

/*
 * R258 (0x102) - Sample rate 1
 */
#define CS47L24_SAMPLE_RATE_1_MASK               0x001F  /* SAMPLE_RATE_1 - [4:0] */
#define CS47L24_SAMPLE_RATE_1_SHIFT                   0  /* SAMPLE_RATE_1 - [4:0] */
#define CS47L24_SAMPLE_RATE_1_WIDTH                   5  /* SAMPLE_RATE_1 - [4:0] */

/*
 * R259 (0x103) - Sample rate 2
 */
#define CS47L24_SAMPLE_RATE_2_MASK               0x001F  /* SAMPLE_RATE_2 - [4:0] */
#define CS47L24_SAMPLE_RATE_2_SHIFT                   0  /* SAMPLE_RATE_2 - [4:0] */
#define CS47L24_SAMPLE_RATE_2_WIDTH                   5  /* SAMPLE_RATE_2 - [4:0] */

/*
 * R260 (0x104) - Sample rate 3
 */
#define CS47L24_SAMPLE_RATE_3_MASK               0x001F  /* SAMPLE_RATE_3 - [4:0] */
#define CS47L24_SAMPLE_RATE_3_SHIFT                   0  /* SAMPLE_RATE_3 - [4:0] */
#define CS47L24_SAMPLE_RATE_3_WIDTH                   5  /* SAMPLE_RATE_3 - [4:0] */

/*
 * R266 (0x10A) - Sample rate 1 status
 */
#define CS47L24_SAMPLE_RATE_1_STS_MASK           0x001F  /* SAMPLE_RATE_1_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_1_STS_SHIFT               0  /* SAMPLE_RATE_1_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_1_STS_WIDTH               5  /* SAMPLE_RATE_1_STS - [4:0] */

/*
 * R267 (0x10B) - Sample rate 2 status
 */
#define CS47L24_SAMPLE_RATE_2_STS_MASK           0x001F  /* SAMPLE_RATE_2_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_2_STS_SHIFT               0  /* SAMPLE_RATE_2_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_2_STS_WIDTH               5  /* SAMPLE_RATE_2_STS - [4:0] */

/*
 * R268 (0x10C) - Sample rate 3 status
 */
#define CS47L24_SAMPLE_RATE_3_STS_MASK           0x001F  /* SAMPLE_RATE_3_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_3_STS_SHIFT               0  /* SAMPLE_RATE_3_STS - [4:0] */
#define CS47L24_SAMPLE_RATE_3_STS_WIDTH               5  /* SAMPLE_RATE_3_STS - [4:0] */

/*
 * R274 (0x112) - Async clock 1
 */
#define CS47L24_ASYNC_CLK_FREQ_MASK              0x0700  /* ASYNC_CLK_FREQ - [10:8] */
#define CS47L24_ASYNC_CLK_FREQ_SHIFT                  8  /* ASYNC_CLK_FREQ - [10:8] */
#define CS47L24_ASYNC_CLK_FREQ_WIDTH                  3  /* ASYNC_CLK_FREQ - [10:8] */
#define CS47L24_ASYNC_CLK_ENA                    0x0040  /* ASYNC_CLK_ENA */
#define CS47L24_ASYNC_CLK_ENA_MASK               0x0040  /* ASYNC_CLK_ENA */
#define CS47L24_ASYNC_CLK_ENA_SHIFT                   6  /* ASYNC_CLK_ENA */
#define CS47L24_ASYNC_CLK_ENA_WIDTH                   1  /* ASYNC_CLK_ENA */
#define CS47L24_ASYNC_CLK_SRC_MASK               0x000F  /* ASYNC_CLK_SRC - [3:0] */
#define CS47L24_ASYNC_CLK_SRC_SHIFT                   0  /* ASYNC_CLK_SRC - [3:0] */
#define CS47L24_ASYNC_CLK_SRC_WIDTH                   4  /* ASYNC_CLK_SRC - [3:0] */

/*
 * R275 (0x113) - Async sample rate 1
 */
#define CS47L24_ASYNC_SAMPLE_RATE_1_MASK         0x001F  /* ASYNC_SAMPLE_RATE_1 - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_1_SHIFT             0  /* ASYNC_SAMPLE_RATE_1 - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_1_WIDTH             5  /* ASYNC_SAMPLE_RATE_1 - [4:0] */

/*
 * R276 (0x114) - Async sample rate 2
 */
#define CS47L24_ASYNC_SAMPLE_RATE_2_MASK         0x001F  /* ASYNC_SAMPLE_RATE_2 - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_2_SHIFT             0  /* ASYNC_SAMPLE_RATE_2 - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_2_WIDTH             5  /* ASYNC_SAMPLE_RATE_2 - [4:0] */

/*
 * R283 (0x11B) - Async sample rate 1 status
 */
#define CS47L24_ASYNC_SAMPLE_RATE_1_STS_MASK     0x001F  /* ASYNC_SAMPLE_RATE_1_STS - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_1_STS_SHIFT         0  /* ASYNC_SAMPLE_RATE_1_STS - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_1_STS_WIDTH         5  /* ASYNC_SAMPLE_RATE_1_STS - [4:0] */

/*
 * R284 (0x11C) - Async sample rate 2 status
 */
#define CS47L24_ASYNC_SAMPLE_RATE_2_STS_MASK     0x001F  /* ASYNC_SAMPLE_RATE_2_STS - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_2_STS_SHIFT         0  /* ASYNC_SAMPLE_RATE_2_STS - [4:0] */
#define CS47L24_ASYNC_SAMPLE_RATE_2_STS_WIDTH         5  /* ASYNC_SAMPLE_RATE_2_STS - [4:0] */

/*
 * R329 (0x149) - Output system clock
 */
#define CS47L24_OPCLK_ENA                        0x8000  /* OPCLK_ENA */
#define CS47L24_OPCLK_ENA_MASK                   0x8000  /* OPCLK_ENA */
#define CS47L24_OPCLK_ENA_SHIFT                      15  /* OPCLK_ENA */
#define CS47L24_OPCLK_ENA_WIDTH                       1  /* OPCLK_ENA */
#define CS47L24_OPCLK_DIV_MASK                   0x00F8  /* OPCLK_DIV - [7:3] */
#define CS47L24_OPCLK_DIV_SHIFT                       3  /* OPCLK_DIV - [7:3] */
#define CS47L24_OPCLK_DIV_WIDTH                       5  /* OPCLK_DIV - [7:3] */
#define CS47L24_OPCLK_SEL_MASK                   0x0007  /* OPCLK_SEL - [2:0] */
#define CS47L24_OPCLK_SEL_SHIFT                       0  /* OPCLK_SEL - [2:0] */
#define CS47L24_OPCLK_SEL_WIDTH                       3  /* OPCLK_SEL - [2:0] */

/*
 * R330 (0x14A) - Output async clock
 */
#define CS47L24_OPCLK_ASYNC_ENA                  0x8000  /* OPCLK_ASYNC_ENA */
#define CS47L24_OPCLK_ASYNC_ENA_MASK             0x8000  /* OPCLK_ASYNC_ENA */
#define CS47L24_OPCLK_ASYNC_ENA_SHIFT                15  /* OPCLK_ASYNC_ENA */
#define CS47L24_OPCLK_ASYNC_ENA_WIDTH                 1  /* OPCLK_ASYNC_ENA */
#define CS47L24_OPCLK_ASYNC_DIV_MASK             0x00F8  /* OPCLK_ASYNC_DIV - [7:3] */
#define CS47L24_OPCLK_ASYNC_DIV_SHIFT                 3  /* OPCLK_ASYNC_DIV - [7:3] */
#define CS47L24_OPCLK_ASYNC_DIV_WIDTH                 5  /* OPCLK_ASYNC_DIV - [7:3] */
#define CS47L24_OPCLK_ASYNC_SEL_MASK             0x0007  /* OPCLK_ASYNC_SEL - [2:0] */
#define CS47L24_OPCLK_ASYNC_SEL_SHIFT                 0  /* OPCLK_ASYNC_SEL - [2:0] */
#define CS47L24_OPCLK_ASYNC_SEL_WIDTH                 3  /* OPCLK_ASYNC_SEL - [2:0] */

/*
 * R338 (0x152) - Rate Estimator 1
 */
#define CS47L24_TRIG_ON_STARTUP                  0x0010  /* TRIG_ON_STARTUP */
#define CS47L24_TRIG_ON_STARTUP_MASK             0x0010  /* TRIG_ON_STARTUP */
#define CS47L24_TRIG_ON_STARTUP_SHIFT                 4  /* TRIG_ON_STARTUP */
#define CS47L24_TRIG_ON_STARTUP_WIDTH                 1  /* TRIG_ON_STARTUP */
#define CS47L24_LRCLK_SRC_MASK                   0x000E  /* LRCLK_SRC - [3:1] */
#define CS47L24_LRCLK_SRC_SHIFT                       1  /* LRCLK_SRC - [3:1] */
#define CS47L24_LRCLK_SRC_WIDTH                       3  /* LRCLK_SRC - [3:1] */
#define CS47L24_RATE_EST_ENA                     0x0001  /* RATE_EST_ENA */
#define CS47L24_RATE_EST_ENA_MASK                0x0001  /* RATE_EST_ENA */
#define CS47L24_RATE_EST_ENA_SHIFT                    0  /* RATE_EST_ENA */
#define CS47L24_RATE_EST_ENA_WIDTH                    1  /* RATE_EST_ENA */

/*
 * R339 (0x153) - Rate Estimator 2
 */
#define CS47L24_SAMPLE_RATE_DETECT_A_MASK        0x001F  /* SAMPLE_RATE_DETECT_A - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_A_SHIFT            0  /* SAMPLE_RATE_DETECT_A - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_A_WIDTH            5  /* SAMPLE_RATE_DETECT_A - [4:0] */

/*
 * R340 (0x154) - Rate Estimator 3
 */
#define CS47L24_SAMPLE_RATE_DETECT_B_MASK        0x001F  /* SAMPLE_RATE_DETECT_B - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_B_SHIFT            0  /* SAMPLE_RATE_DETECT_B - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_B_WIDTH            5  /* SAMPLE_RATE_DETECT_B - [4:0] */

/*
 * R341 (0x155) - Rate Estimator 4
 */
#define CS47L24_SAMPLE_RATE_DETECT_C_MASK        0x001F  /* SAMPLE_RATE_DETECT_C - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_C_SHIFT            0  /* SAMPLE_RATE_DETECT_C - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_C_WIDTH            5  /* SAMPLE_RATE_DETECT_C - [4:0] */

/*
 * R342 (0x156) - Rate Estimator 5
 */
#define CS47L24_SAMPLE_RATE_DETECT_D_MASK        0x001F  /* SAMPLE_RATE_DETECT_D - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_D_SHIFT            0  /* SAMPLE_RATE_DETECT_D - [4:0] */
#define CS47L24_SAMPLE_RATE_DETECT_D_WIDTH            5  /* SAMPLE_RATE_DETECT_D - [4:0] */

/*
 * R369 (0x171) - FLL1 Control 1
 */
#define CS47L24_FLL1_FREERUN                     0x0002  /* FLL1_FREERUN */
#define CS47L24_FLL1_FREERUN_MASK                0x0002  /* FLL1_FREERUN */
#define CS47L24_FLL1_FREERUN_SHIFT                    1  /* FLL1_FREERUN */
#define CS47L24_FLL1_FREERUN_WIDTH                    1  /* FLL1_FREERUN */
#define CS47L24_FLL1_ENA                         0x0001  /* FLL1_ENA */
#define CS47L24_FLL1_ENA_MASK                    0x0001  /* FLL1_ENA */
#define CS47L24_FLL1_ENA_SHIFT                        0  /* FLL1_ENA */
#define CS47L24_FLL1_ENA_WIDTH                        1  /* FLL1_ENA */

/*
 * R370 (0x172) - FLL1 Control 2
 */
#define CS47L24_FLL1_CTRL_UPD                    0x8000  /* FLL1_CTRL_UPD */
#define CS47L24_FLL1_CTRL_UPD_MASK               0x8000  /* FLL1_CTRL_UPD */
#define CS47L24_FLL1_CTRL_UPD_SHIFT                  15  /* FLL1_CTRL_UPD */
#define CS47L24_FLL1_CTRL_UPD_WIDTH                   1  /* FLL1_CTRL_UPD */
#define CS47L24_FLL1_N_MASK                      0x03FF  /* FLL1_N - [9:0] */
#define CS47L24_FLL1_N_SHIFT                          0  /* FLL1_N - [9:0] */
#define CS47L24_FLL1_N_WIDTH                         10  /* FLL1_N - [9:0] */

/*
 * R371 (0x173) - FLL1 Control 3
 */
#define CS47L24_FLL1_THETA_MASK                  0xFFFF  /* FLL1_THETA - [15:0] */
#define CS47L24_FLL1_THETA_SHIFT                      0  /* FLL1_THETA - [15:0] */
#define CS47L24_FLL1_THETA_WIDTH                     16  /* FLL1_THETA - [15:0] */

/*
 * R372 (0x174) - FLL1 Control 4
 */
#define CS47L24_FLL1_LAMBDA_MASK                 0xFFFF  /* FLL1_LAMBDA - [15:0] */
#define CS47L24_FLL1_LAMBDA_SHIFT                     0  /* FLL1_LAMBDA - [15:0] */
#define CS47L24_FLL1_LAMBDA_WIDTH                    16  /* FLL1_LAMBDA - [15:0] */

/*
 * R373 (0x175) - FLL1 Control 5
 */
#define CS47L24_FLL1_FRATIO_MASK                 0x0F00  /* FLL1_FRATIO - [11:8] */
#define CS47L24_FLL1_FRATIO_SHIFT                     8  /* FLL1_FRATIO - [11:8] */
#define CS47L24_FLL1_FRATIO_WIDTH                     4  /* FLL1_FRATIO - [11:8] */
#define CS47L24_FLL1_OUTDIV_MASK                 0x000E  /* FLL1_OUTDIV - [3:1] */
#define CS47L24_FLL1_OUTDIV_SHIFT                     1  /* FLL1_OUTDIV - [3:1] */
#define CS47L24_FLL1_OUTDIV_WIDTH                     3  /* FLL1_OUTDIV - [3:1] */

/*
 * R374 (0x176) - FLL1 Control 6
 */
#define CS47L24_FLL1_CLK_REF_DIV_MASK            0x00C0  /* FLL1_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL1_CLK_REF_DIV_SHIFT                6  /* FLL1_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL1_CLK_REF_DIV_WIDTH                2  /* FLL1_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL1_CLK_REF_SRC_MASK            0x000F  /* FLL1_CLK_REF_SRC - [3:0] */
#define CS47L24_FLL1_CLK_REF_SRC_SHIFT                0  /* FLL1_CLK_REF_SRC - [3:0] */
#define CS47L24_FLL1_CLK_REF_SRC_WIDTH                4  /* FLL1_CLK_REF_SRC - [3:0] */

/*
 * R375 (0x177) - FLL1 Loop Filter Test 1
 */
#define CS47L24_FLL1_FRC_INTEG_UPD               0x8000  /* FLL1_FRC_INTEG_UPD */
#define CS47L24_FLL1_FRC_INTEG_UPD_MASK          0x8000  /* FLL1_FRC_INTEG_UPD */
#define CS47L24_FLL1_FRC_INTEG_UPD_SHIFT             15  /* FLL1_FRC_INTEG_UPD */
#define CS47L24_FLL1_FRC_INTEG_UPD_WIDTH              1  /* FLL1_FRC_INTEG_UPD */
#define CS47L24_FLL1_FRC_INTEG_VAL_MASK          0x0FFF  /* FLL1_FRC_INTEG_VAL - [11:0] */
#define CS47L24_FLL1_FRC_INTEG_VAL_SHIFT              0  /* FLL1_FRC_INTEG_VAL - [11:0] */
#define CS47L24_FLL1_FRC_INTEG_VAL_WIDTH             12  /* FLL1_FRC_INTEG_VAL - [11:0] */

/*
 * R377 (0x179) - FLL1 Control 7
 */
#define CS47L24_FLL1_GAIN_MASK                   0x003c  /* FLL1_GAIN */
#define CS47L24_FLL1_GAIN_SHIFT                       2  /* FLL1_GAIN */
#define CS47L24_FLL1_GAIN_WIDTH                       4  /* FLL1_GAIN */

/*
 * R378 (0x17A) - FLL1 EFS 2
 */
#define CS47L24_FLL1_PHASE_GAIN_MASK             0xF000  /* FLL1_PHASE_GAIN */
#define CS47L24_FLL1_PHASE_GAIN_SHIFT                12  /* FLL1_PHASE_GAIN */
#define CS47L24_FLL1_PHASE_GAIN_WIDTH                 4  /* FLL1_PHASE_GAIN */
#define CS47L24_FLL1_PHASE_ENA_MASK              0x0800  /* FLL1_PHASE_ENA */
#define CS47L24_FLL1_PHASE_ENA_SHIFT                 11  /* FLL1_PHASE_ENA */
#define CS47L24_FLL1_PHASE_ENA_WIDTH                  1  /* FLL1_PHASE_ENA */

/*
 * R385 (0x181) - FLL1 Synchroniser 1
 */
#define CS47L24_FLL1_SYNC_ENA                    0x0001  /* FLL1_SYNC_ENA */
#define CS47L24_FLL1_SYNC_ENA_MASK               0x0001  /* FLL1_SYNC_ENA */
#define CS47L24_FLL1_SYNC_ENA_SHIFT                   0  /* FLL1_SYNC_ENA */
#define CS47L24_FLL1_SYNC_ENA_WIDTH                   1  /* FLL1_SYNC_ENA */

/*
 * R386 (0x182) - FLL1 Synchroniser 2
 */
#define CS47L24_FLL1_SYNC_N_MASK                 0x03FF  /* FLL1_SYNC_N - [9:0] */
#define CS47L24_FLL1_SYNC_N_SHIFT                     0  /* FLL1_SYNC_N - [9:0] */
#define CS47L24_FLL1_SYNC_N_WIDTH                    10  /* FLL1_SYNC_N - [9:0] */

/*
 * R387 (0x183) - FLL1 Synchroniser 3
 */
#define CS47L24_FLL1_SYNC_THETA_MASK             0xFFFF  /* FLL1_SYNC_THETA - [15:0] */
#define CS47L24_FLL1_SYNC_THETA_SHIFT                 0  /* FLL1_SYNC_THETA - [15:0] */
#define CS47L24_FLL1_SYNC_THETA_WIDTH                16  /* FLL1_SYNC_THETA - [15:0] */

/*
 * R388 (0x184) - FLL1 Synchroniser 4
 */
#define CS47L24_FLL1_SYNC_LAMBDA_MASK            0xFFFF  /* FLL1_SYNC_LAMBDA - [15:0] */
#define CS47L24_FLL1_SYNC_LAMBDA_SHIFT                0  /* FLL1_SYNC_LAMBDA - [15:0] */
#define CS47L24_FLL1_SYNC_LAMBDA_WIDTH               16  /* FLL1_SYNC_LAMBDA - [15:0] */

/*
 * R389 (0x185) - FLL1 Synchroniser 5
 */
#define CS47L24_FLL1_SYNC_FRATIO_MASK            0x0700  /* FLL1_SYNC_FRATIO - [10:8] */
#define CS47L24_FLL1_SYNC_FRATIO_SHIFT                8  /* FLL1_SYNC_FRATIO - [10:8] */
#define CS47L24_FLL1_SYNC_FRATIO_WIDTH                3  /* FLL1_SYNC_FRATIO - [10:8] */

/*
 * R390 (0x186) - FLL1 Synchroniser 6
 */
#define CS47L24_FLL1_SYNC_CLK_DIV_MASK           0x00C0  /* FLL1_SYNC_CLK_DIV - [7:6] */
#define CS47L24_FLL1_SYNC_CLK_DIV_SHIFT               6  /* FLL1_SYNC_CLK_DIV - [7:6] */
#define CS47L24_FLL1_SYNC_CLK_DIV_WIDTH               2  /* FLL1_SYNC_CLK_DIV - [7:6] */
#define CS47L24_FLL1_SYNC_CLK_SRC_MASK           0x000F  /* FLL1_SYNC_CLK_SRC - [3:0] */
#define CS47L24_FLL1_SYNC_CLK_SRC_SHIFT               0  /* FLL1_SYNC_CLK_SRC - [3:0] */
#define CS47L24_FLL1_SYNC_CLK_SRC_WIDTH               4  /* FLL1_SYNC_CLK_SRC - [3:0] */

/*
 * R391 (0x187) - FLL1 Synchroniser 7
 */
#define CS47L24_FLL1_SYNC_GAIN_MASK              0x003c  /* FLL1_SYNC_GAIN */
#define CS47L24_FLL1_SYNC_GAIN_SHIFT                  2  /* FLL1_SYNC_GAIN */
#define CS47L24_FLL1_SYNC_GAIN_WIDTH                  4  /* FLL1_SYNC_GAIN */
#define CS47L24_FLL1_SYNC_BW                     0x0001  /* FLL1_SYNC_BW */
#define CS47L24_FLL1_SYNC_BW_MASK                0x0001  /* FLL1_SYNC_BW */
#define CS47L24_FLL1_SYNC_BW_SHIFT                    0  /* FLL1_SYNC_BW */
#define CS47L24_FLL1_SYNC_BW_WIDTH                    1  /* FLL1_SYNC_BW */

/*
 * R393 (0x189) - FLL1 Spread Spectrum
 */
#define CS47L24_FLL1_SS_AMPL_MASK                0x0030  /* FLL1_SS_AMPL - [5:4] */
#define CS47L24_FLL1_SS_AMPL_SHIFT                    4  /* FLL1_SS_AMPL - [5:4] */
#define CS47L24_FLL1_SS_AMPL_WIDTH                    2  /* FLL1_SS_AMPL - [5:4] */
#define CS47L24_FLL1_SS_FREQ_MASK                0x000C  /* FLL1_SS_FREQ - [3:2] */
#define CS47L24_FLL1_SS_FREQ_SHIFT                    2  /* FLL1_SS_FREQ - [3:2] */
#define CS47L24_FLL1_SS_FREQ_WIDTH                    2  /* FLL1_SS_FREQ - [3:2] */
#define CS47L24_FLL1_SS_SEL_MASK                 0x0003  /* FLL1_SS_SEL - [1:0] */
#define CS47L24_FLL1_SS_SEL_SHIFT                     0  /* FLL1_SS_SEL - [1:0] */
#define CS47L24_FLL1_SS_SEL_WIDTH                     2  /* FLL1_SS_SEL - [1:0] */

/*
 * R394 (0x18A) - FLL1 GPIO Clock
 */
#define CS47L24_FLL1_GPDIV_MASK                  0x00FE  /* FLL1_GPDIV - [7:1] */
#define CS47L24_FLL1_GPDIV_SHIFT                      1  /* FLL1_GPDIV - [7:1] */
#define CS47L24_FLL1_GPDIV_WIDTH                      7  /* FLL1_GPDIV - [7:1] */
#define CS47L24_FLL1_GPDIV_ENA                   0x0001  /* FLL1_GPDIV_ENA */
#define CS47L24_FLL1_GPDIV_ENA_MASK              0x0001  /* FLL1_GPDIV_ENA */
#define CS47L24_FLL1_GPDIV_ENA_SHIFT                  0  /* FLL1_GPDIV_ENA */
#define CS47L24_FLL1_GPDIV_ENA_WIDTH                  1  /* FLL1_GPDIV_ENA */

/*
 * R401 (0x191) - FLL2 Control 1
 */
#define CS47L24_FLL2_FREERUN                     0x0002  /* FLL2_FREERUN */
#define CS47L24_FLL2_FREERUN_MASK                0x0002  /* FLL2_FREERUN */
#define CS47L24_FLL2_FREERUN_SHIFT                    1  /* FLL2_FREERUN */
#define CS47L24_FLL2_FREERUN_WIDTH                    1  /* FLL2_FREERUN */
#define CS47L24_FLL2_ENA                         0x0001  /* FLL2_ENA */
#define CS47L24_FLL2_ENA_MASK                    0x0001  /* FLL2_ENA */
#define CS47L24_FLL2_ENA_SHIFT                        0  /* FLL2_ENA */
#define CS47L24_FLL2_ENA_WIDTH                        1  /* FLL2_ENA */

/*
 * R402 (0x192) - FLL2 Control 2
 */
#define CS47L24_FLL2_CTRL_UPD                    0x8000  /* FLL2_CTRL_UPD */
#define CS47L24_FLL2_CTRL_UPD_MASK               0x8000  /* FLL2_CTRL_UPD */
#define CS47L24_FLL2_CTRL_UPD_SHIFT                  15  /* FLL2_CTRL_UPD */
#define CS47L24_FLL2_CTRL_UPD_WIDTH                   1  /* FLL2_CTRL_UPD */
#define CS47L24_FLL2_N_MASK                      0x03FF  /* FLL2_N - [9:0] */
#define CS47L24_FLL2_N_SHIFT                          0  /* FLL2_N - [9:0] */
#define CS47L24_FLL2_N_WIDTH                         10  /* FLL2_N - [9:0] */

/*
 * R403 (0x193) - FLL2 Control 3
 */
#define CS47L24_FLL2_THETA_MASK                  0xFFFF  /* FLL2_THETA - [15:0] */
#define CS47L24_FLL2_THETA_SHIFT                      0  /* FLL2_THETA - [15:0] */
#define CS47L24_FLL2_THETA_WIDTH                     16  /* FLL2_THETA - [15:0] */

/*
 * R404 (0x194) - FLL2 Control 4
 */
#define CS47L24_FLL2_LAMBDA_MASK                 0xFFFF  /* FLL2_LAMBDA - [15:0] */
#define CS47L24_FLL2_LAMBDA_SHIFT                     0  /* FLL2_LAMBDA - [15:0] */
#define CS47L24_FLL2_LAMBDA_WIDTH                    16  /* FLL2_LAMBDA - [15:0] */

/*
 * R405 (0x195) - FLL2 Control 5
 */
#define CS47L24_FLL2_FRATIO_MASK                 0x0700  /* FLL2_FRATIO - [10:8] */
#define CS47L24_FLL2_FRATIO_SHIFT                     8  /* FLL2_FRATIO - [10:8] */
#define CS47L24_FLL2_FRATIO_WIDTH                     3  /* FLL2_FRATIO - [10:8] */
#define CS47L24_FLL2_OUTDIV_MASK                 0x000E  /* FLL2_OUTDIV - [3:1] */
#define CS47L24_FLL2_OUTDIV_SHIFT                     1  /* FLL2_OUTDIV - [3:1] */
#define CS47L24_FLL2_OUTDIV_WIDTH                     3  /* FLL2_OUTDIV - [3:1] */

/*
 * R406 (0x196) - FLL2 Control 6
 */
#define CS47L24_FLL2_CLK_REF_DIV_MASK            0x00C0  /* FLL2_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL2_CLK_REF_DIV_SHIFT                6  /* FLL2_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL2_CLK_REF_DIV_WIDTH                2  /* FLL2_CLK_REF_DIV - [7:6] */
#define CS47L24_FLL2_CLK_REF_SRC_MASK            0x000F  /* FLL2_CLK_REF_SRC - [3:0] */
#define CS47L24_FLL2_CLK_REF_SRC_SHIFT                0  /* FLL2_CLK_REF_SRC - [3:0] */
#define CS47L24_FLL2_CLK_REF_SRC_WIDTH                4  /* FLL2_CLK_REF_SRC - [3:0] */

/*
 * R407 (0x197) - FLL2 Loop Filter Test 1
 */
#define CS47L24_FLL2_FRC_INTEG_UPD               0x8000  /* FLL2_FRC_INTEG_UPD */
#define CS47L24_FLL2_FRC_INTEG_UPD_MASK          0x8000  /* FLL2_FRC_INTEG_UPD */
#define CS47L24_FLL2_FRC_INTEG_UPD_SHIFT             15  /* FLL2_FRC_INTEG_UPD */
#define CS47L24_FLL2_FRC_INTEG_UPD_WIDTH              1  /* FLL2_FRC_INTEG_UPD */
#define CS47L24_FLL2_FRC_INTEG_VAL_MASK          0x0FFF  /* FLL2_FRC_INTEG_VAL - [11:0] */
#define CS47L24_FLL2_FRC_INTEG_VAL_SHIFT              0  /* FLL2_FRC_INTEG_VAL - [11:0] */
#define CS47L24_FLL2_FRC_INTEG_VAL_WIDTH             12  /* FLL2_FRC_INTEG_VAL - [11:0] */

/*
 * R409 (0x199) - FLL2 Control 7
 */
#define CS47L24_FLL2_GAIN_MASK                   0x003c  /* FLL2_GAIN */
#define CS47L24_FLL2_GAIN_SHIFT                       2  /* FLL2_GAIN */
#define CS47L24_FLL2_GAIN_WIDTH                       4  /* FLL2_GAIN */

/*
 * R417 (0x1A1) - FLL2 Synchroniser 1
 */
#define CS47L24_FLL2_SYNC_ENA                    0x0001  /* FLL2_SYNC_ENA */
#define CS47L24_FLL2_SYNC_ENA_MASK               0x0001  /* FLL2_SYNC_ENA */
#define CS47L24_FLL2_SYNC_ENA_SHIFT                   0  /* FLL2_SYNC_ENA */
#define CS47L24_FLL2_SYNC_ENA_WIDTH                   1  /* FLL2_SYNC_ENA */

/*
 * R418 (0x1A2) - FLL2 Synchroniser 2
 */
#define CS47L24_FLL2_SYNC_N_MASK                 0x03FF  /* FLL2_SYNC_N - [9:0] */
#define CS47L24_FLL2_SYNC_N_SHIFT                     0  /* FLL2_SYNC_N - [9:0] */
#define CS47L24_FLL2_SYNC_N_WIDTH                    10  /* FLL2_SYNC_N - [9:0] */

/*
 * R419 (0x1A3) - FLL2 Synchroniser 3
 */
#define CS47L24_FLL2_SYNC_THETA_MASK             0xFFFF  /* FLL2_SYNC_THETA - [15:0] */
#define CS47L24_FLL2_SYNC_THETA_SHIFT                 0  /* FLL2_SYNC_THETA - [15:0] */
#define CS47L24_FLL2_SYNC_THETA_WIDTH                16  /* FLL2_SYNC_THETA - [15:0] */

/*
 * R420 (0x1A4) - FLL2 Synchroniser 4
 */
#define CS47L24_FLL2_SYNC_LAMBDA_MASK            0xFFFF  /* FLL2_SYNC_LAMBDA - [15:0] */
#define CS47L24_FLL2_SYNC_LAMBDA_SHIFT                0  /* FLL2_SYNC_LAMBDA - [15:0] */
#define CS47L24_FLL2_SYNC_LAMBDA_WIDTH               16  /* FLL2_SYNC_LAMBDA - [15:0] */

/*
 * R421 (0x1A5) - FLL2 Synchroniser 5
 */
#define CS47L24_FLL2_SYNC_FRATIO_MASK            0x0700  /* FLL2_SYNC_FRATIO - [10:8] */
#define CS47L24_FLL2_SYNC_FRATIO_SHIFT                8  /* FLL2_SYNC_FRATIO - [10:8] */
#define CS47L24_FLL2_SYNC_FRATIO_WIDTH                3  /* FLL2_SYNC_FRATIO - [10:8] */

/*
 * R422 (0x1A6) - FLL2 Synchroniser 6
 */
#define CS47L24_FLL2_CLK_SYNC_DIV_MASK           0x00C0  /* FLL2_CLK_SYNC_DIV - [7:6] */
#define CS47L24_FLL2_CLK_SYNC_DIV_SHIFT               6  /* FLL2_CLK_SYNC_DIV - [7:6] */
#define CS47L24_FLL2_CLK_SYNC_DIV_WIDTH               2  /* FLL2_CLK_SYNC_DIV - [7:6] */
#define CS47L24_FLL2_CLK_SYNC_SRC_MASK           0x000F  /* FLL2_CLK_SYNC_SRC - [3:0] */
#define CS47L24_FLL2_CLK_SYNC_SRC_SHIFT               0  /* FLL2_CLK_SYNC_SRC - [3:0] */
#define CS47L24_FLL2_CLK_SYNC_SRC_WIDTH               4  /* FLL2_CLK_SYNC_SRC - [3:0] */

/*
 * R423 (0x1A7) - FLL2 Synchroniser 7
 */
#define CS47L24_FLL2_SYNC_GAIN_MASK              0x003c  /* FLL2_SYNC_GAIN */
#define CS47L24_FLL2_SYNC_GAIN_SHIFT                  2  /* FLL2_SYNC_GAIN */
#define CS47L24_FLL2_SYNC_GAIN_WIDTH                  4  /* FLL2_SYNC_GAIN */
#define CS47L24_FLL2_SYNC_BW                     0x0001  /* FLL2_SYNC_BW */
#define CS47L24_FLL2_SYNC_BW_MASK                0x0001  /* FLL2_SYNC_BW */
#define CS47L24_FLL2_SYNC_BW_SHIFT                    0  /* FLL2_SYNC_BW */
#define CS47L24_FLL2_SYNC_BW_WIDTH                    1  /* FLL2_SYNC_BW */

/*
 * R425 (0x1A9) - FLL2 Spread Spectrum
 */
#define CS47L24_FLL2_SS_AMPL_MASK                0x0030  /* FLL2_SS_AMPL - [5:4] */
#define CS47L24_FLL2_SS_AMPL_SHIFT                    4  /* FLL2_SS_AMPL - [5:4] */
#define CS47L24_FLL2_SS_AMPL_WIDTH                    2  /* FLL2_SS_AMPL - [5:4] */
#define CS47L24_FLL2_SS_FREQ_MASK                0x000C  /* FLL2_SS_FREQ - [3:2] */
#define CS47L24_FLL2_SS_FREQ_SHIFT                    2  /* FLL2_SS_FREQ - [3:2] */
#define CS47L24_FLL2_SS_FREQ_WIDTH                    2  /* FLL2_SS_FREQ - [3:2] */
#define CS47L24_FLL2_SS_SEL_MASK                 0x0003  /* FLL2_SS_SEL - [1:0] */
#define CS47L24_FLL2_SS_SEL_SHIFT                     0  /* FLL2_SS_SEL - [1:0] */
#define CS47L24_FLL2_SS_SEL_WIDTH                     2  /* FLL2_SS_SEL - [1:0] */

/*
 * R426 (0x1AA) - FLL2 GPIO Clock
 */
#define CS47L24_FLL2_GPDIV_MASK                  0x00FE  /* FLL2_GPDIV - [7:1] */
#define CS47L24_FLL2_GPDIV_SHIFT                      1  /* FLL2_GPDIV - [7:1] */
#define CS47L24_FLL2_GPDIV_WIDTH                      7  /* FLL2_GPDIV - [7:1] */
#define CS47L24_FLL2_GPDIV_ENA                   0x0001  /* FLL2_GPDIV_ENA */
#define CS47L24_FLL2_GPDIV_ENA_MASK              0x0001  /* FLL2_GPDIV_ENA */
#define CS47L24_FLL2_GPDIV_ENA_SHIFT                  0  /* FLL2_GPDIV_ENA */
#define CS47L24_FLL2_GPDIV_ENA_WIDTH                  1  /* FLL2_GPDIV_ENA */

/*
 * R536 (0x218) - Mic Bias Ctrl 1
 */
#define CS47L24_MICB1_EXT_CAP                    0x8000  /* MICB1_EXT_CAP */
#define CS47L24_MICB1_EXT_CAP_MASK               0x8000  /* MICB1_EXT_CAP */
#define CS47L24_MICB1_EXT_CAP_SHIFT                  15  /* MICB1_EXT_CAP */
#define CS47L24_MICB1_EXT_CAP_WIDTH                   1  /* MICB1_EXT_CAP */
#define CS47L24_MICB1_LVL_MASK                   0x01E0  /* MICB1_LVL - [8:5] */
#define CS47L24_MICB1_LVL_SHIFT                       5  /* MICB1_LVL - [8:5] */
#define CS47L24_MICB1_LVL_WIDTH                       4  /* MICB1_LVL - [8:5] */
#define CS47L24_MICB1_FAST                       0x0010  /* MICB1_FAST */
#define CS47L24_MICB1_FAST_MASK                  0x0010  /* MICB1_FAST */
#define CS47L24_MICB1_FAST_SHIFT                      4  /* MICB1_FAST */
#define CS47L24_MICB1_FAST_WIDTH                      1  /* MICB1_FAST */
#define CS47L24_MICB1_RATE                       0x0008  /* MICB1_RATE */
#define CS47L24_MICB1_RATE_MASK                  0x0008  /* MICB1_RATE */
#define CS47L24_MICB1_RATE_SHIFT                      3  /* MICB1_RATE */
#define CS47L24_MICB1_RATE_WIDTH                      1  /* MICB1_RATE */
#define CS47L24_MICB1_DISCH                      0x0004  /* MICB1_DISCH */
#define CS47L24_MICB1_DISCH_MASK                 0x0004  /* MICB1_DISCH */
#define CS47L24_MICB1_DISCH_SHIFT                     2  /* MICB1_DISCH */
#define CS47L24_MICB1_DISCH_WIDTH                     1  /* MICB1_DISCH */
#define CS47L24_MICB1_BYPASS                     0x0002  /* MICB1_BYPASS */
#define CS47L24_MICB1_BYPASS_MASK                0x0002  /* MICB1_BYPASS */
#define CS47L24_MICB1_BYPASS_SHIFT                    1  /* MICB1_BYPASS */
#define CS47L24_MICB1_BYPASS_WIDTH                    1  /* MICB1_BYPASS */
#define CS47L24_MICB1_ENA                        0x0001  /* MICB1_ENA */
#define CS47L24_MICB1_ENA_MASK                   0x0001  /* MICB1_ENA */
#define CS47L24_MICB1_ENA_SHIFT                       0  /* MICB1_ENA */
#define CS47L24_MICB1_ENA_WIDTH                       1  /* MICB1_ENA */

/*
 * R537 (0x219) - Mic Bias Ctrl 2
 */
#define CS47L24_MICB2_EXT_CAP                    0x8000  /* MICB2_EXT_CAP */
#define CS47L24_MICB2_EXT_CAP_MASK               0x8000  /* MICB2_EXT_CAP */
#define CS47L24_MICB2_EXT_CAP_SHIFT                  15  /* MICB2_EXT_CAP */
#define CS47L24_MICB2_EXT_CAP_WIDTH                   1  /* MICB2_EXT_CAP */
#define CS47L24_MICB2_LVL_MASK                   0x01E0  /* MICB2_LVL - [8:5] */
#define CS47L24_MICB2_LVL_SHIFT                       5  /* MICB2_LVL - [8:5] */
#define CS47L24_MICB2_LVL_WIDTH                       4  /* MICB2_LVL - [8:5] */
#define CS47L24_MICB2_FAST                       0x0010  /* MICB2_FAST */
#define CS47L24_MICB2_FAST_MASK                  0x0010  /* MICB2_FAST */
#define CS47L24_MICB2_FAST_SHIFT                      4  /* MICB2_FAST */
#define CS47L24_MICB2_FAST_WIDTH                      1  /* MICB2_FAST */
#define CS47L24_MICB2_RATE                       0x0008  /* MICB2_RATE */
#define CS47L24_MICB2_RATE_MASK                  0x0008  /* MICB2_RATE */
#define CS47L24_MICB2_RATE_SHIFT                      3  /* MICB2_RATE */
#define CS47L24_MICB2_RATE_WIDTH                      1  /* MICB2_RATE */
#define CS47L24_MICB2_DISCH                      0x0004  /* MICB2_DISCH */
#define CS47L24_MICB2_DISCH_MASK                 0x0004  /* MICB2_DISCH */
#define CS47L24_MICB2_DISCH_SHIFT                     2  /* MICB2_DISCH */
#define CS47L24_MICB2_DISCH_WIDTH                     1  /* MICB2_DISCH */
#define CS47L24_MICB2_BYPASS                     0x0002  /* MICB2_BYPASS */
#define CS47L24_MICB2_BYPASS_MASK                0x0002  /* MICB2_BYPASS */
#define CS47L24_MICB2_BYPASS_SHIFT                    1  /* MICB2_BYPASS */
#define CS47L24_MICB2_BYPASS_WIDTH                    1  /* MICB2_BYPASS */
#define CS47L24_MICB2_ENA                        0x0001  /* MICB2_ENA */
#define CS47L24_MICB2_ENA_MASK                   0x0001  /* MICB2_ENA */
#define CS47L24_MICB2_ENA_SHIFT                       0  /* MICB2_ENA */
#define CS47L24_MICB2_ENA_WIDTH                       1  /* MICB2_ENA */

/*
 * R768 (0x300) - Input Enables
 */
#define CS47L24_IN2L_ENA                         0x0008  /* IN2L_ENA */
#define CS47L24_IN2L_ENA_MASK                    0x0008  /* IN2L_ENA */
#define CS47L24_IN2L_ENA_SHIFT                        3  /* IN2L_ENA */
#define CS47L24_IN2L_ENA_WIDTH                        1  /* IN2L_ENA */
#define CS47L24_IN2R_ENA                         0x0004  /* IN2R_ENA */
#define CS47L24_IN2R_ENA_MASK                    0x0004  /* IN2R_ENA */
#define CS47L24_IN2R_ENA_SHIFT                        2  /* IN2R_ENA */
#define CS47L24_IN2R_ENA_WIDTH                        1  /* IN2R_ENA */
#define CS47L24_IN1L_ENA                         0x0002  /* IN1L_ENA */
#define CS47L24_IN1L_ENA_MASK                    0x0002  /* IN1L_ENA */
#define CS47L24_IN1L_ENA_SHIFT                        1  /* IN1L_ENA */
#define CS47L24_IN1L_ENA_WIDTH                        1  /* IN1L_ENA */
#define CS47L24_IN1R_ENA                         0x0001  /* IN1R_ENA */
#define CS47L24_IN1R_ENA_MASK                    0x0001  /* IN1R_ENA */
#define CS47L24_IN1R_ENA_SHIFT                        0  /* IN1R_ENA */
#define CS47L24_IN1R_ENA_WIDTH                        1  /* IN1R_ENA */

/*
 * R776 (0x308) - Input Rate
 */
#define CS47L24_IN_RATE_MASK                     0x7800  /* IN_RATE - [14:11] */
#define CS47L24_IN_RATE_SHIFT                        11  /* IN_RATE - [14:11] */
#define CS47L24_IN_RATE_WIDTH                         4  /* IN_RATE - [14:11] */

/*
 * R777 (0x309) - Input Volume Ramp
 */
#define CS47L24_IN_VD_RAMP_MASK                  0x0070  /* IN_VD_RAMP - [6:4] */
#define CS47L24_IN_VD_RAMP_SHIFT                      4  /* IN_VD_RAMP - [6:4] */
#define CS47L24_IN_VD_RAMP_WIDTH                      3  /* IN_VD_RAMP - [6:4] */
#define CS47L24_IN_VI_RAMP_MASK                  0x0007  /* IN_VI_RAMP - [2:0] */
#define CS47L24_IN_VI_RAMP_SHIFT                      0  /* IN_VI_RAMP - [2:0] */
#define CS47L24_IN_VI_RAMP_WIDTH                      3  /* IN_VI_RAMP - [2:0] */

/*
 * R780 (0x30C) - HPF Control
 */
#define CS47L24_IN_HPF_CUT_MASK                  0x0007  /* IN_HPF_CUT [2:0] */
#define CS47L24_IN_HPF_CUT_SHIFT                      0  /* IN_HPF_CUT [2:0] */
#define CS47L24_IN_HPF_CUT_WIDTH                      3  /* IN_HPF_CUT [2:0] */

/*
 * R784 (0x310) - IN1L Control
 */
#define CS47L24_IN1L_HPF_ENA                     0x8000  /* IN1L_HPF - [15] */
#define CS47L24_IN1L_HPF_MASK                    0x8000  /* IN1L_HPF - [15] */
#define CS47L24_IN1L_HPF_SHIFT                       15  /* IN1L_HPF - [15] */
#define CS47L24_IN1L_HPF_WIDTH                        1  /* IN1L_HPF - [15] */
#define CS47L24_IN1_OSR_MASK                     0x6000  /* IN1_OSR - [14:13] */
#define CS47L24_IN1_OSR_SHIFT                        13  /* IN1_OSR - [14:13] */
#define CS47L24_IN1_OSR_WIDTH                         2  /* IN1_OSR - [14:13] */
#define CS47L24_IN1_DMIC_SUP_MASK                0x1800  /* IN1_DMIC_SUP - [12:11] */
#define CS47L24_IN1_DMIC_SUP_SHIFT                   11  /* IN1_DMIC_SUP - [12:11] */
#define CS47L24_IN1_DMIC_SUP_WIDTH                    2  /* IN1_DMIC_SUP - [12:11] */

/*
 * R785 (0x311) - ADC Digital Volume 1L
 */
#define CS47L24_IN_VU                            0x0200  /* IN_VU */
#define CS47L24_IN_VU_MASK                       0x0200  /* IN_VU */
#define CS47L24_IN_VU_SHIFT                           9  /* IN_VU */
#define CS47L24_IN_VU_WIDTH                           1  /* IN_VU */
#define CS47L24_IN1L_MUTE                        0x0100  /* IN1L_MUTE */
#define CS47L24_IN1L_MUTE_MASK                   0x0100  /* IN1L_MUTE */
#define CS47L24_IN1L_MUTE_SHIFT                       8  /* IN1L_MUTE */
#define CS47L24_IN1L_MUTE_WIDTH                       1  /* IN1L_MUTE */
#define CS47L24_IN1L_DIG_VOL_MASK                0x00FF  /* IN1L_DIG_VOL - [7:0] */
#define CS47L24_IN1L_DIG_VOL_SHIFT                    0  /* IN1L_DIG_VOL - [7:0] */
#define CS47L24_IN1L_DIG_VOL_WIDTH                    8  /* IN1L_DIG_VOL - [7:0] */

/*
 * R786 (0x312) - DMIC1L Control
 */
#define CS47L24_IN1_DMICL_DLY_MASK               0x003F  /* IN1_DMICL_DLY - [5:0] */
#define CS47L24_IN1_DMICL_DLY_SHIFT                   0  /* IN1_DMICL_DLY - [5:0] */
#define CS47L24_IN1_DMICL_DLY_WIDTH                   6  /* IN1_DMICL_DLY - [5:0] */

/*
 * R788 (0x314) - IN1R Control
 */
#define CS47L24_IN1R_HPF_ENA                     0x8000  /* IN1R_HPF - [15] */
#define CS47L24_IN1R_HPF_MASK                    0x8000  /* IN1R_HPF - [15] */
#define CS47L24_IN1R_HPF_SHIFT                       15  /* IN1R_HPF - [15] */
#define CS47L24_IN1R_HPF_WIDTH                        1  /* IN1R_HPF - [15] */

/*
 * R789 (0x315) - ADC Digital Volume 1R
 */
#define CS47L24_IN_VU                            0x0200  /* IN_VU */
#define CS47L24_IN_VU_MASK                       0x0200  /* IN_VU */
#define CS47L24_IN_VU_SHIFT                           9  /* IN_VU */
#define CS47L24_IN_VU_WIDTH                           1  /* IN_VU */
#define CS47L24_IN1R_MUTE                        0x0100  /* IN1R_MUTE */
#define CS47L24_IN1R_MUTE_MASK                   0x0100  /* IN1R_MUTE */
#define CS47L24_IN1R_MUTE_SHIFT                       8  /* IN1R_MUTE */
#define CS47L24_IN1R_MUTE_WIDTH                       1  /* IN1R_MUTE */
#define CS47L24_IN1R_DIG_VOL_MASK                0x00FF  /* IN1R_DIG_VOL - [7:0] */
#define CS47L24_IN1R_DIG_VOL_SHIFT                    0  /* IN1R_DIG_VOL - [7:0] */
#define CS47L24_IN1R_DIG_VOL_WIDTH                    8  /* IN1R_DIG_VOL - [7:0] */

/*
 * R790 (0x316) - DMIC1R Control
 */
#define CS47L24_IN1_DMICR_DLY_MASK               0x003F  /* IN1_DMICR_DLY - [5:0] */
#define CS47L24_IN1_DMICR_DLY_SHIFT                   0  /* IN1_DMICR_DLY - [5:0] */
#define CS47L24_IN1_DMICR_DLY_WIDTH                   6  /* IN1_DMICR_DLY - [5:0] */

/*
 * R792 (0x318) - IN2L Control
 */
#define CS47L24_IN2L_HPF_ENA                     0x8000  /* IN2L_HPF - [15] */
#define CS47L24_IN2L_HPF_MASK                    0x8000  /* IN2L_HPF - [15] */
#define CS47L24_IN2L_HPF_SHIFT                       15  /* IN2L_HPF - [15] */
#define CS47L24_IN2L_HPF_WIDTH                        1  /* IN2L_HPF - [15] */
#define CS47L24_IN2_OSR_MASK                     0x6000  /* IN2_OSR - [14:13] */
#define CS47L24_IN2_OSR_SHIFT                        13  /* IN2_OSR - [14:13] */
#define CS47L24_IN2_OSR_WIDTH                         2  /* IN2_OSR - [14:13] */
#define CS47L24_IN2_DMIC_SUP_MASK                0x1800  /* IN2_DMIC_SUP - [12:11] */
#define CS47L24_IN2_DMIC_SUP_SHIFT                   11  /* IN2_DMIC_SUP - [12:11] */
#define CS47L24_IN2_DMIC_SUP_WIDTH                    2  /* IN2_DMIC_SUP - [12:11] */

/*
 * R793 (0x319) - ADC Digital Volume 2L
 */
#define CS47L24_IN_VU                            0x0200  /* IN_VU */
#define CS47L24_IN_VU_MASK                       0x0200  /* IN_VU */
#define CS47L24_IN_VU_SHIFT                           9  /* IN_VU */
#define CS47L24_IN_VU_WIDTH                           1  /* IN_VU */
#define CS47L24_IN2L_MUTE                        0x0100  /* IN2L_MUTE */
#define CS47L24_IN2L_MUTE_MASK                   0x0100  /* IN2L_MUTE */
#define CS47L24_IN2L_MUTE_SHIFT                       8  /* IN2L_MUTE */
#define CS47L24_IN2L_MUTE_WIDTH                       1  /* IN2L_MUTE */
#define CS47L24_IN2L_DIG_VOL_MASK                0x00FF  /* IN2L_DIG_VOL - [7:0] */
#define CS47L24_IN2L_DIG_VOL_SHIFT                    0  /* IN2L_DIG_VOL - [7:0] */
#define CS47L24_IN2L_DIG_VOL_WIDTH                    8  /* IN2L_DIG_VOL - [7:0] */

/*
 * R794 (0x31A) - DMIC2L Control
 */
#define CS47L24_IN2_DMICL_DLY_MASK               0x003F  /* IN2_DMICL_DLY - [5:0] */
#define CS47L24_IN2_DMICL_DLY_SHIFT                   0  /* IN2_DMICL_DLY - [5:0] */
#define CS47L24_IN2_DMICL_DLY_WIDTH                   6  /* IN2_DMICL_DLY - [5:0] */

/*
 * R796 (0x31C) - IN2R Control
 */
#define CS47L24_IN2R_HPF_ENA                     0x8000  /* IN2R_HPF - [15] */
#define CS47L24_IN2R_HPF_MASK                    0x8000  /* IN2R_HPF - [15] */
#define CS47L24_IN2R_HPF_SHIFT                       15  /* IN2R_HPF - [15] */
#define CS47L24_IN2R_HPF_WIDTH                        1  /* IN2R_HPF - [15] */

/*
 * R797 (0x31D) - ADC Digital Volume 2R
 */
#define CS47L24_IN_VU                            0x0200  /* IN_VU */
#define CS47L24_IN_VU_MASK                       0x0200  /* IN_VU */
#define CS47L24_IN_VU_SHIFT                           9  /* IN_VU */
#define CS47L24_IN_VU_WIDTH                           1  /* IN_VU */
#define CS47L24_IN2R_MUTE                        0x0100  /* IN2R_MUTE */
#define CS47L24_IN2R_MUTE_MASK                   0x0100  /* IN2R_MUTE */
#define CS47L24_IN2R_MUTE_SHIFT                       8  /* IN2R_MUTE */
#define CS47L24_IN2R_MUTE_WIDTH                       1  /* IN2R_MUTE */
#define CS47L24_IN2R_DIG_VOL_MASK                0x00FF  /* IN2R_DIG_VOL - [7:0] */
#define CS47L24_IN2R_DIG_VOL_SHIFT                    0  /* IN2R_DIG_VOL - [7:0] */
#define CS47L24_IN2R_DIG_VOL_WIDTH                    8  /* IN2R_DIG_VOL - [7:0] */

/*
 * R798 (0x31E) - DMIC2R Control
 */
#define CS47L24_IN2_DMICR_DLY_MASK               0x003F  /* IN2_DMICR_DLY - [5:0] */
#define CS47L24_IN2_DMICR_DLY_SHIFT                   0  /* IN2_DMICR_DLY - [5:0] */
#define CS47L24_IN2_DMICR_DLY_WIDTH                   6  /* IN2_DMICR_DLY - [5:0] */

/*
 * R1024 (0x400) - Output Enables 1
 */
#define CS47L24_OUT4L_ENA                        0x0080  /* OUT4L_ENA */
#define CS47L24_OUT4L_ENA_MASK                   0x0080  /* OUT4L_ENA */
#define CS47L24_OUT4L_ENA_SHIFT                       7  /* OUT4L_ENA */
#define CS47L24_OUT4L_ENA_WIDTH                       1  /* OUT4L_ENA */
#define CS47L24_OUT1L_ENA                        0x0002  /* OUT1L_ENA */
#define CS47L24_OUT1L_ENA_MASK                   0x0002  /* OUT1L_ENA */
#define CS47L24_OUT1L_ENA_SHIFT                       1  /* OUT1L_ENA */
#define CS47L24_OUT1L_ENA_WIDTH                       1  /* OUT1L_ENA */
#define CS47L24_OUT1R_ENA                        0x0001  /* OUT1R_ENA */
#define CS47L24_OUT1R_ENA_MASK                   0x0001  /* OUT1R_ENA */
#define CS47L24_OUT1R_ENA_SHIFT                       0  /* OUT1R_ENA */
#define CS47L24_OUT1R_ENA_WIDTH                       1  /* OUT1R_ENA */

/*
 * R1025 (0x401) - Output Status 1
 */
#define CS47L24_OUT4L_ENA_STS                    0x0080  /* OUT4L_ENA_STS */
#define CS47L24_OUT4L_ENA_STS_MASK               0x0080  /* OUT4L_ENA_STS */
#define CS47L24_OUT4L_ENA_STS_SHIFT                   7  /* OUT4L_ENA_STS */
#define CS47L24_OUT4L_ENA_STS_WIDTH                   1  /* OUT4L_ENA_STS */

/*
 * R1030 (0x406) - Raw Output Status 1
 */
#define CS47L24_OUT1L_ENA_STS                    0x0002  /* OUT1L_ENA_STS */
#define CS47L24_OUT1L_ENA_STS_MASK               0x0002  /* OUT1L_ENA_STS */
#define CS47L24_OUT1L_ENA_STS_SHIFT                   1  /* OUT1L_ENA_STS */
#define CS47L24_OUT1L_ENA_STS_WIDTH                   1  /* OUT1L_ENA_STS */
#define CS47L24_OUT1R_ENA_STS                    0x0001  /* OUT1R_ENA_STS */
#define CS47L24_OUT1R_ENA_STS_MASK               0x0001  /* OUT1R_ENA_STS */
#define CS47L24_OUT1R_ENA_STS_SHIFT                   0  /* OUT1R_ENA_STS */
#define CS47L24_OUT1R_ENA_STS_WIDTH                   1  /* OUT1R_ENA_STS */

/*
 * R1032 (0x408) - Output Rate 1
 */
#define CS47L24_OUT_RATE_MASK                    0x7800  /* OUT_RATE - [14:11] */
#define CS47L24_OUT_RATE_SHIFT                       11  /* OUT_RATE - [14:11] */
#define CS47L24_OUT_RATE_WIDTH                        4  /* OUT_RATE - [14:11] */

/*
 * R1033 (0x409) - Output Volume Ramp
 */
#define CS47L24_OUT_VD_RAMP_MASK                 0x0070  /* OUT_VD_RAMP - [6:4] */
#define CS47L24_OUT_VD_RAMP_SHIFT                     4  /* OUT_VD_RAMP - [6:4] */
#define CS47L24_OUT_VD_RAMP_WIDTH                     3  /* OUT_VD_RAMP - [6:4] */
#define CS47L24_OUT_VI_RAMP_MASK                 0x0007  /* OUT_VI_RAMP - [2:0] */
#define CS47L24_OUT_VI_RAMP_SHIFT                     0  /* OUT_VI_RAMP - [2:0] */
#define CS47L24_OUT_VI_RAMP_WIDTH                     3  /* OUT_VI_RAMP - [2:0] */

/*
 * R1040 (0x410) - Output Path Config 1L
 */
#define CS47L24_OUT1_LP_MODE                     0x8000  /* OUT1_LP_MODE */
#define CS47L24_OUT1_LP_MODE_MASK                0x8000  /* OUT1_LP_MODE */
#define CS47L24_OUT1_LP_MODE_SHIFT                   15  /* OUT1_LP_MODE */
#define CS47L24_OUT1_LP_MODE_WIDTH                    1  /* OUT1_LP_MODE */
#define CS47L24_OUT1_OSR                         0x2000  /* OUT1_OSR */
#define CS47L24_OUT1_OSR_MASK                    0x2000  /* OUT1_OSR */
#define CS47L24_OUT1_OSR_SHIFT                       13  /* OUT1_OSR */
#define CS47L24_OUT1_OSR_WIDTH                        1  /* OUT1_OSR */
#define CS47L24_OUT1_MONO                        0x1000  /* OUT1_MONO */
#define CS47L24_OUT1_MONO_MASK                   0x1000  /* OUT1_MONO */
#define CS47L24_OUT1_MONO_SHIFT                      12  /* OUT1_MONO */
#define CS47L24_OUT1_MONO_WIDTH                       1  /* OUT1_MONO */
#define CS47L24_OUT1L_ANC_SRC_MASK               0x0C00  /* OUT1L_ANC_SRC - [11:10] */
#define CS47L24_OUT1L_ANC_SRC_SHIFT                  10  /* OUT1L_ANC_SRC - [11:10] */
#define CS47L24_OUT1L_ANC_SRC_WIDTH                   2  /* OUT1L_ANC_SRC - [11:10] */
#define CS47L24_OUT1L_PGA_VOL_MASK               0x00FE  /* OUT1L_PGA_VOL - [7:1] */
#define CS47L24_OUT1L_PGA_VOL_SHIFT                   1  /* OUT1L_PGA_VOL - [7:1] */
#define CS47L24_OUT1L_PGA_VOL_WIDTH                   7  /* OUT1L_PGA_VOL - [7:1] */

/*
 * R1041 (0x411) - DAC Digital Volume 1L
 */
#define CS47L24_OUT_VU                           0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_MASK                      0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_SHIFT                          9  /* OUT_VU */
#define CS47L24_OUT_VU_WIDTH                          1  /* OUT_VU */
#define CS47L24_OUT1L_MUTE                       0x0100  /* OUT1L_MUTE */
#define CS47L24_OUT1L_MUTE_MASK                  0x0100  /* OUT1L_MUTE */
#define CS47L24_OUT1L_MUTE_SHIFT                      8  /* OUT1L_MUTE */
#define CS47L24_OUT1L_MUTE_WIDTH                      1  /* OUT1L_MUTE */
#define CS47L24_OUT1L_VOL_MASK                   0x00FF  /* OUT1L_VOL - [7:0] */
#define CS47L24_OUT1L_VOL_SHIFT                       0  /* OUT1L_VOL - [7:0] */
#define CS47L24_OUT1L_VOL_WIDTH                       8  /* OUT1L_VOL - [7:0] */

/*
 * R1042 (0x412) - DAC Volume Limit 1L
 */
#define CS47L24_OUT1L_VOL_LIM_MASK               0x00FF  /* OUT1L_VOL_LIM - [7:0] */
#define CS47L24_OUT1L_VOL_LIM_SHIFT                   0  /* OUT1L_VOL_LIM - [7:0] */
#define CS47L24_OUT1L_VOL_LIM_WIDTH                   8  /* OUT1L_VOL_LIM - [7:0] */

/*
 * R1043 (0x413) - Noise Gate Select 1L
 */
#define CS47L24_OUT1L_NGATE_SRC_MASK             0x0FFF  /* OUT1L_NGATE_SRC - [11:0] */
#define CS47L24_OUT1L_NGATE_SRC_SHIFT                 0  /* OUT1L_NGATE_SRC - [11:0] */
#define CS47L24_OUT1L_NGATE_SRC_WIDTH                12  /* OUT1L_NGATE_SRC - [11:0] */

/*
 * R1044 (0x414) - Output Path Config 1R
 */
#define CS47L24_OUT1R_ANC_SRC_MASK               0x0C00  /* OUT1R_ANC_SRC - [11:10] */
#define CS47L24_OUT1R_ANC_SRC_SHIFT                  10  /* OUT1R_ANC_SRC - [11:10] */
#define CS47L24_OUT1R_ANC_SRC_WIDTH                   2  /* OUT1R_ANC_SRC - [11:10] */
#define CS47L24_OUT1R_PGA_VOL_MASK               0x00FE  /* OUT1R_PGA_VOL - [7:1] */
#define CS47L24_OUT1R_PGA_VOL_SHIFT                   1  /* OUT1R_PGA_VOL - [7:1] */
#define CS47L24_OUT1R_PGA_VOL_WIDTH                   7  /* OUT1R_PGA_VOL - [7:1] */

/*
 * R1045 (0x415) - DAC Digital Volume 1R
 */
#define CS47L24_OUT_VU                           0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_MASK                      0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_SHIFT                          9  /* OUT_VU */
#define CS47L24_OUT_VU_WIDTH                          1  /* OUT_VU */
#define CS47L24_OUT1R_MUTE                       0x0100  /* OUT1R_MUTE */
#define CS47L24_OUT1R_MUTE_MASK                  0x0100  /* OUT1R_MUTE */
#define CS47L24_OUT1R_MUTE_SHIFT                      8  /* OUT1R_MUTE */
#define CS47L24_OUT1R_MUTE_WIDTH                      1  /* OUT1R_MUTE */
#define CS47L24_OUT1R_VOL_MASK                   0x00FF  /* OUT1R_VOL - [7:0] */
#define CS47L24_OUT1R_VOL_SHIFT                       0  /* OUT1R_VOL - [7:0] */
#define CS47L24_OUT1R_VOL_WIDTH                       8  /* OUT1R_VOL - [7:0] */

/*
 * R1046 (0x416) - DAC Volume Limit 1R
 */
#define CS47L24_OUT1R_VOL_LIM_MASK               0x00FF  /* OUT1R_VOL_LIM - [7:0] */
#define CS47L24_OUT1R_VOL_LIM_SHIFT                   0  /* OUT1R_VOL_LIM - [7:0] */
#define CS47L24_OUT1R_VOL_LIM_WIDTH                   8  /* OUT1R_VOL_LIM - [7:0] */

/*
 * R1047 (0x417) - Noise Gate Select 1R
 */
#define CS47L24_OUT1R_NGATE_SRC_MASK             0x0FFF  /* OUT1R_NGATE_SRC - [11:0] */
#define CS47L24_OUT1R_NGATE_SRC_SHIFT                 0  /* OUT1R_NGATE_SRC - [11:0] */
#define CS47L24_OUT1R_NGATE_SRC_WIDTH                12  /* OUT1R_NGATE_SRC - [11:0] */

/*
 * R1065 (0x429) - DAC Digital Volume 4L
 */
#define CS47L24_OUT_VU                           0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_MASK                      0x0200  /* OUT_VU */
#define CS47L24_OUT_VU_SHIFT                          9  /* OUT_VU */
#define CS47L24_OUT_VU_WIDTH                          1  /* OUT_VU */
#define CS47L24_OUT4L_MUTE                       0x0100  /* OUT4L_MUTE */
#define CS47L24_OUT4L_MUTE_MASK                  0x0100  /* OUT4L_MUTE */
#define CS47L24_OUT4L_MUTE_SHIFT                      8  /* OUT4L_MUTE */
#define CS47L24_OUT4L_MUTE_WIDTH                      1  /* OUT4L_MUTE */
#define CS47L24_OUT4L_VOL_MASK                   0x00FF  /* OUT4L_VOL - [7:0] */
#define CS47L24_OUT4L_VOL_SHIFT                       0  /* OUT4L_VOL - [7:0] */
#define CS47L24_OUT4L_VOL_WIDTH                       8  /* OUT4L_VOL - [7:0] */

/*
 * R1066 (0x42A) - Out Volume 4L
 */
#define CS47L24_OUT4L_VOL_LIM_MASK               0x00FF  /* OUT4L_VOL_LIM - [7:0] */
#define CS47L24_OUT4L_VOL_LIM_SHIFT                   0  /* OUT4L_VOL_LIM - [7:0] */
#define CS47L24_OUT4L_VOL_LIM_WIDTH                   8  /* OUT4L_VOL_LIM - [7:0] */

/*
 * R1067 (0x42B) - Noise Gate Select 4L
 */
#define CS47L24_OUT4L_NGATE_SRC_MASK             0x0FFF  /* OUT4L_NGATE_SRC - [11:0] */
#define CS47L24_OUT4L_NGATE_SRC_SHIFT                 0  /* OUT4L_NGATE_SRC - [11:0] */
#define CS47L24_OUT4L_NGATE_SRC_WIDTH                12  /* OUT4L_NGATE_SRC - [11:0] */

/*
 * R1104 (0x450) - DAC AEC Control 1
 */
#define CS47L24_AEC_LOOPBACK_SRC_MASK            0x003C  /* AEC_LOOPBACK_SRC - [5:2] */
#define CS47L24_AEC_LOOPBACK_SRC_SHIFT                2  /* AEC_LOOPBACK_SRC - [5:2] */
#define CS47L24_AEC_LOOPBACK_SRC_WIDTH                4  /* AEC_LOOPBACK_SRC - [5:2] */
#define CS47L24_AEC_ENA_STS                      0x0002  /* AEC_ENA_STS */
#define CS47L24_AEC_ENA_STS_MASK                 0x0002  /* AEC_ENA_STS */
#define CS47L24_AEC_ENA_STS_SHIFT                     1  /* AEC_ENA_STS */
#define CS47L24_AEC_ENA_STS_WIDTH                     1  /* AEC_ENA_STS */
#define CS47L24_AEC_LOOPBACK_ENA                 0x0001  /* AEC_LOOPBACK_ENA */
#define CS47L24_AEC_LOOPBACK_ENA_MASK            0x0001  /* AEC_LOOPBACK_ENA */
#define CS47L24_AEC_LOOPBACK_ENA_SHIFT                0  /* AEC_LOOPBACK_ENA */
#define CS47L24_AEC_LOOPBACK_ENA_WIDTH                1  /* AEC_LOOPBACK_ENA */

/*
 * R1112 (0x458) - Noise Gate Control
 */
#define CS47L24_NGATE_HOLD_MASK                  0x0030  /* NGATE_HOLD - [5:4] */
#define CS47L24_NGATE_HOLD_SHIFT                      4  /* NGATE_HOLD - [5:4] */
#define CS47L24_NGATE_HOLD_WIDTH                      2  /* NGATE_HOLD - [5:4] */
#define CS47L24_NGATE_THR_MASK                   0x000E  /* NGATE_THR - [3:1] */
#define CS47L24_NGATE_THR_SHIFT                       1  /* NGATE_THR - [3:1] */
#define CS47L24_NGATE_THR_WIDTH                       3  /* NGATE_THR - [3:1] */
#define CS47L24_NGATE_ENA                        0x0001  /* NGATE_ENA */
#define CS47L24_NGATE_ENA_MASK                   0x0001  /* NGATE_ENA */
#define CS47L24_NGATE_ENA_SHIFT                       0  /* NGATE_ENA */
#define CS47L24_NGATE_ENA_WIDTH                       1  /* NGATE_ENA */

/*
 * R1280 (0x500) - AIF1 BCLK Ctrl
 */
#define CS47L24_AIF1_BCLK_INV                    0x0080  /* AIF1_BCLK_INV */
#define CS47L24_AIF1_BCLK_INV_MASK               0x0080  /* AIF1_BCLK_INV */
#define CS47L24_AIF1_BCLK_INV_SHIFT                   7  /* AIF1_BCLK_INV */
#define CS47L24_AIF1_BCLK_INV_WIDTH                   1  /* AIF1_BCLK_INV */
#define CS47L24_AIF1_BCLK_FRC                    0x0040  /* AIF1_BCLK_FRC */
#define CS47L24_AIF1_BCLK_FRC_MASK               0x0040  /* AIF1_BCLK_FRC */
#define CS47L24_AIF1_BCLK_FRC_SHIFT                   6  /* AIF1_BCLK_FRC */
#define CS47L24_AIF1_BCLK_FRC_WIDTH                   1  /* AIF1_BCLK_FRC */
#define CS47L24_AIF1_BCLK_MSTR                   0x0020  /* AIF1_BCLK_MSTR */
#define CS47L24_AIF1_BCLK_MSTR_MASK              0x0020  /* AIF1_BCLK_MSTR */
#define CS47L24_AIF1_BCLK_MSTR_SHIFT                  5  /* AIF1_BCLK_MSTR */
#define CS47L24_AIF1_BCLK_MSTR_WIDTH                  1  /* AIF1_BCLK_MSTR */
#define CS47L24_AIF1_BCLK_FREQ_MASK              0x001F  /* AIF1_BCLK_FREQ - [4:0] */
#define CS47L24_AIF1_BCLK_FREQ_SHIFT                  0  /* AIF1_BCLK_FREQ - [4:0] */
#define CS47L24_AIF1_BCLK_FREQ_WIDTH                  5  /* AIF1_BCLK_FREQ - [4:0] */

/*
 * R1281 (0x501) - AIF1 Tx Pin Ctrl
 */
#define CS47L24_AIF1TX_DAT_TRI                   0x0020  /* AIF1TX_DAT_TRI */
#define CS47L24_AIF1TX_DAT_TRI_MASK              0x0020  /* AIF1TX_DAT_TRI */
#define CS47L24_AIF1TX_DAT_TRI_SHIFT                  5  /* AIF1TX_DAT_TRI */
#define CS47L24_AIF1TX_DAT_TRI_WIDTH                  1  /* AIF1TX_DAT_TRI */
#define CS47L24_AIF1TX_LRCLK_SRC                 0x0008  /* AIF1TX_LRCLK_SRC */
#define CS47L24_AIF1TX_LRCLK_SRC_MASK            0x0008  /* AIF1TX_LRCLK_SRC */
#define CS47L24_AIF1TX_LRCLK_SRC_SHIFT                3  /* AIF1TX_LRCLK_SRC */
#define CS47L24_AIF1TX_LRCLK_SRC_WIDTH                1  /* AIF1TX_LRCLK_SRC */
#define CS47L24_AIF1TX_LRCLK_INV                 0x0004  /* AIF1TX_LRCLK_INV */
#define CS47L24_AIF1TX_LRCLK_INV_MASK            0x0004  /* AIF1TX_LRCLK_INV */
#define CS47L24_AIF1TX_LRCLK_INV_SHIFT                2  /* AIF1TX_LRCLK_INV */
#define CS47L24_AIF1TX_LRCLK_INV_WIDTH                1  /* AIF1TX_LRCLK_INV */
#define CS47L24_AIF1TX_LRCLK_FRC                 0x0002  /* AIF1TX_LRCLK_FRC */
#define CS47L24_AIF1TX_LRCLK_FRC_MASK            0x0002  /* AIF1TX_LRCLK_FRC */
#define CS47L24_AIF1TX_LRCLK_FRC_SHIFT                1  /* AIF1TX_LRCLK_FRC */
#define CS47L24_AIF1TX_LRCLK_FRC_WIDTH                1  /* AIF1TX_LRCLK_FRC */
#define CS47L24_AIF1TX_LRCLK_MSTR                0x0001  /* AIF1TX_LRCLK_MSTR */
#define CS47L24_AIF1TX_LRCLK_MSTR_MASK           0x0001  /* AIF1TX_LRCLK_MSTR */
#define CS47L24_AIF1TX_LRCLK_MSTR_SHIFT               0  /* AIF1TX_LRCLK_MSTR */
#define CS47L24_AIF1TX_LRCLK_MSTR_WIDTH               1  /* AIF1TX_LRCLK_MSTR */

/*
 * R1282 (0x502) - AIF1 Rx Pin Ctrl
 */
#define CS47L24_AIF1RX_LRCLK_INV                 0x0004  /* AIF1RX_LRCLK_INV */
#define CS47L24_AIF1RX_LRCLK_INV_MASK            0x0004  /* AIF1RX_LRCLK_INV */
#define CS47L24_AIF1RX_LRCLK_INV_SHIFT                2  /* AIF1RX_LRCLK_INV */
#define CS47L24_AIF1RX_LRCLK_INV_WIDTH                1  /* AIF1RX_LRCLK_INV */
#define CS47L24_AIF1RX_LRCLK_FRC                 0x0002  /* AIF1RX_LRCLK_FRC */
#define CS47L24_AIF1RX_LRCLK_FRC_MASK            0x0002  /* AIF1RX_LRCLK_FRC */
#define CS47L24_AIF1RX_LRCLK_FRC_SHIFT                1  /* AIF1RX_LRCLK_FRC */
#define CS47L24_AIF1RX_LRCLK_FRC_WIDTH                1  /* AIF1RX_LRCLK_FRC */
#define CS47L24_AIF1RX_LRCLK_MSTR                0x0001  /* AIF1RX_LRCLK_MSTR */
#define CS47L24_AIF1RX_LRCLK_MSTR_MASK           0x0001  /* AIF1RX_LRCLK_MSTR */
#define CS47L24_AIF1RX_LRCLK_MSTR_SHIFT               0  /* AIF1RX_LRCLK_MSTR */
#define CS47L24_AIF1RX_LRCLK_MSTR_WIDTH               1  /* AIF1RX_LRCLK_MSTR */

/*
 * R1283 (0x503) - AIF1 Rate Ctrl
 */
#define CS47L24_AIF1_RATE_MASK                   0x7800  /* AIF1_RATE - [14:11] */
#define CS47L24_AIF1_RATE_SHIFT                      11  /* AIF1_RATE - [14:11] */
#define CS47L24_AIF1_RATE_WIDTH                       4  /* AIF1_RATE - [14:11] */
#define CS47L24_AIF1_TRI                         0x0040  /* AIF1_TRI */
#define CS47L24_AIF1_TRI_MASK                    0x0040  /* AIF1_TRI */
#define CS47L24_AIF1_TRI_SHIFT                        6  /* AIF1_TRI */
#define CS47L24_AIF1_TRI_WIDTH                        1  /* AIF1_TRI */

/*
 * R1284 (0x504) - AIF1 Format
 */
#define CS47L24_AIF1_FMT_MASK                    0x0007  /* AIF1_FMT - [2:0] */
#define CS47L24_AIF1_FMT_SHIFT                        0  /* AIF1_FMT - [2:0] */
#define CS47L24_AIF1_FMT_WIDTH                        3  /* AIF1_FMT - [2:0] */

/*
 * R1285 (0x505) - AIF1 Tx BCLK Rate
 */
#define CS47L24_AIF1TX_BCPF_MASK                 0x1FFF  /* AIF1TX_BCPF - [12:0] */
#define CS47L24_AIF1TX_BCPF_SHIFT                     0  /* AIF1TX_BCPF - [12:0] */
#define CS47L24_AIF1TX_BCPF_WIDTH                    13  /* AIF1TX_BCPF - [12:0] */

/*
 * R1286 (0x506) - AIF1 Rx BCLK Rate
 */
#define CS47L24_AIF1RX_BCPF_MASK                 0x1FFF  /* AIF1RX_BCPF - [12:0] */
#define CS47L24_AIF1RX_BCPF_SHIFT                     0  /* AIF1RX_BCPF - [12:0] */
#define CS47L24_AIF1RX_BCPF_WIDTH                    13  /* AIF1RX_BCPF - [12:0] */

/*
 * R1287 (0x507) - AIF1 Frame Ctrl 1
 */
#define CS47L24_AIF1TX_WL_MASK                   0x3F00  /* AIF1TX_WL - [13:8] */
#define CS47L24_AIF1TX_WL_SHIFT                       8  /* AIF1TX_WL - [13:8] */
#define CS47L24_AIF1TX_WL_WIDTH                       6  /* AIF1TX_WL - [13:8] */
#define CS47L24_AIF1TX_SLOT_LEN_MASK             0x00FF  /* AIF1TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF1TX_SLOT_LEN_SHIFT                 0  /* AIF1TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF1TX_SLOT_LEN_WIDTH                 8  /* AIF1TX_SLOT_LEN - [7:0] */

/*
 * R1288 (0x508) - AIF1 Frame Ctrl 2
 */
#define CS47L24_AIF1RX_WL_MASK                   0x3F00  /* AIF1RX_WL - [13:8] */
#define CS47L24_AIF1RX_WL_SHIFT                       8  /* AIF1RX_WL - [13:8] */
#define CS47L24_AIF1RX_WL_WIDTH                       6  /* AIF1RX_WL - [13:8] */
#define CS47L24_AIF1RX_SLOT_LEN_MASK             0x00FF  /* AIF1RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF1RX_SLOT_LEN_SHIFT                 0  /* AIF1RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF1RX_SLOT_LEN_WIDTH                 8  /* AIF1RX_SLOT_LEN - [7:0] */

/*
 * R1289 (0x509) - AIF1 Frame Ctrl 3
 */
#define CS47L24_AIF1TX1_SLOT_MASK                0x003F  /* AIF1TX1_SLOT - [5:0] */
#define CS47L24_AIF1TX1_SLOT_SHIFT                    0  /* AIF1TX1_SLOT - [5:0] */
#define CS47L24_AIF1TX1_SLOT_WIDTH                    6  /* AIF1TX1_SLOT - [5:0] */

/*
 * R1290 (0x50A) - AIF1 Frame Ctrl 4
 */
#define CS47L24_AIF1TX2_SLOT_MASK                0x003F  /* AIF1TX2_SLOT - [5:0] */
#define CS47L24_AIF1TX2_SLOT_SHIFT                    0  /* AIF1TX2_SLOT - [5:0] */
#define CS47L24_AIF1TX2_SLOT_WIDTH                    6  /* AIF1TX2_SLOT - [5:0] */

/*
 * R1291 (0x50B) - AIF1 Frame Ctrl 5
 */
#define CS47L24_AIF1TX3_SLOT_MASK                0x003F  /* AIF1TX3_SLOT - [5:0] */
#define CS47L24_AIF1TX3_SLOT_SHIFT                    0  /* AIF1TX3_SLOT - [5:0] */
#define CS47L24_AIF1TX3_SLOT_WIDTH                    6  /* AIF1TX3_SLOT - [5:0] */

/*
 * R1292 (0x50C) - AIF1 Frame Ctrl 6
 */
#define CS47L24_AIF1TX4_SLOT_MASK                0x003F  /* AIF1TX4_SLOT - [5:0] */
#define CS47L24_AIF1TX4_SLOT_SHIFT                    0  /* AIF1TX4_SLOT - [5:0] */
#define CS47L24_AIF1TX4_SLOT_WIDTH                    6  /* AIF1TX4_SLOT - [5:0] */

/*
 * R1293 (0x50D) - AIF1 Frame Ctrl 7
 */
#define CS47L24_AIF1TX5_SLOT_MASK                0x003F  /* AIF1TX5_SLOT - [5:0] */
#define CS47L24_AIF1TX5_SLOT_SHIFT                    0  /* AIF1TX5_SLOT - [5:0] */
#define CS47L24_AIF1TX5_SLOT_WIDTH                    6  /* AIF1TX5_SLOT - [5:0] */

/*
 * R1294 (0x50E) - AIF1 Frame Ctrl 8
 */
#define CS47L24_AIF1TX6_SLOT_MASK                0x003F  /* AIF1TX6_SLOT - [5:0] */
#define CS47L24_AIF1TX6_SLOT_SHIFT                    0  /* AIF1TX6_SLOT - [5:0] */
#define CS47L24_AIF1TX6_SLOT_WIDTH                    6  /* AIF1TX6_SLOT - [5:0] */

/*
 * R1295 (0x50F) - AIF1 Frame Ctrl 9
 */
#define CS47L24_AIF1TX7_SLOT_MASK                0x003F  /* AIF1TX7_SLOT - [5:0] */
#define CS47L24_AIF1TX7_SLOT_SHIFT                    0  /* AIF1TX7_SLOT - [5:0] */
#define CS47L24_AIF1TX7_SLOT_WIDTH                    6  /* AIF1TX7_SLOT - [5:0] */

/*
 * R1296 (0x510) - AIF1 Frame Ctrl 10
 */
#define CS47L24_AIF1TX8_SLOT_MASK                0x003F  /* AIF1TX8_SLOT - [5:0] */
#define CS47L24_AIF1TX8_SLOT_SHIFT                    0  /* AIF1TX8_SLOT - [5:0] */
#define CS47L24_AIF1TX8_SLOT_WIDTH                    6  /* AIF1TX8_SLOT - [5:0] */

/*
 * R1297 (0x511) - AIF1 Frame Ctrl 11
 */
#define CS47L24_AIF1RX1_SLOT_MASK                0x003F  /* AIF1RX1_SLOT - [5:0] */
#define CS47L24_AIF1RX1_SLOT_SHIFT                    0  /* AIF1RX1_SLOT - [5:0] */
#define CS47L24_AIF1RX1_SLOT_WIDTH                    6  /* AIF1RX1_SLOT - [5:0] */

/*
 * R1298 (0x512) - AIF1 Frame Ctrl 12
 */
#define CS47L24_AIF1RX2_SLOT_MASK                0x003F  /* AIF1RX2_SLOT - [5:0] */
#define CS47L24_AIF1RX2_SLOT_SHIFT                    0  /* AIF1RX2_SLOT - [5:0] */
#define CS47L24_AIF1RX2_SLOT_WIDTH                    6  /* AIF1RX2_SLOT - [5:0] */

/*
 * R1299 (0x513) - AIF1 Frame Ctrl 13
 */
#define CS47L24_AIF1RX3_SLOT_MASK                0x003F  /* AIF1RX3_SLOT - [5:0] */
#define CS47L24_AIF1RX3_SLOT_SHIFT                    0  /* AIF1RX3_SLOT - [5:0] */
#define CS47L24_AIF1RX3_SLOT_WIDTH                    6  /* AIF1RX3_SLOT - [5:0] */

/*
 * R1300 (0x514) - AIF1 Frame Ctrl 14
 */
#define CS47L24_AIF1RX4_SLOT_MASK                0x003F  /* AIF1RX4_SLOT - [5:0] */
#define CS47L24_AIF1RX4_SLOT_SHIFT                    0  /* AIF1RX4_SLOT - [5:0] */
#define CS47L24_AIF1RX4_SLOT_WIDTH                    6  /* AIF1RX4_SLOT - [5:0] */

/*
 * R1301 (0x515) - AIF1 Frame Ctrl 15
 */
#define CS47L24_AIF1RX5_SLOT_MASK                0x003F  /* AIF1RX5_SLOT - [5:0] */
#define CS47L24_AIF1RX5_SLOT_SHIFT                    0  /* AIF1RX5_SLOT - [5:0] */
#define CS47L24_AIF1RX5_SLOT_WIDTH                    6  /* AIF1RX5_SLOT - [5:0] */

/*
 * R1302 (0x516) - AIF1 Frame Ctrl 16
 */
#define CS47L24_AIF1RX6_SLOT_MASK                0x003F  /* AIF1RX6_SLOT - [5:0] */
#define CS47L24_AIF1RX6_SLOT_SHIFT                    0  /* AIF1RX6_SLOT - [5:0] */
#define CS47L24_AIF1RX6_SLOT_WIDTH                    6  /* AIF1RX6_SLOT - [5:0] */

/*
 * R1303 (0x517) - AIF1 Frame Ctrl 17
 */
#define CS47L24_AIF1RX7_SLOT_MASK                0x003F  /* AIF1RX7_SLOT - [5:0] */
#define CS47L24_AIF1RX7_SLOT_SHIFT                    0  /* AIF1RX7_SLOT - [5:0] */
#define CS47L24_AIF1RX7_SLOT_WIDTH                    6  /* AIF1RX7_SLOT - [5:0] */

/*
 * R1304 (0x518) - AIF1 Frame Ctrl 18
 */
#define CS47L24_AIF1RX8_SLOT_MASK                0x003F  /* AIF1RX8_SLOT - [5:0] */
#define CS47L24_AIF1RX8_SLOT_SHIFT                    0  /* AIF1RX8_SLOT - [5:0] */
#define CS47L24_AIF1RX8_SLOT_WIDTH                    6  /* AIF1RX8_SLOT - [5:0] */

/*
 * R1305 (0x519) - AIF1 Tx Enables
 */
#define CS47L24_AIF1TX_ALL_ENA                   0x00FF
#define CS47L24_AIF1TX_ALL_ENA_MASK              0x00FF
#define CS47L24_AIF1TX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF1TX_ALL_ENA_WIDTH                  8
#define CS47L24_AIF1TX8_ENA                      0x0080  /* AIF1TX8_ENA */
#define CS47L24_AIF1TX8_ENA_MASK                 0x0080  /* AIF1TX8_ENA */
#define CS47L24_AIF1TX8_ENA_SHIFT                     7  /* AIF1TX8_ENA */
#define CS47L24_AIF1TX8_ENA_WIDTH                     1  /* AIF1TX8_ENA */
#define CS47L24_AIF1TX7_ENA                      0x0040  /* AIF1TX7_ENA */
#define CS47L24_AIF1TX7_ENA_MASK                 0x0040  /* AIF1TX7_ENA */
#define CS47L24_AIF1TX7_ENA_SHIFT                     6  /* AIF1TX7_ENA */
#define CS47L24_AIF1TX7_ENA_WIDTH                     1  /* AIF1TX7_ENA */
#define CS47L24_AIF1TX6_ENA                      0x0020  /* AIF1TX6_ENA */
#define CS47L24_AIF1TX6_ENA_MASK                 0x0020  /* AIF1TX6_ENA */
#define CS47L24_AIF1TX6_ENA_SHIFT                     5  /* AIF1TX6_ENA */
#define CS47L24_AIF1TX6_ENA_WIDTH                     1  /* AIF1TX6_ENA */
#define CS47L24_AIF1TX5_ENA                      0x0010  /* AIF1TX5_ENA */
#define CS47L24_AIF1TX5_ENA_MASK                 0x0010  /* AIF1TX5_ENA */
#define CS47L24_AIF1TX5_ENA_SHIFT                     4  /* AIF1TX5_ENA */
#define CS47L24_AIF1TX5_ENA_WIDTH                     1  /* AIF1TX5_ENA */
#define CS47L24_AIF1TX4_ENA                      0x0008  /* AIF1TX4_ENA */
#define CS47L24_AIF1TX4_ENA_MASK                 0x0008  /* AIF1TX4_ENA */
#define CS47L24_AIF1TX4_ENA_SHIFT                     3  /* AIF1TX4_ENA */
#define CS47L24_AIF1TX4_ENA_WIDTH                     1  /* AIF1TX4_ENA */
#define CS47L24_AIF1TX3_ENA                      0x0004  /* AIF1TX3_ENA */
#define CS47L24_AIF1TX3_ENA_MASK                 0x0004  /* AIF1TX3_ENA */
#define CS47L24_AIF1TX3_ENA_SHIFT                     2  /* AIF1TX3_ENA */
#define CS47L24_AIF1TX3_ENA_WIDTH                     1  /* AIF1TX3_ENA */
#define CS47L24_AIF1TX2_ENA                      0x0002  /* AIF1TX2_ENA */
#define CS47L24_AIF1TX2_ENA_MASK                 0x0002  /* AIF1TX2_ENA */
#define CS47L24_AIF1TX2_ENA_SHIFT                     1  /* AIF1TX2_ENA */
#define CS47L24_AIF1TX2_ENA_WIDTH                     1  /* AIF1TX2_ENA */
#define CS47L24_AIF1TX1_ENA                      0x0001  /* AIF1TX1_ENA */
#define CS47L24_AIF1TX1_ENA_MASK                 0x0001  /* AIF1TX1_ENA */
#define CS47L24_AIF1TX1_ENA_SHIFT                     0  /* AIF1TX1_ENA */
#define CS47L24_AIF1TX1_ENA_WIDTH                     1  /* AIF1TX1_ENA */

/*
 * R1306 (0x51A) - AIF1 Rx Enables
 */
#define CS47L24_AIF1RX_ALL_ENA                   0x00FF
#define CS47L24_AIF1RX_ALL_ENA_MASK              0x00FF
#define CS47L24_AIF1RX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF1RX_ALL_ENA_WIDTH                  8
#define CS47L24_AIF1RX8_ENA                      0x0080  /* AIF1RX8_ENA */
#define CS47L24_AIF1RX8_ENA_MASK                 0x0080  /* AIF1RX8_ENA */
#define CS47L24_AIF1RX8_ENA_SHIFT                     7  /* AIF1RX8_ENA */
#define CS47L24_AIF1RX8_ENA_WIDTH                     1  /* AIF1RX8_ENA */
#define CS47L24_AIF1RX7_ENA                      0x0040  /* AIF1RX7_ENA */
#define CS47L24_AIF1RX7_ENA_MASK                 0x0040  /* AIF1RX7_ENA */
#define CS47L24_AIF1RX7_ENA_SHIFT                     6  /* AIF1RX7_ENA */
#define CS47L24_AIF1RX7_ENA_WIDTH                     1  /* AIF1RX7_ENA */
#define CS47L24_AIF1RX6_ENA                      0x0020  /* AIF1RX6_ENA */
#define CS47L24_AIF1RX6_ENA_MASK                 0x0020  /* AIF1RX6_ENA */
#define CS47L24_AIF1RX6_ENA_SHIFT                     5  /* AIF1RX6_ENA */
#define CS47L24_AIF1RX6_ENA_WIDTH                     1  /* AIF1RX6_ENA */
#define CS47L24_AIF1RX5_ENA                      0x0010  /* AIF1RX5_ENA */
#define CS47L24_AIF1RX5_ENA_MASK                 0x0010  /* AIF1RX5_ENA */
#define CS47L24_AIF1RX5_ENA_SHIFT                     4  /* AIF1RX5_ENA */
#define CS47L24_AIF1RX5_ENA_WIDTH                     1  /* AIF1RX5_ENA */
#define CS47L24_AIF1RX4_ENA                      0x0008  /* AIF1RX4_ENA */
#define CS47L24_AIF1RX4_ENA_MASK                 0x0008  /* AIF1RX4_ENA */
#define CS47L24_AIF1RX4_ENA_SHIFT                     3  /* AIF1RX4_ENA */
#define CS47L24_AIF1RX4_ENA_WIDTH                     1  /* AIF1RX4_ENA */
#define CS47L24_AIF1RX3_ENA                      0x0004  /* AIF1RX3_ENA */
#define CS47L24_AIF1RX3_ENA_MASK                 0x0004  /* AIF1RX3_ENA */
#define CS47L24_AIF1RX3_ENA_SHIFT                     2  /* AIF1RX3_ENA */
#define CS47L24_AIF1RX3_ENA_WIDTH                     1  /* AIF1RX3_ENA */
#define CS47L24_AIF1RX2_ENA                      0x0002  /* AIF1RX2_ENA */
#define CS47L24_AIF1RX2_ENA_MASK                 0x0002  /* AIF1RX2_ENA */
#define CS47L24_AIF1RX2_ENA_SHIFT                     1  /* AIF1RX2_ENA */
#define CS47L24_AIF1RX2_ENA_WIDTH                     1  /* AIF1RX2_ENA */
#define CS47L24_AIF1RX1_ENA                      0x0001  /* AIF1RX1_ENA */
#define CS47L24_AIF1RX1_ENA_MASK                 0x0001  /* AIF1RX1_ENA */
#define CS47L24_AIF1RX1_ENA_SHIFT                     0  /* AIF1RX1_ENA */
#define CS47L24_AIF1RX1_ENA_WIDTH                     1  /* AIF1RX1_ENA */

/*
 * R1344 (0x540) - AIF2 BCLK Ctrl
 */
#define CS47L24_AIF2_BCLK_INV                    0x0080  /* AIF2_BCLK_INV */
#define CS47L24_AIF2_BCLK_INV_MASK               0x0080  /* AIF2_BCLK_INV */
#define CS47L24_AIF2_BCLK_INV_SHIFT                   7  /* AIF2_BCLK_INV */
#define CS47L24_AIF2_BCLK_INV_WIDTH                   1  /* AIF2_BCLK_INV */
#define CS47L24_AIF2_BCLK_FRC                    0x0040  /* AIF2_BCLK_FRC */
#define CS47L24_AIF2_BCLK_FRC_MASK               0x0040  /* AIF2_BCLK_FRC */
#define CS47L24_AIF2_BCLK_FRC_SHIFT                   6  /* AIF2_BCLK_FRC */
#define CS47L24_AIF2_BCLK_FRC_WIDTH                   1  /* AIF2_BCLK_FRC */
#define CS47L24_AIF2_BCLK_MSTR                   0x0020  /* AIF2_BCLK_MSTR */
#define CS47L24_AIF2_BCLK_MSTR_MASK              0x0020  /* AIF2_BCLK_MSTR */
#define CS47L24_AIF2_BCLK_MSTR_SHIFT                  5  /* AIF2_BCLK_MSTR */
#define CS47L24_AIF2_BCLK_MSTR_WIDTH                  1  /* AIF2_BCLK_MSTR */
#define CS47L24_AIF2_BCLK_FREQ_MASK              0x001F  /* AIF2_BCLK_FREQ - [4:0] */
#define CS47L24_AIF2_BCLK_FREQ_SHIFT                  0  /* AIF2_BCLK_FREQ - [4:0] */
#define CS47L24_AIF2_BCLK_FREQ_WIDTH                  5  /* AIF2_BCLK_FREQ - [4:0] */

/*
 * R1345 (0x541) - AIF2 Tx Pin Ctrl
 */
#define CS47L24_AIF2TX_DAT_TRI                   0x0020  /* AIF2TX_DAT_TRI */
#define CS47L24_AIF2TX_DAT_TRI_MASK              0x0020  /* AIF2TX_DAT_TRI */
#define CS47L24_AIF2TX_DAT_TRI_SHIFT                  5  /* AIF2TX_DAT_TRI */
#define CS47L24_AIF2TX_DAT_TRI_WIDTH                  1  /* AIF2TX_DAT_TRI */
#define CS47L24_AIF2TX_LRCLK_SRC                 0x0008  /* AIF2TX_LRCLK_SRC */
#define CS47L24_AIF2TX_LRCLK_SRC_MASK            0x0008  /* AIF2TX_LRCLK_SRC */
#define CS47L24_AIF2TX_LRCLK_SRC_SHIFT                3  /* AIF2TX_LRCLK_SRC */
#define CS47L24_AIF2TX_LRCLK_SRC_WIDTH                1  /* AIF2TX_LRCLK_SRC */
#define CS47L24_AIF2TX_LRCLK_INV                 0x0004  /* AIF2TX_LRCLK_INV */
#define CS47L24_AIF2TX_LRCLK_INV_MASK            0x0004  /* AIF2TX_LRCLK_INV */
#define CS47L24_AIF2TX_LRCLK_INV_SHIFT                2  /* AIF2TX_LRCLK_INV */
#define CS47L24_AIF2TX_LRCLK_INV_WIDTH                1  /* AIF2TX_LRCLK_INV */
#define CS47L24_AIF2TX_LRCLK_FRC                 0x0002  /* AIF2TX_LRCLK_FRC */
#define CS47L24_AIF2TX_LRCLK_FRC_MASK            0x0002  /* AIF2TX_LRCLK_FRC */
#define CS47L24_AIF2TX_LRCLK_FRC_SHIFT                1  /* AIF2TX_LRCLK_FRC */
#define CS47L24_AIF2TX_LRCLK_FRC_WIDTH                1  /* AIF2TX_LRCLK_FRC */
#define CS47L24_AIF2TX_LRCLK_MSTR                0x0001  /* AIF2TX_LRCLK_MSTR */
#define CS47L24_AIF2TX_LRCLK_MSTR_MASK           0x0001  /* AIF2TX_LRCLK_MSTR */
#define CS47L24_AIF2TX_LRCLK_MSTR_SHIFT               0  /* AIF2TX_LRCLK_MSTR */
#define CS47L24_AIF2TX_LRCLK_MSTR_WIDTH               1  /* AIF2TX_LRCLK_MSTR */

/*
 * R1346 (0x542) - AIF2 Rx Pin Ctrl
 */
#define CS47L24_AIF2RX_LRCLK_INV                 0x0004  /* AIF2RX_LRCLK_INV */
#define CS47L24_AIF2RX_LRCLK_INV_MASK            0x0004  /* AIF2RX_LRCLK_INV */
#define CS47L24_AIF2RX_LRCLK_INV_SHIFT                2  /* AIF2RX_LRCLK_INV */
#define CS47L24_AIF2RX_LRCLK_INV_WIDTH                1  /* AIF2RX_LRCLK_INV */
#define CS47L24_AIF2RX_LRCLK_FRC                 0x0002  /* AIF2RX_LRCLK_FRC */
#define CS47L24_AIF2RX_LRCLK_FRC_MASK            0x0002  /* AIF2RX_LRCLK_FRC */
#define CS47L24_AIF2RX_LRCLK_FRC_SHIFT                1  /* AIF2RX_LRCLK_FRC */
#define CS47L24_AIF2RX_LRCLK_FRC_WIDTH                1  /* AIF2RX_LRCLK_FRC */
#define CS47L24_AIF2RX_LRCLK_MSTR                0x0001  /* AIF2RX_LRCLK_MSTR */
#define CS47L24_AIF2RX_LRCLK_MSTR_MASK           0x0001  /* AIF2RX_LRCLK_MSTR */
#define CS47L24_AIF2RX_LRCLK_MSTR_SHIFT               0  /* AIF2RX_LRCLK_MSTR */
#define CS47L24_AIF2RX_LRCLK_MSTR_WIDTH               1  /* AIF2RX_LRCLK_MSTR */

/*
 * R1347 (0x543) - AIF2 Rate Ctrl
 */
#define CS47L24_AIF2_RATE_MASK                   0x7800  /* AIF2_RATE - [14:11] */
#define CS47L24_AIF2_RATE_SHIFT                      11  /* AIF2_RATE - [14:11] */
#define CS47L24_AIF2_RATE_WIDTH                       4  /* AIF2_RATE - [14:11] */
#define CS47L24_AIF2_TRI                         0x0040  /* AIF2_TRI */
#define CS47L24_AIF2_TRI_MASK                    0x0040  /* AIF2_TRI */
#define CS47L24_AIF2_TRI_SHIFT                        6  /* AIF2_TRI */
#define CS47L24_AIF2_TRI_WIDTH                        1  /* AIF2_TRI */

/*
 * R1348 (0x544) - AIF2 Format
 */
#define CS47L24_AIF2_FMT_MASK                    0x0007  /* AIF2_FMT - [2:0] */
#define CS47L24_AIF2_FMT_SHIFT                        0  /* AIF2_FMT - [2:0] */
#define CS47L24_AIF2_FMT_WIDTH                        3  /* AIF2_FMT - [2:0] */

/*
 * R1349 (0x545) - AIF2 Tx BCLK Rate
 */
#define CS47L24_AIF2TX_BCPF_MASK                 0x1FFF  /* AIF2TX_BCPF - [12:0] */
#define CS47L24_AIF2TX_BCPF_SHIFT                     0  /* AIF2TX_BCPF - [12:0] */
#define CS47L24_AIF2TX_BCPF_WIDTH                    13  /* AIF2TX_BCPF - [12:0] */

/*
 * R1350 (0x546) - AIF2 Rx BCLK Rate
 */
#define CS47L24_AIF2RX_BCPF_MASK                 0x1FFF  /* AIF2RX_BCPF - [12:0] */
#define CS47L24_AIF2RX_BCPF_SHIFT                     0  /* AIF2RX_BCPF - [12:0] */
#define CS47L24_AIF2RX_BCPF_WIDTH                    13  /* AIF2RX_BCPF - [12:0] */

/*
 * R1351 (0x547) - AIF2 Frame Ctrl 1
 */
#define CS47L24_AIF2TX_WL_MASK                   0x3F00  /* AIF2TX_WL - [13:8] */
#define CS47L24_AIF2TX_WL_SHIFT                       8  /* AIF2TX_WL - [13:8] */
#define CS47L24_AIF2TX_WL_WIDTH                       6  /* AIF2TX_WL - [13:8] */
#define CS47L24_AIF2TX_SLOT_LEN_MASK             0x00FF  /* AIF2TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF2TX_SLOT_LEN_SHIFT                 0  /* AIF2TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF2TX_SLOT_LEN_WIDTH                 8  /* AIF2TX_SLOT_LEN - [7:0] */

/*
 * R1352 (0x548) - AIF2 Frame Ctrl 2
 */
#define CS47L24_AIF2RX_WL_MASK                   0x3F00  /* AIF2RX_WL - [13:8] */
#define CS47L24_AIF2RX_WL_SHIFT                       8  /* AIF2RX_WL - [13:8] */
#define CS47L24_AIF2RX_WL_WIDTH                       6  /* AIF2RX_WL - [13:8] */
#define CS47L24_AIF2RX_SLOT_LEN_MASK             0x00FF  /* AIF2RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF2RX_SLOT_LEN_SHIFT                 0  /* AIF2RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF2RX_SLOT_LEN_WIDTH                 8  /* AIF2RX_SLOT_LEN - [7:0] */

/*
 * R1353 (0x549) - AIF2 Frame Ctrl 3
 */
#define CS47L24_AIF2TX1_SLOT_MASK                0x003F  /* AIF2TX1_SLOT - [5:0] */
#define CS47L24_AIF2TX1_SLOT_SHIFT                    0  /* AIF2TX1_SLOT - [5:0] */
#define CS47L24_AIF2TX1_SLOT_WIDTH                    6  /* AIF2TX1_SLOT - [5:0] */

/*
 * R1354 (0x54A) - AIF2 Frame Ctrl 4
 */
#define CS47L24_AIF2TX2_SLOT_MASK                0x003F  /* AIF2TX2_SLOT - [5:0] */
#define CS47L24_AIF2TX2_SLOT_SHIFT                    0  /* AIF2TX2_SLOT - [5:0] */
#define CS47L24_AIF2TX2_SLOT_WIDTH                    6  /* AIF2TX2_SLOT - [5:0] */

/*
 * R1355 (0x54B) - AIF2 Frame Ctrl 5
 */
#define CS47L24_AIF2TX3_SLOT_MASK                0x003F  /* AIF2TX3_SLOT - [5:0] */
#define CS47L24_AIF2TX3_SLOT_SHIFT                    0  /* AIF2TX3_SLOT - [5:0] */
#define CS47L24_AIF2TX3_SLOT_WIDTH                    6  /* AIF2TX3_SLOT - [5:0] */

/*
 * R1356 (0x54C) - AIF2 Frame Ctrl 6
 */
#define CS47L24_AIF2TX4_SLOT_MASK                0x003F  /* AIF2TX4_SLOT - [5:0] */
#define CS47L24_AIF2TX4_SLOT_SHIFT                    0  /* AIF2TX4_SLOT - [5:0] */
#define CS47L24_AIF2TX4_SLOT_WIDTH                    6  /* AIF2TX4_SLOT - [5:0] */


/*
 * R1357 (0x54D) - AIF2 Frame Ctrl 7
 */
#define CS47L24_AIF2TX5_SLOT_MASK                0x003F  /* AIF2TX5_SLOT - [5:0] */
#define CS47L24_AIF2TX5_SLOT_SHIFT                    0  /* AIF2TX5_SLOT - [5:0] */
#define CS47L24_AIF2TX5_SLOT_WIDTH                    6  /* AIF2TX5_SLOT - [5:0] */

/*
 * R1358 (0x54E) - AIF2 Frame Ctrl 8
 */
#define CS47L24_AIF2TX6_SLOT_MASK                0x003F  /* AIF2TX6_SLOT - [5:0] */
#define CS47L24_AIF2TX6_SLOT_SHIFT                    0  /* AIF2TX6_SLOT - [5:0] */
#define CS47L24_AIF2TX6_SLOT_WIDTH                    6  /* AIF2TX6_SLOT - [5:0] */

/*
 * R1359 (0x54F) - AIF2 Frame Ctrl 9
 */
#define CS47L24_AIF2TX7_SLOT_MASK                0x003F  /* AIF2TX7_SLOT - [5:0] */
#define CS47L24_AIF2TX7_SLOT_SHIFT                    0  /* AIF2TX7_SLOT - [5:0] */
#define CS47L24_AIF2TX7_SLOT_WIDTH                    6  /* AIF2TX7_SLOT - [5:0] */
/*
 * R1360 (0x550) - AIF2 Frame Ctrl 10
 */
#define CS47L24_AIF2TX8_SLOT_MASK                0x003F  /* AIF2TX8_SLOT - [5:0] */
#define CS47L24_AIF2TX8_SLOT_SHIFT                    0  /* AIF2TX8_SLOT - [5:0] */
#define CS47L24_AIF2TX8_SLOT_WIDTH                    6  /* AIF2TX8_SLOT - [5:0] */

/*
 * R1361 (0x551) - AIF2 Frame Ctrl 11
 */
#define CS47L24_AIF2RX1_SLOT_MASK                0x003F  /* AIF2RX1_SLOT - [5:0] */
#define CS47L24_AIF2RX1_SLOT_SHIFT                    0  /* AIF2RX1_SLOT - [5:0] */
#define CS47L24_AIF2RX1_SLOT_WIDTH                    6  /* AIF2RX1_SLOT - [5:0] */

/*
 * R1362 (0x552) - AIF2 Frame Ctrl 12
 */
#define CS47L24_AIF2RX2_SLOT_MASK                0x003F  /* AIF2RX2_SLOT - [5:0] */
#define CS47L24_AIF2RX2_SLOT_SHIFT                    0  /* AIF2RX2_SLOT - [5:0] */
#define CS47L24_AIF2RX2_SLOT_WIDTH                    6  /* AIF2RX2_SLOT - [5:0] */

/*
 * R1363 (0x553) - AIF2 Frame Ctrl 13
 */
#define CS47L24_AIF2RX3_SLOT_MASK                0x003F  /* AIF2RX3_SLOT - [5:0] */
#define CS47L24_AIF2RX3_SLOT_SHIFT                    0  /* AIF2RX3_SLOT - [5:0] */
#define CS47L24_AIF2RX3_SLOT_WIDTH                    6  /* AIF2RX3_SLOT - [5:0] */

/*
 * R1364 (0x554) - AIF2 Frame Ctrl 14
 */
#define CS47L24_AIF2RX4_SLOT_MASK                0x003F  /* AIF2RX4_SLOT - [5:0] */
#define CS47L24_AIF2RX4_SLOT_SHIFT                    0  /* AIF2RX4_SLOT - [5:0] */
#define CS47L24_AIF2RX4_SLOT_WIDTH                    6  /* AIF2RX4_SLOT - [5:0] */

/*
 * R1365 (0x555) - AIF2 Frame Ctrl 15
 */
#define CS47L24_AIF2RX5_SLOT_MASK                0x003F  /* AIF2RX5_SLOT - [5:0] */
#define CS47L24_AIF2RX5_SLOT_SHIFT                    0  /* AIF2RX5_SLOT - [5:0] */
#define CS47L24_AIF2RX5_SLOT_WIDTH                    6  /* AIF2RX5_SLOT - [5:0] */

/*
 * R1366 (0x556) - AIF2 Frame Ctrl 16
 */
#define CS47L24_AIF2RX6_SLOT_MASK                0x003F  /* AIF2RX6_SLOT - [5:0] */
#define CS47L24_AIF2RX6_SLOT_SHIFT                    0  /* AIF2RX6_SLOT - [5:0] */
#define CS47L24_AIF2RX6_SLOT_WIDTH                    6  /* AIF2RX6_SLOT - [5:0] */

/*
 * R1367 (0x557) - AIF2 Frame Ctrl 17
 */
#define CS47L24_AIF2RX7_SLOT_MASK                0x003F  /* AIF2RX7_SLOT - [5:0] */
#define CS47L24_AIF2RX7_SLOT_SHIFT                    0  /* AIF2RX7_SLOT - [5:0] */
#define CS47L24_AIF2RX7_SLOT_WIDTH                    6  /* AIF2RX7_SLOT - [5:0] */

/*
 * R1368 (0x558) - AIF2 Frame Ctrl 18
 */
#define CS47L24_AIF2RX8_SLOT_MASK                0x003F  /* AIF2RX8_SLOT - [5:0] */
#define CS47L24_AIF2RX8_SLOT_SHIFT                    0  /* AIF2RX8_SLOT - [5:0] */
#define CS47L24_AIF2RX8_SLOT_WIDTH                    6  /* AIF2RX8_SLOT - [5:0] */

/*
 * R1369 (0x559) - AIF2 Tx Enables
 */
#define CS47L24_AIF2TX_ALL_ENA                   0x003F
#define CS47L24_AIF2TX_ALL_ENA_MASK              0x003F
#define CS47L24_AIF2TX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF2TX_ALL_ENA_WIDTH                  6
#define CS47L24_AIF2TX6_ENA                      0x0020  /* AIF2TX6_ENA */
#define CS47L24_AIF2TX6_ENA_MASK                 0x0020  /* AIF2TX6_ENA */
#define CS47L24_AIF2TX6_ENA_SHIFT                     5  /* AIF2TX6_ENA */
#define CS47L24_AIF2TX6_ENA_WIDTH                     1  /* AIF2TX6_ENA */
#define CS47L24_AIF2TX5_ENA                      0x0010  /* AIF2TX5_ENA */
#define CS47L24_AIF2TX5_ENA_MASK                 0x0010  /* AIF2TX5_ENA */
#define CS47L24_AIF2TX5_ENA_SHIFT                     4  /* AIF2TX5_ENA */
#define CS47L24_AIF2TX5_ENA_WIDTH                     1  /* AIF2TX5_ENA */
#define CS47L24_AIF2TX4_ENA                      0x0008  /* AIF2TX4_ENA */
#define CS47L24_AIF2TX4_ENA_MASK                 0x0008  /* AIF2TX4_ENA */
#define CS47L24_AIF2TX4_ENA_SHIFT                     3  /* AIF2TX4_ENA */
#define CS47L24_AIF2TX4_ENA_WIDTH                     1  /* AIF2TX4_ENA */
#define CS47L24_AIF2TX3_ENA                      0x0004  /* AIF2TX3_ENA */
#define CS47L24_AIF2TX3_ENA_MASK                 0x0004  /* AIF2TX3_ENA */
#define CS47L24_AIF2TX3_ENA_SHIFT                     2  /* AIF2TX3_ENA */
#define CS47L24_AIF2TX3_ENA_WIDTH                     1  /* AIF2TX3_ENA */
#define CS47L24_AIF2TX2_ENA                      0x0002  /* AIF2TX2_ENA */
#define CS47L24_AIF2TX2_ENA_MASK                 0x0002  /* AIF2TX2_ENA */
#define CS47L24_AIF2TX2_ENA_SHIFT                     1  /* AIF2TX2_ENA */
#define CS47L24_AIF2TX2_ENA_WIDTH                     1  /* AIF2TX2_ENA */
#define CS47L24_AIF2TX1_ENA                      0x0001  /* AIF2TX1_ENA */
#define CS47L24_AIF2TX1_ENA_MASK                 0x0001  /* AIF2TX1_ENA */
#define CS47L24_AIF2TX1_ENA_SHIFT                     0  /* AIF2TX1_ENA */
#define CS47L24_AIF2TX1_ENA_WIDTH                     1  /* AIF2TX1_ENA */

/*
 * R1370 (0x55A) - AIF2 Rx Enables
 */
#define CS47L24_AIF2RX_ALL_ENA                   0x003F
#define CS47L24_AIF2RX_ALL_ENA_MASK              0x003F
#define CS47L24_AIF2RX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF2RX_ALL_ENA_WIDTH                  6
#define CS47L24_AIF2RX6_ENA                      0x0020  /* AIF2RX6_ENA */
#define CS47L24_AIF2RX6_ENA_MASK                 0x0020  /* AIF2RX6_ENA */
#define CS47L24_AIF2RX6_ENA_SHIFT                     5  /* AIF2RX6_ENA */
#define CS47L24_AIF2RX6_ENA_WIDTH                     1  /* AIF2RX6_ENA */
#define CS47L24_AIF2RX5_ENA                      0x0010  /* AIF2RX5_ENA */
#define CS47L24_AIF2RX5_ENA_MASK                 0x0010  /* AIF2RX5_ENA */
#define CS47L24_AIF2RX5_ENA_SHIFT                     4  /* AIF2RX5_ENA */
#define CS47L24_AIF2RX5_ENA_WIDTH                     1  /* AIF2RX5_ENA */
#define CS47L24_AIF2RX4_ENA                      0x0008  /* AIF2RX4_ENA */
#define CS47L24_AIF2RX4_ENA_MASK                 0x0008  /* AIF2RX4_ENA */
#define CS47L24_AIF2RX4_ENA_SHIFT                     3  /* AIF2RX4_ENA */
#define CS47L24_AIF2RX4_ENA_WIDTH                     1  /* AIF2RX4_ENA */
#define CS47L24_AIF2RX3_ENA                      0x0004  /* AIF2RX3_ENA */
#define CS47L24_AIF2RX3_ENA_MASK                 0x0004  /* AIF2RX3_ENA */
#define CS47L24_AIF2RX3_ENA_SHIFT                     2  /* AIF2RX3_ENA */
#define CS47L24_AIF2RX3_ENA_WIDTH                     1  /* AIF2RX3_ENA */
#define CS47L24_AIF2RX2_ENA                      0x0002  /* AIF2RX2_ENA */
#define CS47L24_AIF2RX2_ENA_MASK                 0x0002  /* AIF2RX2_ENA */
#define CS47L24_AIF2RX2_ENA_SHIFT                     1  /* AIF2RX2_ENA */
#define CS47L24_AIF2RX2_ENA_WIDTH                     1  /* AIF2RX2_ENA */
#define CS47L24_AIF2RX1_ENA                      0x0001  /* AIF2RX1_ENA */
#define CS47L24_AIF2RX1_ENA_MASK                 0x0001  /* AIF2RX1_ENA */
#define CS47L24_AIF2RX1_ENA_SHIFT                     0  /* AIF2RX1_ENA */
#define CS47L24_AIF2RX1_ENA_WIDTH                     1  /* AIF2RX1_ENA */

/*
 * R1408 (0x580) - AIF3 BCLK Ctrl
 */
#define CS47L24_AIF3_BCLK_INV                    0x0080  /* AIF3_BCLK_INV */
#define CS47L24_AIF3_BCLK_INV_MASK               0x0080  /* AIF3_BCLK_INV */
#define CS47L24_AIF3_BCLK_INV_SHIFT                   7  /* AIF3_BCLK_INV */
#define CS47L24_AIF3_BCLK_INV_WIDTH                   1  /* AIF3_BCLK_INV */
#define CS47L24_AIF3_BCLK_FRC                    0x0040  /* AIF3_BCLK_FRC */
#define CS47L24_AIF3_BCLK_FRC_MASK               0x0040  /* AIF3_BCLK_FRC */
#define CS47L24_AIF3_BCLK_FRC_SHIFT                   6  /* AIF3_BCLK_FRC */
#define CS47L24_AIF3_BCLK_FRC_WIDTH                   1  /* AIF3_BCLK_FRC */
#define CS47L24_AIF3_BCLK_MSTR                   0x0020  /* AIF3_BCLK_MSTR */
#define CS47L24_AIF3_BCLK_MSTR_MASK              0x0020  /* AIF3_BCLK_MSTR */
#define CS47L24_AIF3_BCLK_MSTR_SHIFT                  5  /* AIF3_BCLK_MSTR */
#define CS47L24_AIF3_BCLK_MSTR_WIDTH                  1  /* AIF3_BCLK_MSTR */
#define CS47L24_AIF3_BCLK_FREQ_MASK              0x001F  /* AIF3_BCLK_FREQ - [4:0] */
#define CS47L24_AIF3_BCLK_FREQ_SHIFT                  0  /* AIF3_BCLK_FREQ - [4:0] */
#define CS47L24_AIF3_BCLK_FREQ_WIDTH                  5  /* AIF3_BCLK_FREQ - [4:0] */

/*
 * R1409 (0x581) - AIF3 Tx Pin Ctrl
 */
#define CS47L24_AIF3TX_DAT_TRI                   0x0020  /* AIF3TX_DAT_TRI */
#define CS47L24_AIF3TX_DAT_TRI_MASK              0x0020  /* AIF3TX_DAT_TRI */
#define CS47L24_AIF3TX_DAT_TRI_SHIFT                  5  /* AIF3TX_DAT_TRI */
#define CS47L24_AIF3TX_DAT_TRI_WIDTH                  1  /* AIF3TX_DAT_TRI */
#define CS47L24_AIF3TX_LRCLK_SRC                 0x0008  /* AIF3TX_LRCLK_SRC */
#define CS47L24_AIF3TX_LRCLK_SRC_MASK            0x0008  /* AIF3TX_LRCLK_SRC */
#define CS47L24_AIF3TX_LRCLK_SRC_SHIFT                3  /* AIF3TX_LRCLK_SRC */
#define CS47L24_AIF3TX_LRCLK_SRC_WIDTH                1  /* AIF3TX_LRCLK_SRC */
#define CS47L24_AIF3TX_LRCLK_INV                 0x0004  /* AIF3TX_LRCLK_INV */
#define CS47L24_AIF3TX_LRCLK_INV_MASK            0x0004  /* AIF3TX_LRCLK_INV */
#define CS47L24_AIF3TX_LRCLK_INV_SHIFT                2  /* AIF3TX_LRCLK_INV */
#define CS47L24_AIF3TX_LRCLK_INV_WIDTH                1  /* AIF3TX_LRCLK_INV */
#define CS47L24_AIF3TX_LRCLK_FRC                 0x0002  /* AIF3TX_LRCLK_FRC */
#define CS47L24_AIF3TX_LRCLK_FRC_MASK            0x0002  /* AIF3TX_LRCLK_FRC */
#define CS47L24_AIF3TX_LRCLK_FRC_SHIFT                1  /* AIF3TX_LRCLK_FRC */
#define CS47L24_AIF3TX_LRCLK_FRC_WIDTH                1  /* AIF3TX_LRCLK_FRC */
#define CS47L24_AIF3TX_LRCLK_MSTR                0x0001  /* AIF3TX_LRCLK_MSTR */
#define CS47L24_AIF3TX_LRCLK_MSTR_MASK           0x0001  /* AIF3TX_LRCLK_MSTR */
#define CS47L24_AIF3TX_LRCLK_MSTR_SHIFT               0  /* AIF3TX_LRCLK_MSTR */
#define CS47L24_AIF3TX_LRCLK_MSTR_WIDTH               1  /* AIF3TX_LRCLK_MSTR */

/*
 * R1410 (0x582) - AIF3 Rx Pin Ctrl
 */
#define CS47L24_AIF3RX_LRCLK_INV                 0x0004  /* AIF3RX_LRCLK_INV */
#define CS47L24_AIF3RX_LRCLK_INV_MASK            0x0004  /* AIF3RX_LRCLK_INV */
#define CS47L24_AIF3RX_LRCLK_INV_SHIFT                2  /* AIF3RX_LRCLK_INV */
#define CS47L24_AIF3RX_LRCLK_INV_WIDTH                1  /* AIF3RX_LRCLK_INV */
#define CS47L24_AIF3RX_LRCLK_FRC                 0x0002  /* AIF3RX_LRCLK_FRC */
#define CS47L24_AIF3RX_LRCLK_FRC_MASK            0x0002  /* AIF3RX_LRCLK_FRC */
#define CS47L24_AIF3RX_LRCLK_FRC_SHIFT                1  /* AIF3RX_LRCLK_FRC */
#define CS47L24_AIF3RX_LRCLK_FRC_WIDTH                1  /* AIF3RX_LRCLK_FRC */
#define CS47L24_AIF3RX_LRCLK_MSTR                0x0001  /* AIF3RX_LRCLK_MSTR */
#define CS47L24_AIF3RX_LRCLK_MSTR_MASK           0x0001  /* AIF3RX_LRCLK_MSTR */
#define CS47L24_AIF3RX_LRCLK_MSTR_SHIFT               0  /* AIF3RX_LRCLK_MSTR */
#define CS47L24_AIF3RX_LRCLK_MSTR_WIDTH               1  /* AIF3RX_LRCLK_MSTR */

/*
 * R1411 (0x583) - AIF3 Rate Ctrl
 */
#define CS47L24_AIF3_RATE_MASK                   0x7800  /* AIF3_RATE - [14:11] */
#define CS47L24_AIF3_RATE_SHIFT                      11  /* AIF3_RATE - [14:11] */
#define CS47L24_AIF3_RATE_WIDTH                       4  /* AIF3_RATE - [14:11] */
#define CS47L24_AIF3_TRI                         0x0040  /* AIF3_TRI */
#define CS47L24_AIF3_TRI_MASK                    0x0040  /* AIF3_TRI */
#define CS47L24_AIF3_TRI_SHIFT                        6  /* AIF3_TRI */
#define CS47L24_AIF3_TRI_WIDTH                        1  /* AIF3_TRI */

/*
 * R1412 (0x584) - AIF3 Format
 */
#define CS47L24_AIF3_FMT_MASK                    0x0007  /* AIF3_FMT - [2:0] */
#define CS47L24_AIF3_FMT_SHIFT                        0  /* AIF3_FMT - [2:0] */
#define CS47L24_AIF3_FMT_WIDTH                        3  /* AIF3_FMT - [2:0] */

/*
 * R1413 (0x585) - AIF3 Tx BCLK Rate
 */
#define CS47L24_AIF3TX_BCPF_MASK                 0x1FFF  /* AIF3TX_BCPF - [12:0] */
#define CS47L24_AIF3TX_BCPF_SHIFT                     0  /* AIF3TX_BCPF - [12:0] */
#define CS47L24_AIF3TX_BCPF_WIDTH                    13  /* AIF3TX_BCPF - [12:0] */

/*
 * R1414 (0x586) - AIF3 Rx BCLK Rate
 */
#define CS47L24_AIF3RX_BCPF_MASK                 0x1FFF  /* AIF3RX_BCPF - [12:0] */
#define CS47L24_AIF3RX_BCPF_SHIFT                     0  /* AIF3RX_BCPF - [12:0] */
#define CS47L24_AIF3RX_BCPF_WIDTH                    13  /* AIF3RX_BCPF - [12:0] */

/*
 * R1415 (0x587) - AIF3 Frame Ctrl 1
 */
#define CS47L24_AIF3TX_WL_MASK                   0x3F00  /* AIF3TX_WL - [13:8] */
#define CS47L24_AIF3TX_WL_SHIFT                       8  /* AIF3TX_WL - [13:8] */
#define CS47L24_AIF3TX_WL_WIDTH                       6  /* AIF3TX_WL - [13:8] */
#define CS47L24_AIF3TX_SLOT_LEN_MASK             0x00FF  /* AIF3TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF3TX_SLOT_LEN_SHIFT                 0  /* AIF3TX_SLOT_LEN - [7:0] */
#define CS47L24_AIF3TX_SLOT_LEN_WIDTH                 8  /* AIF3TX_SLOT_LEN - [7:0] */

/*
 * R1416 (0x588) - AIF3 Frame Ctrl 2
 */
#define CS47L24_AIF3RX_WL_MASK                   0x3F00  /* AIF3RX_WL - [13:8] */
#define CS47L24_AIF3RX_WL_SHIFT                       8  /* AIF3RX_WL - [13:8] */
#define CS47L24_AIF3RX_WL_WIDTH                       6  /* AIF3RX_WL - [13:8] */
#define CS47L24_AIF3RX_SLOT_LEN_MASK             0x00FF  /* AIF3RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF3RX_SLOT_LEN_SHIFT                 0  /* AIF3RX_SLOT_LEN - [7:0] */
#define CS47L24_AIF3RX_SLOT_LEN_WIDTH                 8  /* AIF3RX_SLOT_LEN - [7:0] */

/*
 * R1417 (0x589) - AIF3 Frame Ctrl 3
 */
#define CS47L24_AIF3TX1_SLOT_MASK                0x003F  /* AIF3TX1_SLOT - [5:0] */
#define CS47L24_AIF3TX1_SLOT_SHIFT                    0  /* AIF3TX1_SLOT - [5:0] */
#define CS47L24_AIF3TX1_SLOT_WIDTH                    6  /* AIF3TX1_SLOT - [5:0] */

/*
 * R1418 (0x58A) - AIF3 Frame Ctrl 4
 */
#define CS47L24_AIF3TX2_SLOT_MASK                0x003F  /* AIF3TX2_SLOT - [5:0] */
#define CS47L24_AIF3TX2_SLOT_SHIFT                    0  /* AIF3TX2_SLOT - [5:0] */
#define CS47L24_AIF3TX2_SLOT_WIDTH                    6  /* AIF3TX2_SLOT - [5:0] */

/*
 * R1425 (0x591) - AIF3 Frame Ctrl 11
 */
#define CS47L24_AIF3RX1_SLOT_MASK                0x003F  /* AIF3RX1_SLOT - [5:0] */
#define CS47L24_AIF3RX1_SLOT_SHIFT                    0  /* AIF3RX1_SLOT - [5:0] */
#define CS47L24_AIF3RX1_SLOT_WIDTH                    6  /* AIF3RX1_SLOT - [5:0] */

/*
 * R1426 (0x592) - AIF3 Frame Ctrl 12
 */
#define CS47L24_AIF3RX2_SLOT_MASK                0x003F  /* AIF3RX2_SLOT - [5:0] */
#define CS47L24_AIF3RX2_SLOT_SHIFT                    0  /* AIF3RX2_SLOT - [5:0] */
#define CS47L24_AIF3RX2_SLOT_WIDTH                    6  /* AIF3RX2_SLOT - [5:0] */

/*
 * R1433 (0x599) - AIF3 Tx Enables
 */
#define CS47L24_AIF3TX_ALL_ENA                   0x0003
#define CS47L24_AIF3TX_ALL_ENA_MASK              0x0003
#define CS47L24_AIF3TX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF3TX_ALL_ENA_WIDTH                  2
#define CS47L24_AIF3TX2_ENA                      0x0002  /* AIF3TX2_ENA */
#define CS47L24_AIF3TX2_ENA_MASK                 0x0002  /* AIF3TX2_ENA */
#define CS47L24_AIF3TX2_ENA_SHIFT                     1  /* AIF3TX2_ENA */
#define CS47L24_AIF3TX2_ENA_WIDTH                     1  /* AIF3TX2_ENA */
#define CS47L24_AIF3TX1_ENA                      0x0001  /* AIF3TX1_ENA */
#define CS47L24_AIF3TX1_ENA_MASK                 0x0001  /* AIF3TX1_ENA */
#define CS47L24_AIF3TX1_ENA_SHIFT                     0  /* AIF3TX1_ENA */
#define CS47L24_AIF3TX1_ENA_WIDTH                     1  /* AIF3TX1_ENA */

/*
 * R1434 (0x59A) - AIF3 Rx Enables
 */
#define CS47L24_AIF3RX_ALL_ENA                   0x0003
#define CS47L24_AIF3RX_ALL_ENA_MASK              0x0003
#define CS47L24_AIF3RX_ALL_ENA_SHIFT                  0
#define CS47L24_AIF3RX_ALL_ENA_WIDTH                  2
#define CS47L24_AIF3RX2_ENA                      0x0002  /* AIF3RX2_ENA */
#define CS47L24_AIF3RX2_ENA_MASK                 0x0002  /* AIF3RX2_ENA */
#define CS47L24_AIF3RX2_ENA_SHIFT                     1  /* AIF3RX2_ENA */
#define CS47L24_AIF3RX2_ENA_WIDTH                     1  /* AIF3RX2_ENA */
#define CS47L24_AIF3RX1_ENA                      0x0001  /* AIF3RX1_ENA */
#define CS47L24_AIF3RX1_ENA_MASK                 0x0001  /* AIF3RX1_ENA */
#define CS47L24_AIF3RX1_ENA_SHIFT                     0  /* AIF3RX1_ENA */
#define CS47L24_AIF3RX1_ENA_WIDTH                     1  /* AIF3RX1_ENA */

/*
 * R1600 (0640h) to R3000 (0BB8h) (EVEN)
 */
#define CS47L24_INPUT_SOURCE_ENA_STS             0x8000
#define CS47L24_INPUT_SOURCE_ENA_STS_MASK        0x8000
#define CS47L24_INPUT_SOURCE_ENA_STS_SHIFT           15
#define CS47L24_INPUT_SOURCE_ENA_STS_WIDTH            1
#define CS47L24_INPUT_SOURCE_MASK                0x00FF
#define CS47L24_INPUT_SOURCE_SHIFT                    0
#define CS47L24_INPUT_SOURCE_WIDTH                    8

/*
 * R1601 (0641h) to R2511 (09CFh) (ODD)
 */
#define CS47L24_INPUT_VOLUME_MASK                0x00FE
#define CS47L24_INPUT_VOLUME_SHIFT                    1
#define CS47L24_INPUT_VOLUME_WIDTH                    7

/*
 * R3072 (0C00h) and R3073 (0C01h)
 */
#define CS47L24_GPIO_DIR_OUT                     0x0000
#define CS47L24_GPIO_DIR_IN                      0x8000
#define CS47L24_GPIO_DIR_MASK                    0x8000
#define CS47L24_GPIO_DIR_SHIFT                       15
#define CS47L24_GPIO_DIR_WIDTH                        1
#define CS47L24_GPIO_PUP_ENA                     0x4000
#define CS47L24_GPIO_PUP_MASK                    0x4000
#define CS47L24_GPIO_PUP_SHIFT                       14
#define CS47L24_GPIO_PUP_WIDTH                        1
#define CS47L24_GPIO_PDOWN_ENA                   0x2000
#define CS47L24_GPIO_PDOWN_MASK                  0x2000
#define CS47L24_GPIO_PDOWN_SHIFT                     13
#define CS47L24_GPIO_PDOWN_WIDTH                      1
#define CS47L24_GPIO_LEVEL_MASK                  0x0800
#define CS47L24_GPIO_LEVEL_SHIFT                     11
#define CS47L24_GPIO_LEVEL_WIDTH                      1
#define CS47L24_GPIO_POLARITY_INV                0x0400
#define CS47L24_GPIO_POLARITY_MASK               0x0400
#define CS47L24_GPIO_POLARITY_SHIFT                  10
#define CS47L24_GPIO_POLARITY_WIDTH                   1
#define CS47L24_GPIO_OUT_CONF_CMOS               0x0000
#define CS47L24_GPIO_OUT_CONF_OPEND              0x0200
#define CS47L24_GPIO_OUT_CONF_MASK               0x0200
#define CS47L24_GPIO_OUT_CONF_SHIFT                   9
#define CS47L24_GPIO_OUT_CONF_WIDTH                   1
#define CS47L24_GPIO_IN_DEBOUNCE_ENA             0x0100
#define CS47L24_GPIO_IN_DEBOUNCE_MASK            0x0100
#define CS47L24_GPIO_IN_DEBOUNCE_SHIFT                8
#define CS47L24_GPIO_IN_DEBOUNCE_WIDTH                1
#define CS47L24_GPIO_FUNC_MASK                   0x007F
#define CS47L24_GPIO_FUNC_SHIFT                       0
#define CS47L24_GPIO_FUNC_WIDTH                       7
/*
 * R3087 (0xC0F) - IRQ CTRL 1
 */
#define CS47L24_IRQ_POL                          0x0400  /* IRQ_POL */
#define CS47L24_IRQ_POL_MASK                     0x0400  /* IRQ_POL */
#define CS47L24_IRQ_POL_SHIFT                        10  /* IRQ_POL */
#define CS47L24_IRQ_POL_WIDTH                         1  /* IRQ_POL */
#define CS47L24_IRQ_OP_CFG                       0x0200  /* IRQ_OP_CFG */
#define CS47L24_IRQ_OP_CFG_MASK                  0x0200  /* IRQ_OP_CFG */
#define CS47L24_IRQ_OP_CFG_SHIFT                      9  /* IRQ_OP_CFG */
#define CS47L24_IRQ_OP_CFG_WIDTH                      1  /* IRQ_OP_CFG */

/*
 * R3088 (0xC10) - GPIO Debounce Config
 */
#define CS47L24_GP_DBTIME_MASK                   0xF000  /* GP_DBTIME - [15:12] */
#define CS47L24_GP_DBTIME_SHIFT                      12  /* GP_DBTIME - [15:12] */
#define CS47L24_GP_DBTIME_WIDTH                       4  /* GP_DBTIME - [15:12] */

/*
 * R3104 (0xC20) - Misc Pad Ctrl 1
 */
#define CS47L24_LDO1ENA_PD                       0x8000  /* LDO1ENA_PD */
#define CS47L24_LDO1ENA_PD_MASK                  0x8000  /* LDO1ENA_PD */
#define CS47L24_LDO1ENA_PD_SHIFT                     15  /* LDO1ENA_PD */
#define CS47L24_LDO1ENA_PD_WIDTH                      1  /* LDO1ENA_PD */
#define CS47L24_MCLK2_PD                         0x2000  /* MCLK2_PD */
#define CS47L24_MCLK2_PD_MASK                    0x2000  /* MCLK2_PD */
#define CS47L24_MCLK2_PD_SHIFT                       13  /* MCLK2_PD */
#define CS47L24_MCLK2_PD_WIDTH                        1  /* MCLK2_PD */
#define CS47L24_RSTB_PU                          0x0002  /* RSTB_PU */
#define CS47L24_RSTB_PU_MASK                     0x0002  /* RSTB_PU */
#define CS47L24_RSTB_PU_SHIFT                         1  /* RSTB_PU */
#define CS47L24_RSTB_PU_WIDTH                         1  /* RSTB_PU */

/*
 * R3105 (0xC21) - Misc Pad Ctrl 2
 */
#define CS47L24_MCLK1_PD                         0x1000  /* MCLK1_PD */
#define CS47L24_MCLK1_PD_MASK                    0x1000  /* MCLK1_PD */
#define CS47L24_MCLK1_PD_SHIFT                       12  /* MCLK1_PD */
#define CS47L24_MCLK1_PD_WIDTH                        1  /* MCLK1_PD */
#define CS47L24_MICD_PD                          0x0100  /* MICD_PD */
#define CS47L24_MICD_PD_MASK                     0x0100  /* MICD_PD */
#define CS47L24_MICD_PD_SHIFT                         8  /* MICD_PD */
#define CS47L24_MICD_PD_WIDTH                         1  /* MICD_PD */
#define CS47L24_ADDR_PD                          0x0001  /* ADDR_PD */
#define CS47L24_ADDR_PD_MASK                     0x0001  /* ADDR_PD */
#define CS47L24_ADDR_PD_SHIFT                         0  /* ADDR_PD */
#define CS47L24_ADDR_PD_WIDTH                         1  /* ADDR_PD */

/*
 * R3106 (0xC22) - Misc Pad Ctrl 3
 */
#define CS47L24_DMICDAT4_PD                      0x0008  /* DMICDAT4_PD */
#define CS47L24_DMICDAT4_PD_MASK                 0x0008  /* DMICDAT4_PD */
#define CS47L24_DMICDAT4_PD_SHIFT                     3  /* DMICDAT4_PD */
#define CS47L24_DMICDAT4_PD_WIDTH                     1  /* DMICDAT4_PD */
#define CS47L24_DMICDAT3_PD                      0x0004  /* DMICDAT3_PD */
#define CS47L24_DMICDAT3_PD_MASK                 0x0004  /* DMICDAT3_PD */
#define CS47L24_DMICDAT3_PD_SHIFT                     2  /* DMICDAT3_PD */
#define CS47L24_DMICDAT3_PD_WIDTH                     1  /* DMICDAT3_PD */
#define CS47L24_DMICDAT2_PD                      0x0002  /* DMICDAT2_PD */
#define CS47L24_DMICDAT2_PD_MASK                 0x0002  /* DMICDAT2_PD */
#define CS47L24_DMICDAT2_PD_SHIFT                     1  /* DMICDAT2_PD */
#define CS47L24_DMICDAT2_PD_WIDTH                     1  /* DMICDAT2_PD */
#define CS47L24_DMICDAT1_PD                      0x0001  /* DMICDAT1_PD */
#define CS47L24_DMICDAT1_PD_MASK                 0x0001  /* DMICDAT1_PD */
#define CS47L24_DMICDAT1_PD_SHIFT                     0  /* DMICDAT1_PD */
#define CS47L24_DMICDAT1_PD_WIDTH                     1  /* DMICDAT1_PD */

/*
 * R3107 (0xC23) - Misc Pad Ctrl 4
 */
#define CS47L24_AIF1RXLRCLK_PU                   0x0020  /* AIF1RXLRCLK_PU */
#define CS47L24_AIF1RXLRCLK_PU_MASK              0x0020  /* AIF1RXLRCLK_PU */
#define CS47L24_AIF1RXLRCLK_PU_SHIFT                  5  /* AIF1RXLRCLK_PU */
#define CS47L24_AIF1RXLRCLK_PU_WIDTH                  1  /* AIF1RXLRCLK_PU */
#define CS47L24_AIF1RXLRCLK_PD                   0x0010  /* AIF1RXLRCLK_PD */
#define CS47L24_AIF1RXLRCLK_PD_MASK              0x0010  /* AIF1RXLRCLK_PD */
#define CS47L24_AIF1RXLRCLK_PD_SHIFT                  4  /* AIF1RXLRCLK_PD */
#define CS47L24_AIF1RXLRCLK_PD_WIDTH                  1  /* AIF1RXLRCLK_PD */
#define CS47L24_AIF1BCLK_PU                      0x0008  /* AIF1BCLK_PU */
#define CS47L24_AIF1BCLK_PU_MASK                 0x0008  /* AIF1BCLK_PU */
#define CS47L24_AIF1BCLK_PU_SHIFT                     3  /* AIF1BCLK_PU */
#define CS47L24_AIF1BCLK_PU_WIDTH                     1  /* AIF1BCLK_PU */
#define CS47L24_AIF1BCLK_PD                      0x0004  /* AIF1BCLK_PD */
#define CS47L24_AIF1BCLK_PD_MASK                 0x0004  /* AIF1BCLK_PD */
#define CS47L24_AIF1BCLK_PD_SHIFT                     2  /* AIF1BCLK_PD */
#define CS47L24_AIF1BCLK_PD_WIDTH                     1  /* AIF1BCLK_PD */
#define CS47L24_AIF1RXDAT_PU                     0x0002  /* AIF1RXDAT_PU */
#define CS47L24_AIF1RXDAT_PU_MASK                0x0002  /* AIF1RXDAT_PU */
#define CS47L24_AIF1RXDAT_PU_SHIFT                    1  /* AIF1RXDAT_PU */
#define CS47L24_AIF1RXDAT_PU_WIDTH                    1  /* AIF1RXDAT_PU */
#define CS47L24_AIF1RXDAT_PD                     0x0001  /* AIF1RXDAT_PD */
#define CS47L24_AIF1RXDAT_PD_MASK                0x0001  /* AIF1RXDAT_PD */
#define CS47L24_AIF1RXDAT_PD_SHIFT                    0  /* AIF1RXDAT_PD */
#define CS47L24_AIF1RXDAT_PD_WIDTH                    1  /* AIF1RXDAT_PD */

/*
 * R3108 (0xC24) - Misc Pad Ctrl 5
 */
#define CS47L24_AIF2RXLRCLK_PU                   0x0020  /* AIF2RXLRCLK_PU */
#define CS47L24_AIF2RXLRCLK_PU_MASK              0x0020  /* AIF2RXLRCLK_PU */
#define CS47L24_AIF2RXLRCLK_PU_SHIFT                  5  /* AIF2RXLRCLK_PU */
#define CS47L24_AIF2RXLRCLK_PU_WIDTH                  1  /* AIF2RXLRCLK_PU */
#define CS47L24_AIF2RXLRCLK_PD                   0x0010  /* AIF2RXLRCLK_PD */
#define CS47L24_AIF2RXLRCLK_PD_MASK              0x0010  /* AIF2RXLRCLK_PD */
#define CS47L24_AIF2RXLRCLK_PD_SHIFT                  4  /* AIF2RXLRCLK_PD */
#define CS47L24_AIF2RXLRCLK_PD_WIDTH                  1  /* AIF2RXLRCLK_PD */
#define CS47L24_AIF2BCLK_PU                      0x0008  /* AIF2BCLK_PU */
#define CS47L24_AIF2BCLK_PU_MASK                 0x0008  /* AIF2BCLK_PU */
#define CS47L24_AIF2BCLK_PU_SHIFT                     3  /* AIF2BCLK_PU */
#define CS47L24_AIF2BCLK_PU_WIDTH                     1  /* AIF2BCLK_PU */
#define CS47L24_AIF2BCLK_PD                      0x0004  /* AIF2BCLK_PD */
#define CS47L24_AIF2BCLK_PD_MASK                 0x0004  /* AIF2BCLK_PD */
#define CS47L24_AIF2BCLK_PD_SHIFT                     2  /* AIF2BCLK_PD */
#define CS47L24_AIF2BCLK_PD_WIDTH                     1  /* AIF2BCLK_PD */
#define CS47L24_AIF2RXDAT_PU                     0x0002  /* AIF2RXDAT_PU */
#define CS47L24_AIF2RXDAT_PU_MASK                0x0002  /* AIF2RXDAT_PU */
#define CS47L24_AIF2RXDAT_PU_SHIFT                    1  /* AIF2RXDAT_PU */
#define CS47L24_AIF2RXDAT_PU_WIDTH                    1  /* AIF2RXDAT_PU */
#define CS47L24_AIF2RXDAT_PD                     0x0001  /* AIF2RXDAT_PD */
#define CS47L24_AIF2RXDAT_PD_MASK                0x0001  /* AIF2RXDAT_PD */
#define CS47L24_AIF2RXDAT_PD_SHIFT                    0  /* AIF2RXDAT_PD */
#define CS47L24_AIF2RXDAT_PD_WIDTH                    1  /* AIF2RXDAT_PD */

/*
 * R3109 (0xC25) - Misc Pad Ctrl 6
 */
#define CS47L24_AIF3RXLRCLK_PU                   0x0020  /* AIF3RXLRCLK_PU */
#define CS47L24_AIF3RXLRCLK_PU_MASK              0x0020  /* AIF3RXLRCLK_PU */
#define CS47L24_AIF3RXLRCLK_PU_SHIFT                  5  /* AIF3RXLRCLK_PU */
#define CS47L24_AIF3RXLRCLK_PU_WIDTH                  1  /* AIF3RXLRCLK_PU */
#define CS47L24_AIF3RXLRCLK_PD                   0x0010  /* AIF3RXLRCLK_PD */
#define CS47L24_AIF3RXLRCLK_PD_MASK              0x0010  /* AIF3RXLRCLK_PD */
#define CS47L24_AIF3RXLRCLK_PD_SHIFT                  4  /* AIF3RXLRCLK_PD */
#define CS47L24_AIF3RXLRCLK_PD_WIDTH                  1  /* AIF3RXLRCLK_PD */
#define CS47L24_AIF3BCLK_PU                      0x0008  /* AIF3BCLK_PU */
#define CS47L24_AIF3BCLK_PU_MASK                 0x0008  /* AIF3BCLK_PU */
#define CS47L24_AIF3BCLK_PU_SHIFT                     3  /* AIF3BCLK_PU */
#define CS47L24_AIF3BCLK_PU_WIDTH                     1  /* AIF3BCLK_PU */
#define CS47L24_AIF3BCLK_PD                      0x0004  /* AIF3BCLK_PD */
#define CS47L24_AIF3BCLK_PD_MASK                 0x0004  /* AIF3BCLK_PD */
#define CS47L24_AIF3BCLK_PD_SHIFT                     2  /* AIF3BCLK_PD */
#define CS47L24_AIF3BCLK_PD_WIDTH                     1  /* AIF3BCLK_PD */
#define CS47L24_AIF3RXDAT_PU                     0x0002  /* AIF3RXDAT_PU */
#define CS47L24_AIF3RXDAT_PU_MASK                0x0002  /* AIF3RXDAT_PU */
#define CS47L24_AIF3RXDAT_PU_SHIFT                    1  /* AIF3RXDAT_PU */
#define CS47L24_AIF3RXDAT_PU_WIDTH                    1  /* AIF3RXDAT_PU */
#define CS47L24_AIF3RXDAT_PD                     0x0001  /* AIF3RXDAT_PD */
#define CS47L24_AIF3RXDAT_PD_MASK                0x0001  /* AIF3RXDAT_PD */
#define CS47L24_AIF3RXDAT_PD_SHIFT                    0  /* AIF3RXDAT_PD */
#define CS47L24_AIF3RXDAT_PD_WIDTH                    1  /* AIF3RXDAT_PD */

/*
 * R3328 (0xD00) - Interrupt Status 1
 */
#define CS47L24_GP2_EINT1                        0x0002  /* GP2_EINT1 */
#define CS47L24_GP2_EINT1_MASK                   0x0002  /* GP2_EINT1 */
#define CS47L24_GP2_EINT1_SHIFT                       1  /* GP2_EINT1 */
#define CS47L24_GP2_EINT1_WIDTH                       1  /* GP2_EINT1 */
#define CS47L24_GP1_EINT1                        0x0001  /* GP1_EINT1 */
#define CS47L24_GP1_EINT1_MASK                   0x0001  /* GP1_EINT1 */
#define CS47L24_GP1_EINT1_SHIFT                       0  /* GP1_EINT1 */
#define CS47L24_GP1_EINT1_WIDTH                       1  /* GP1_EINT1 */

/*
 * R3329 (0xD01) - Interrupt Status 2
 */
#define CS47L24_DSP3_RAM_RDY_EINT1               0x0400  /* DSP3_RAM_RDY_EINT1 */
#define CS47L24_DSP3_RAM_RDY_EINT1_MASK          0x0400  /* DSP3_RAM_RDY_EINT1 */
#define CS47L24_DSP3_RAM_RDY_EINT1_SHIFT             10  /* DSP3_RAM_RDY_EINT1 */
#define CS47L24_DSP3_RAM_RDY_EINT1_WIDTH              1  /* DSP3_RAM_RDY_EINT1 */
#define CS47L24_DSP2_RAM_RDY_EINT1               0x0200  /* DSP2_RAM_RDY_EINT1 */
#define CS47L24_DSP2_RAM_RDY_EINT1_MASK          0x0200  /* DSP2_RAM_RDY_EINT1 */
#define CS47L24_DSP2_RAM_RDY_EINT1_SHIFT              9  /* DSP2_RAM_RDY_EINT1 */
#define CS47L24_DSP2_RAM_RDY_EINT1_WIDTH              1  /* DSP2_RAM_RDY_EINT1 */
#define CS47L24_DSP_IRQ8_EINT1                   0x0080  /* DSP_IRQ8_EINT1 */
#define CS47L24_DSP_IRQ8_EINT1_MASK              0x0080  /* DSP_IRQ8_EINT1 */
#define CS47L24_DSP_IRQ8_EINT1_SHIFT                  7  /* DSP_IRQ8_EINT1 */
#define CS47L24_DSP_IRQ8_EINT1_WIDTH                  1  /* DSP_IRQ8_EINT1 */
#define CS47L24_DSP_IRQ7_EINT1                   0x0040  /* DSP_IRQ7_EINT1 */
#define CS47L24_DSP_IRQ7_EINT1_MASK              0x0040  /* DSP_IRQ7_EINT1 */
#define CS47L24_DSP_IRQ7_EINT1_SHIFT                  6  /* DSP_IRQ7_EINT1 */
#define CS47L24_DSP_IRQ7_EINT1_WIDTH                  1  /* DSP_IRQ7_EINT1 */
#define CS47L24_DSP_IRQ6_EINT1                   0x0020  /* DSP_IRQ6_EINT1 */
#define CS47L24_DSP_IRQ6_EINT1_MASK              0x0020  /* DSP_IRQ6_EINT1 */
#define CS47L24_DSP_IRQ6_EINT1_SHIFT                  5  /* DSP_IRQ6_EINT1 */
#define CS47L24_DSP_IRQ6_EINT1_WIDTH                  1  /* DSP_IRQ6_EINT1 */
#define CS47L24_DSP_IRQ5_EINT1                   0x0010  /* DSP_IRQ5_EINT1 */
#define CS47L24_DSP_IRQ5_EINT1_MASK              0x0010  /* DSP_IRQ5_EINT1 */
#define CS47L24_DSP_IRQ5_EINT1_SHIFT                  4  /* DSP_IRQ5_EINT1 */
#define CS47L24_DSP_IRQ5_EINT1_WIDTH                  1  /* DSP_IRQ5_EINT1 */
#define CS47L24_DSP_IRQ4_EINT1                   0x0008  /* DSP_IRQ4_EINT1 */
#define CS47L24_DSP_IRQ4_EINT1_MASK              0x0008  /* DSP_IRQ4_EINT1 */
#define CS47L24_DSP_IRQ4_EINT1_SHIFT                  3  /* DSP_IRQ4_EINT1 */
#define CS47L24_DSP_IRQ4_EINT1_WIDTH                  1  /* DSP_IRQ4_EINT1 */
#define CS47L24_DSP_IRQ3_EINT1                   0x0004  /* DSP_IRQ3_EINT1 */
#define CS47L24_DSP_IRQ3_EINT1_MASK              0x0004  /* DSP_IRQ3_EINT1 */
#define CS47L24_DSP_IRQ3_EINT1_SHIFT                  2  /* DSP_IRQ3_EINT1 */
#define CS47L24_DSP_IRQ3_EINT1_WIDTH                  1  /* DSP_IRQ3_EINT1 */
#define CS47L24_DSP_IRQ2_EINT1                   0x0002  /* DSP_IRQ2_EINT1 */
#define CS47L24_DSP_IRQ2_EINT1_MASK              0x0002  /* DSP_IRQ2_EINT1 */
#define CS47L24_DSP_IRQ2_EINT1_SHIFT                  1  /* DSP_IRQ2_EINT1 */
#define CS47L24_DSP_IRQ2_EINT1_WIDTH                  1  /* DSP_IRQ2_EINT1 */
#define CS47L24_DSP_IRQ1_EINT1                   0x0001  /* DSP_IRQ1_EINT1 */
#define CS47L24_DSP_IRQ1_EINT1_MASK              0x0001  /* DSP_IRQ1_EINT1 */
#define CS47L24_DSP_IRQ1_EINT1_SHIFT                  0  /* DSP_IRQ1_EINT1 */
#define CS47L24_DSP_IRQ1_EINT1_WIDTH                  1  /* DSP_IRQ1_EINT1 */

/*
 * R3330 (0xD02) - Interrupt Status 3
 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT1          0x8000  /* SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT1_MASK     0x8000  /* SPK_OVERHEAD_WARN_EINT1 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT1_SHIFT        15  /* SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT1_WIDTH         1  /* SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_SPK_OVERHEAT_EINT1               0x4000  /* SPK_OVERHEAT_EINT1 */
#define CS47L24_SPK_OVERHEAT_EINT1_MASK          0x4000  /* SPK_OVERHEAT_EINT1 */
#define CS47L24_SPK_OVERHEAT_EINT1_SHIFT             14  /* SPK_OVERHEAT_EINT1 */
#define CS47L24_SPK_OVERHEAT_EINT1_WIDTH              1  /* SPK_OVERHEAT_EINT1 */
#define CS47L24_HPDET_EINT1                      0x2000  /* HPDET_EINT1 */
#define CS47L24_HPDET_EINT1_MASK                 0x2000  /* HPDET_EINT1 */
#define CS47L24_HPDET_EINT1_SHIFT                    13  /* HPDET_EINT1 */
#define CS47L24_HPDET_EINT1_WIDTH                     1  /* HPDET_EINT1 */
#define CS47L24_MICDET_EINT1                     0x1000  /* MICDET_EINT1 */
#define CS47L24_MICDET_EINT1_MASK                0x1000  /* MICDET_EINT1 */
#define CS47L24_MICDET_EINT1_SHIFT                   12  /* MICDET_EINT1 */
#define CS47L24_MICDET_EINT1_WIDTH                    1  /* MICDET_EINT1 */
#define CS47L24_WSEQ_DONE_EINT1                  0x0800  /* WSEQ_DONE_EINT1 */
#define CS47L24_WSEQ_DONE_EINT1_MASK             0x0800  /* WSEQ_DONE_EINT1 */
#define CS47L24_WSEQ_DONE_EINT1_SHIFT                11  /* WSEQ_DONE_EINT1 */
#define CS47L24_WSEQ_DONE_EINT1_WIDTH                 1  /* WSEQ_DONE_EINT1 */
#define CS47L24_DRC2_SIG_DET_EINT1               0x0400  /* DRC2_SIG_DET_EINT1 */
#define CS47L24_DRC2_SIG_DET_EINT1_MASK          0x0400  /* DRC2_SIG_DET_EINT1 */
#define CS47L24_DRC2_SIG_DET_EINT1_SHIFT             10  /* DRC2_SIG_DET_EINT1 */
#define CS47L24_DRC2_SIG_DET_EINT1_WIDTH              1  /* DRC2_SIG_DET_EINT1 */
#define CS47L24_DRC1_SIG_DET_EINT1               0x0200  /* DRC1_SIG_DET_EINT1 */
#define CS47L24_DRC1_SIG_DET_EINT1_MASK          0x0200  /* DRC1_SIG_DET_EINT1 */
#define CS47L24_DRC1_SIG_DET_EINT1_SHIFT              9  /* DRC1_SIG_DET_EINT1 */
#define CS47L24_DRC1_SIG_DET_EINT1_WIDTH              1  /* DRC1_SIG_DET_EINT1 */
#define CS47L24_ASRC2_LOCK_EINT1                 0x0100  /* ASRC2_LOCK_EINT1 */
#define CS47L24_ASRC2_LOCK_EINT1_MASK            0x0100  /* ASRC2_LOCK_EINT1 */
#define CS47L24_ASRC2_LOCK_EINT1_SHIFT                8  /* ASRC2_LOCK_EINT1 */
#define CS47L24_ASRC2_LOCK_EINT1_WIDTH                1  /* ASRC2_LOCK_EINT1 */
#define CS47L24_ASRC1_LOCK_EINT1                 0x0080  /* ASRC1_LOCK_EINT1 */
#define CS47L24_ASRC1_LOCK_EINT1_MASK            0x0080  /* ASRC1_LOCK_EINT1 */
#define CS47L24_ASRC1_LOCK_EINT1_SHIFT                7  /* ASRC1_LOCK_EINT1 */
#define CS47L24_ASRC1_LOCK_EINT1_WIDTH                1  /* ASRC1_LOCK_EINT1 */
#define CS47L24_UNDERCLOCKED_EINT1               0x0040  /* UNDERCLOCKED_EINT1 */
#define CS47L24_UNDERCLOCKED_EINT1_MASK          0x0040  /* UNDERCLOCKED_EINT1 */
#define CS47L24_UNDERCLOCKED_EINT1_SHIFT              6  /* UNDERCLOCKED_EINT1 */
#define CS47L24_UNDERCLOCKED_EINT1_WIDTH              1  /* UNDERCLOCKED_EINT1 */
#define CS47L24_OVERCLOCKED_EINT1                0x0020  /* OVERCLOCKED_EINT1 */
#define CS47L24_OVERCLOCKED_EINT1_MASK           0x0020  /* OVERCLOCKED_EINT1 */
#define CS47L24_OVERCLOCKED_EINT1_SHIFT               5  /* OVERCLOCKED_EINT1 */
#define CS47L24_OVERCLOCKED_EINT1_WIDTH               1  /* OVERCLOCKED_EINT1 */
#define CS47L24_FLL2_LOCK_EINT1                  0x0008  /* FLL2_LOCK_EINT1 */
#define CS47L24_FLL2_LOCK_EINT1_MASK             0x0008  /* FLL2_LOCK_EINT1 */
#define CS47L24_FLL2_LOCK_EINT1_SHIFT                 3  /* FLL2_LOCK_EINT1 */
#define CS47L24_FLL2_LOCK_EINT1_WIDTH                 1  /* FLL2_LOCK_EINT1 */
#define CS47L24_FLL1_LOCK_EINT1                  0x0004  /* FLL1_LOCK_EINT1 */
#define CS47L24_FLL1_LOCK_EINT1_MASK             0x0004  /* FLL1_LOCK_EINT1 */
#define CS47L24_FLL1_LOCK_EINT1_SHIFT                 2  /* FLL1_LOCK_EINT1 */
#define CS47L24_FLL1_LOCK_EINT1_WIDTH                 1  /* FLL1_LOCK_EINT1 */
#define CS47L24_CLKGEN_ERR_EINT1                 0x0002  /* CLKGEN_ERR_EINT1 */
#define CS47L24_CLKGEN_ERR_EINT1_MASK            0x0002  /* CLKGEN_ERR_EINT1 */
#define CS47L24_CLKGEN_ERR_EINT1_SHIFT                1  /* CLKGEN_ERR_EINT1 */
#define CS47L24_CLKGEN_ERR_EINT1_WIDTH                1  /* CLKGEN_ERR_EINT1 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT1           0x0001  /* CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT1_MASK      0x0001  /* CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT1_SHIFT          0  /* CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT1_WIDTH          1  /* CLKGEN_ERR_ASYNC_EINT1 */

/*
 * R3331 (0xD03) - Interrupt Status 4
 */
#define CS47L24_ASRC_CFG_ERR_EINT1               0x8000  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_ASRC_CFG_ERR_EINT1_MASK          0x8000  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_ASRC_CFG_ERR_EINT1_SHIFT             15  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_ASRC_CFG_ERR_EINT1_WIDTH              1  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_AIF3_ERR_EINT1                   0x4000  /* AIF3_ERR_EINT1 */
#define CS47L24_AIF3_ERR_EINT1_MASK              0x4000  /* AIF3_ERR_EINT1 */
#define CS47L24_AIF3_ERR_EINT1_SHIFT                 14  /* AIF3_ERR_EINT1 */
#define CS47L24_AIF3_ERR_EINT1_WIDTH                  1  /* AIF3_ERR_EINT1 */
#define CS47L24_AIF2_ERR_EINT1                   0x2000  /* AIF2_ERR_EINT1 */
#define CS47L24_AIF2_ERR_EINT1_MASK              0x2000  /* AIF2_ERR_EINT1 */
#define CS47L24_AIF2_ERR_EINT1_SHIFT                 13  /* AIF2_ERR_EINT1 */
#define CS47L24_AIF2_ERR_EINT1_WIDTH                  1  /* AIF2_ERR_EINT1 */
#define CS47L24_AIF1_ERR_EINT1                   0x1000  /* AIF1_ERR_EINT1 */
#define CS47L24_AIF1_ERR_EINT1_MASK              0x1000  /* AIF1_ERR_EINT1 */
#define CS47L24_AIF1_ERR_EINT1_SHIFT                 12  /* AIF1_ERR_EINT1 */
#define CS47L24_AIF1_ERR_EINT1_WIDTH                  1  /* AIF1_ERR_EINT1 */
#define CS47L24_CTRLIF_ERR_EINT1                 0x0800  /* CTRLIF_ERR_EINT1 */
#define CS47L24_CTRLIF_ERR_EINT1_MASK            0x0800  /* CTRLIF_ERR_EINT1 */
#define CS47L24_CTRLIF_ERR_EINT1_SHIFT               11  /* CTRLIF_ERR_EINT1 */
#define CS47L24_CTRLIF_ERR_EINT1_WIDTH                1  /* CTRLIF_ERR_EINT1 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT1       0x0400  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT1_MASK  0x0400  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT1_SHIFT     10  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT1_WIDTH      1  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT1          0x0200  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT1_MASK     0x0200  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT1_SHIFT         9  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT1_WIDTH         1  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_SYSCLK_ENA_LOW_EINT1             0x0100  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_SYSCLK_ENA_LOW_EINT1_MASK        0x0100  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_SYSCLK_ENA_LOW_EINT1_SHIFT            8  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_SYSCLK_ENA_LOW_EINT1_WIDTH            1  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_ISRC1_CFG_ERR_EINT1              0x0080  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_ISRC1_CFG_ERR_EINT1_MASK         0x0080  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_ISRC1_CFG_ERR_EINT1_SHIFT             7  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_ISRC1_CFG_ERR_EINT1_WIDTH             1  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_ISRC2_CFG_ERR_EINT1              0x0040  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_ISRC2_CFG_ERR_EINT1_MASK         0x0040  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_ISRC2_CFG_ERR_EINT1_SHIFT             6  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_ISRC2_CFG_ERR_EINT1_WIDTH             1  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_HP1R_DONE_EINT1                  0x0002  /* HP1R_DONE_EINT1 */
#define CS47L24_HP1R_DONE_EINT1_MASK             0x0002  /* HP1R_DONE_EINT1 */
#define CS47L24_HP1R_DONE_EINT1_SHIFT                 1  /* HP1R_DONE_EINT1 */
#define CS47L24_HP1R_DONE_EINT1_WIDTH                 1  /* HP1R_DONE_EINT1 */
#define CS47L24_HP1L_DONE_EINT1                  0x0001  /* HP1L_DONE_EINT1 */
#define CS47L24_HP1L_DONE_EINT1_MASK             0x0001  /* HP1L_DONE_EINT1 */
#define CS47L24_HP1L_DONE_EINT1_SHIFT                 0  /* HP1L_DONE_EINT1 */
#define CS47L24_HP1L_DONE_EINT1_WIDTH                 1  /* HP1L_DONE_EINT1 */

/*
 * R3331 (0xD03) - Interrupt Status 4 (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_AIF3_ERR_EINT1                  0x8000  /* AIF3_ERR_EINT1 */
#define CS47L24_V2_AIF3_ERR_EINT1_MASK             0x8000  /* AIF3_ERR_EINT1 */
#define CS47L24_V2_AIF3_ERR_EINT1_SHIFT                15  /* AIF3_ERR_EINT1 */
#define CS47L24_V2_AIF3_ERR_EINT1_WIDTH                 1  /* AIF3_ERR_EINT1 */
#define CS47L24_V2_AIF2_ERR_EINT1                  0x4000  /* AIF2_ERR_EINT1 */
#define CS47L24_V2_AIF2_ERR_EINT1_MASK             0x4000  /* AIF2_ERR_EINT1 */
#define CS47L24_V2_AIF2_ERR_EINT1_SHIFT                14  /* AIF2_ERR_EINT1 */
#define CS47L24_V2_AIF2_ERR_EINT1_WIDTH                 1  /* AIF2_ERR_EINT1 */
#define CS47L24_V2_AIF1_ERR_EINT1                  0x2000  /* AIF1_ERR_EINT1 */
#define CS47L24_V2_AIF1_ERR_EINT1_MASK             0x2000  /* AIF1_ERR_EINT1 */
#define CS47L24_V2_AIF1_ERR_EINT1_SHIFT                13  /* AIF1_ERR_EINT1 */
#define CS47L24_V2_AIF1_ERR_EINT1_WIDTH                 1  /* AIF1_ERR_EINT1 */
#define CS47L24_V2_CTRLIF_ERR_EINT1                0x1000  /* CTRLIF_ERR_EINT1 */
#define CS47L24_V2_CTRLIF_ERR_EINT1_MASK           0x1000  /* CTRLIF_ERR_EINT1 */
#define CS47L24_V2_CTRLIF_ERR_EINT1_SHIFT              12  /* CTRLIF_ERR_EINT1 */
#define CS47L24_V2_CTRLIF_ERR_EINT1_WIDTH               1  /* CTRLIF_ERR_EINT1 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT1      0x0800  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT1_MASK 0x0800  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT1_SHIFT    11  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT1_WIDTH     1  /* MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT1         0x0400  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT1_MASK    0x0400  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT1_SHIFT       10  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT1_WIDTH        1  /* ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT1            0x0200  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT1_MASK       0x0200  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT1_SHIFT           9  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT1_WIDTH           1  /* SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT1             0x0100  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT1_MASK        0x0100  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT1_SHIFT            8  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT1_WIDTH            1  /* ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT1             0x0080  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT1_MASK        0x0080  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT1_SHIFT            7  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT1_WIDTH            1  /* ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT1             0x0040  /* ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT1_MASK        0x0040  /* ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT1_SHIFT            6  /* ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT1_WIDTH            1  /* ISRC3_CFG_ERR_EINT1 */

/*
 * R3332 (0xD04) - Interrupt Status 5
 */
#define CS47L24_BOOT_DONE_EINT1                  0x0100  /* BOOT_DONE_EINT1 */
#define CS47L24_BOOT_DONE_EINT1_MASK             0x0100  /* BOOT_DONE_EINT1 */
#define CS47L24_BOOT_DONE_EINT1_SHIFT                 8  /* BOOT_DONE_EINT1 */
#define CS47L24_BOOT_DONE_EINT1_WIDTH                 1  /* BOOT_DONE_EINT1 */
#define CS47L24_DCS_DAC_DONE_EINT1               0x0080  /* DCS_DAC_DONE_EINT1 */
#define CS47L24_DCS_DAC_DONE_EINT1_MASK          0x0080  /* DCS_DAC_DONE_EINT1 */
#define CS47L24_DCS_DAC_DONE_EINT1_SHIFT              7  /* DCS_DAC_DONE_EINT1 */
#define CS47L24_DCS_DAC_DONE_EINT1_WIDTH              1  /* DCS_DAC_DONE_EINT1 */
#define CS47L24_DCS_HP_DONE_EINT1                0x0040  /* DCS_HP_DONE_EINT1 */
#define CS47L24_DCS_HP_DONE_EINT1_MASK           0x0040  /* DCS_HP_DONE_EINT1 */
#define CS47L24_DCS_HP_DONE_EINT1_SHIFT               6  /* DCS_HP_DONE_EINT1 */
#define CS47L24_DCS_HP_DONE_EINT1_WIDTH               1  /* DCS_HP_DONE_EINT1 */
#define CS47L24_FLL2_CLOCK_OK_EINT1              0x0002  /* FLL2_CLOCK_OK_EINT1 */
#define CS47L24_FLL2_CLOCK_OK_EINT1_MASK         0x0002  /* FLL2_CLOCK_OK_EINT1 */
#define CS47L24_FLL2_CLOCK_OK_EINT1_SHIFT             1  /* FLL2_CLOCK_OK_EINT1 */
#define CS47L24_FLL2_CLOCK_OK_EINT1_WIDTH             1  /* FLL2_CLOCK_OK_EINT1 */
#define CS47L24_FLL1_CLOCK_OK_EINT1              0x0001  /* FLL1_CLOCK_OK_EINT1 */
#define CS47L24_FLL1_CLOCK_OK_EINT1_MASK         0x0001  /* FLL1_CLOCK_OK_EINT1 */
#define CS47L24_FLL1_CLOCK_OK_EINT1_SHIFT             0  /* FLL1_CLOCK_OK_EINT1 */
#define CS47L24_FLL1_CLOCK_OK_EINT1_WIDTH             1  /* FLL1_CLOCK_OK_EINT1 */

/*
 * R3332 (0xD05) - Interrupt Status 5 (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT1            0x0008  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT1_MASK       0x0008  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT1_SHIFT           3  /* ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT1_WIDTH           1  /* ASRC_CFG_ERR_EINT1 */

/*
 * R3333 (0xD05) - Interrupt Status 6
 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT1         0x8000  /* DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT1_MASK    0x8000  /* DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT1_SHIFT       15  /* DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT1_WIDTH        1  /* DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_SPK_SHUTDOWN_EINT1               0x4000  /* SPK_SHUTDOWN_EINT1 */
#define CS47L24_SPK_SHUTDOWN_EINT1_MASK          0x4000  /* SPK_SHUTDOWN_EINT1 */
#define CS47L24_SPK_SHUTDOWN_EINT1_SHIFT             14  /* SPK_SHUTDOWN_EINT1 */
#define CS47L24_SPK_SHUTDOWN_EINT1_WIDTH              1  /* SPK_SHUTDOWN_EINT1 */
#define CS47L24_SPK1R_SHORT_EINT1                0x2000  /* SPK1R_SHORT_EINT1 */
#define CS47L24_SPK1R_SHORT_EINT1_MASK           0x2000  /* SPK1R_SHORT_EINT1 */
#define CS47L24_SPK1R_SHORT_EINT1_SHIFT              13  /* SPK1R_SHORT_EINT1 */
#define CS47L24_SPK1R_SHORT_EINT1_WIDTH               1  /* SPK1R_SHORT_EINT1 */
#define CS47L24_SPK1L_SHORT_EINT1                0x1000  /* SPK1L_SHORT_EINT1 */
#define CS47L24_SPK1L_SHORT_EINT1_MASK           0x1000  /* SPK1L_SHORT_EINT1 */
#define CS47L24_SPK1L_SHORT_EINT1_SHIFT              12  /* SPK1L_SHORT_EINT1 */
#define CS47L24_SPK1L_SHORT_EINT1_WIDTH               1  /* SPK1L_SHORT_EINT1 */
#define CS47L24_HP1R_SC_NEG_EINT1                0x0008  /* HP1R_SC_NEG_EINT1 */
#define CS47L24_HP1R_SC_NEG_EINT1_MASK           0x0008  /* HP1R_SC_NEG_EINT1 */
#define CS47L24_HP1R_SC_NEG_EINT1_SHIFT               3  /* HP1R_SC_NEG_EINT1 */
#define CS47L24_HP1R_SC_NEG_EINT1_WIDTH               1  /* HP1R_SC_NEG_EINT1 */
#define CS47L24_HP1R_SC_POS_EINT1                0x0004  /* HP1R_SC_POS_EINT1 */
#define CS47L24_HP1R_SC_POS_EINT1_MASK           0x0004  /* HP1R_SC_POS_EINT1 */
#define CS47L24_HP1R_SC_POS_EINT1_SHIFT               2  /* HP1R_SC_POS_EINT1 */
#define CS47L24_HP1R_SC_POS_EINT1_WIDTH               1  /* HP1R_SC_POS_EINT1 */
#define CS47L24_HP1L_SC_NEG_EINT1                0x0002  /* HP1L_SC_NEG_EINT1 */
#define CS47L24_HP1L_SC_NEG_EINT1_MASK           0x0002  /* HP1L_SC_NEG_EINT1 */
#define CS47L24_HP1L_SC_NEG_EINT1_SHIFT               1  /* HP1L_SC_NEG_EINT1 */
#define CS47L24_HP1L_SC_NEG_EINT1_WIDTH               1  /* HP1L_SC_NEG_EINT1 */
#define CS47L24_HP1L_SC_POS_EINT1                0x0001  /* HP1L_SC_POS_EINT1 */
#define CS47L24_HP1L_SC_POS_EINT1_MASK           0x0001  /* HP1L_SC_POS_EINT1 */
#define CS47L24_HP1L_SC_POS_EINT1_SHIFT               0  /* HP1L_SC_POS_EINT1 */
#define CS47L24_HP1L_SC_POS_EINT1_WIDTH               1  /* HP1L_SC_POS_EINT1 */

/*
 * R3336 (0xD08) - Interrupt Status 1 Mask
 */
#define CS47L24_IM_GP2_EINT1                     0x0002  /* IM_GP2_EINT1 */
#define CS47L24_IM_GP2_EINT1_MASK                0x0002  /* IM_GP2_EINT1 */
#define CS47L24_IM_GP2_EINT1_SHIFT                    1  /* IM_GP2_EINT1 */
#define CS47L24_IM_GP2_EINT1_WIDTH                    1  /* IM_GP2_EINT1 */
#define CS47L24_IM_GP1_EINT1                     0x0001  /* IM_GP1_EINT1 */
#define CS47L24_IM_GP1_EINT1_MASK                0x0001  /* IM_GP1_EINT1 */
#define CS47L24_IM_GP1_EINT1_SHIFT                    0  /* IM_GP1_EINT1 */
#define CS47L24_IM_GP1_EINT1_WIDTH                    1  /* IM_GP1_EINT1 */

/*
 * R3337 (0xD09) - Interrupt Status 2 Mask
 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT1            0x0100  /* IM_DSP1_RAM_RDY_EINT1 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT1_MASK       0x0100  /* IM_DSP1_RAM_RDY_EINT1 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT1_SHIFT           8  /* IM_DSP1_RAM_RDY_EINT1 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT1_WIDTH           1  /* IM_DSP1_RAM_RDY_EINT1 */
#define CS47L24_IM_DSP_IRQ2_EINT1                0x0002  /* IM_DSP_IRQ2_EINT1 */
#define CS47L24_IM_DSP_IRQ2_EINT1_MASK           0x0002  /* IM_DSP_IRQ2_EINT1 */
#define CS47L24_IM_DSP_IRQ2_EINT1_SHIFT               1  /* IM_DSP_IRQ2_EINT1 */
#define CS47L24_IM_DSP_IRQ2_EINT1_WIDTH               1  /* IM_DSP_IRQ2_EINT1 */
#define CS47L24_IM_DSP_IRQ1_EINT1                0x0001  /* IM_DSP_IRQ1_EINT1 */
#define CS47L24_IM_DSP_IRQ1_EINT1_MASK           0x0001  /* IM_DSP_IRQ1_EINT1 */
#define CS47L24_IM_DSP_IRQ1_EINT1_SHIFT               0  /* IM_DSP_IRQ1_EINT1 */
#define CS47L24_IM_DSP_IRQ1_EINT1_WIDTH               1  /* IM_DSP_IRQ1_EINT1 */

/*
 * R3338 (0xD0A) - Interrupt Status 3 Mask
 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT1       0x8000  /* IM_SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT1_MASK  0x8000  /* IM_SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT1_SHIFT     15  /* IM_SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT1_WIDTH      1  /* IM_SPK_OVERHEAT_WARN_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_EINT1            0x4000  /* IM_SPK_OVERHEAT_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_EINT1_MASK       0x4000  /* IM_SPK_OVERHEAT_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_EINT1_SHIFT          14  /* IM_SPK_OVERHEAT_EINT1 */
#define CS47L24_IM_SPK_OVERHEAT_EINT1_WIDTH           1  /* IM_SPK_OVERHEAT_EINT1 */
#define CS47L24_IM_HPDET_EINT1                   0x2000  /* IM_HPDET_EINT1 */
#define CS47L24_IM_HPDET_EINT1_MASK              0x2000  /* IM_HPDET_EINT1 */
#define CS47L24_IM_HPDET_EINT1_SHIFT                 13  /* IM_HPDET_EINT1 */
#define CS47L24_IM_HPDET_EINT1_WIDTH                  1  /* IM_HPDET_EINT1 */
#define CS47L24_IM_MICDET_EINT1                  0x1000  /* IM_MICDET_EINT1 */
#define CS47L24_IM_MICDET_EINT1_MASK             0x1000  /* IM_MICDET_EINT1 */
#define CS47L24_IM_MICDET_EINT1_SHIFT                12  /* IM_MICDET_EINT1 */
#define CS47L24_IM_MICDET_EINT1_WIDTH                 1  /* IM_MICDET_EINT1 */
#define CS47L24_IM_WSEQ_DONE_EINT1               0x0800  /* IM_WSEQ_DONE_EINT1 */
#define CS47L24_IM_WSEQ_DONE_EINT1_MASK          0x0800  /* IM_WSEQ_DONE_EINT1 */
#define CS47L24_IM_WSEQ_DONE_EINT1_SHIFT             11  /* IM_WSEQ_DONE_EINT1 */
#define CS47L24_IM_WSEQ_DONE_EINT1_WIDTH              1  /* IM_WSEQ_DONE_EINT1 */
#define CS47L24_IM_DRC2_SIG_DET_EINT1            0x0400  /* IM_DRC2_SIG_DET_EINT1 */
#define CS47L24_IM_DRC2_SIG_DET_EINT1_MASK       0x0400  /* IM_DRC2_SIG_DET_EINT1 */
#define CS47L24_IM_DRC2_SIG_DET_EINT1_SHIFT          10  /* IM_DRC2_SIG_DET_EINT1 */
#define CS47L24_IM_DRC2_SIG_DET_EINT1_WIDTH           1  /* IM_DRC2_SIG_DET_EINT1 */
#define CS47L24_IM_DRC1_SIG_DET_EINT1            0x0200  /* IM_DRC1_SIG_DET_EINT1 */
#define CS47L24_IM_DRC1_SIG_DET_EINT1_MASK       0x0200  /* IM_DRC1_SIG_DET_EINT1 */
#define CS47L24_IM_DRC1_SIG_DET_EINT1_SHIFT           9  /* IM_DRC1_SIG_DET_EINT1 */
#define CS47L24_IM_DRC1_SIG_DET_EINT1_WIDTH           1  /* IM_DRC1_SIG_DET_EINT1 */
#define CS47L24_IM_ASRC2_LOCK_EINT1              0x0100  /* IM_ASRC2_LOCK_EINT1 */
#define CS47L24_IM_ASRC2_LOCK_EINT1_MASK         0x0100  /* IM_ASRC2_LOCK_EINT1 */
#define CS47L24_IM_ASRC2_LOCK_EINT1_SHIFT             8  /* IM_ASRC2_LOCK_EINT1 */
#define CS47L24_IM_ASRC2_LOCK_EINT1_WIDTH             1  /* IM_ASRC2_LOCK_EINT1 */
#define CS47L24_IM_ASRC1_LOCK_EINT1              0x0080  /* IM_ASRC1_LOCK_EINT1 */
#define CS47L24_IM_ASRC1_LOCK_EINT1_MASK         0x0080  /* IM_ASRC1_LOCK_EINT1 */
#define CS47L24_IM_ASRC1_LOCK_EINT1_SHIFT             7  /* IM_ASRC1_LOCK_EINT1 */
#define CS47L24_IM_ASRC1_LOCK_EINT1_WIDTH             1  /* IM_ASRC1_LOCK_EINT1 */
#define CS47L24_IM_UNDERCLOCKED_EINT1            0x0040  /* IM_UNDERCLOCKED_EINT1 */
#define CS47L24_IM_UNDERCLOCKED_EINT1_MASK       0x0040  /* IM_UNDERCLOCKED_EINT1 */
#define CS47L24_IM_UNDERCLOCKED_EINT1_SHIFT           6  /* IM_UNDERCLOCKED_EINT1 */
#define CS47L24_IM_UNDERCLOCKED_EINT1_WIDTH           1  /* IM_UNDERCLOCKED_EINT1 */
#define CS47L24_IM_OVERCLOCKED_EINT1             0x0020  /* IM_OVERCLOCKED_EINT1 */
#define CS47L24_IM_OVERCLOCKED_EINT1_MASK        0x0020  /* IM_OVERCLOCKED_EINT1 */
#define CS47L24_IM_OVERCLOCKED_EINT1_SHIFT            5  /* IM_OVERCLOCKED_EINT1 */
#define CS47L24_IM_OVERCLOCKED_EINT1_WIDTH            1  /* IM_OVERCLOCKED_EINT1 */
#define CS47L24_IM_FLL2_LOCK_EINT1               0x0008  /* IM_FLL2_LOCK_EINT1 */
#define CS47L24_IM_FLL2_LOCK_EINT1_MASK          0x0008  /* IM_FLL2_LOCK_EINT1 */
#define CS47L24_IM_FLL2_LOCK_EINT1_SHIFT              3  /* IM_FLL2_LOCK_EINT1 */
#define CS47L24_IM_FLL2_LOCK_EINT1_WIDTH              1  /* IM_FLL2_LOCK_EINT1 */
#define CS47L24_IM_FLL1_LOCK_EINT1               0x0004  /* IM_FLL1_LOCK_EINT1 */
#define CS47L24_IM_FLL1_LOCK_EINT1_MASK          0x0004  /* IM_FLL1_LOCK_EINT1 */
#define CS47L24_IM_FLL1_LOCK_EINT1_SHIFT              2  /* IM_FLL1_LOCK_EINT1 */
#define CS47L24_IM_FLL1_LOCK_EINT1_WIDTH              1  /* IM_FLL1_LOCK_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_EINT1              0x0002  /* IM_CLKGEN_ERR_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_EINT1_MASK         0x0002  /* IM_CLKGEN_ERR_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_EINT1_SHIFT             1  /* IM_CLKGEN_ERR_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_EINT1_WIDTH             1  /* IM_CLKGEN_ERR_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT1        0x0001  /* IM_CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT1_MASK   0x0001  /* IM_CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT1_SHIFT       0  /* IM_CLKGEN_ERR_ASYNC_EINT1 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT1_WIDTH       1  /* IM_CLKGEN_ERR_ASYNC_EINT1 */

/*
 * R3339 (0xD0B) - Interrupt Status 4 Mask
 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT1                 0x8000  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT1_MASK            0x8000  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT1_SHIFT               15  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT1_WIDTH                1  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_IM_AIF3_ERR_EINT1                     0x4000  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_IM_AIF3_ERR_EINT1_MASK                0x4000  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_IM_AIF3_ERR_EINT1_SHIFT                   14  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_IM_AIF3_ERR_EINT1_WIDTH                    1  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_IM_AIF2_ERR_EINT1                     0x2000  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_IM_AIF2_ERR_EINT1_MASK                0x2000  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_IM_AIF2_ERR_EINT1_SHIFT                   13  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_IM_AIF2_ERR_EINT1_WIDTH                    1  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_IM_AIF1_ERR_EINT1                     0x1000  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_IM_AIF1_ERR_EINT1_MASK                0x1000  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_IM_AIF1_ERR_EINT1_SHIFT                   12  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_IM_AIF1_ERR_EINT1_WIDTH                    1  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_IM_CTRLIF_ERR_EINT1                   0x0800  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_IM_CTRLIF_ERR_EINT1_MASK              0x0800  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_IM_CTRLIF_ERR_EINT1_SHIFT                 11  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_IM_CTRLIF_ERR_EINT1_WIDTH                  1  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT1         0x0400  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT1_MASK    0x0400  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT1_SHIFT       10  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT1_WIDTH        1  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT1            0x0200  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT1_MASK       0x0200  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT1_SHIFT           9  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT1_WIDTH           1  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT1               0x0100  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT1_MASK          0x0100  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT1_SHIFT              8  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT1_WIDTH              1  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT1                0x0080  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT1_MASK           0x0080  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT1_SHIFT               7  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT1_WIDTH               1  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT1                0x0040  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT1_MASK           0x0040  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT1_SHIFT               6  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT1_WIDTH               1  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_IM_HP1R_DONE_EINT1                    0x0002  /* IM_HP1R_DONE_EINT1 */
#define CS47L24_IM_HP1R_DONE_EINT1_MASK               0x0002  /* IM_HP1R_DONE_EINT1 */
#define CS47L24_IM_HP1R_DONE_EINT1_SHIFT                   1  /* IM_HP1R_DONE_EINT1 */
#define CS47L24_IM_HP1R_DONE_EINT1_WIDTH                   1  /* IM_HP1R_DONE_EINT1 */
#define CS47L24_IM_HP1L_DONE_EINT1                    0x0001  /* IM_HP1L_DONE_EINT1 */
#define CS47L24_IM_HP1L_DONE_EINT1_MASK               0x0001  /* IM_HP1L_DONE_EINT1 */
#define CS47L24_IM_HP1L_DONE_EINT1_SHIFT                   0  /* IM_HP1L_DONE_EINT1 */
#define CS47L24_IM_HP1L_DONE_EINT1_WIDTH                   1  /* IM_HP1L_DONE_EINT1 */

/*
 * R3339 (0xD0B) - Interrupt Status 4 Mask (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_IM_AIF3_ERR_EINT1                  0x8000  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_V2_IM_AIF3_ERR_EINT1_MASK             0x8000  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_V2_IM_AIF3_ERR_EINT1_SHIFT                15  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_V2_IM_AIF3_ERR_EINT1_WIDTH                 1  /* IM_AIF3_ERR_EINT1 */
#define CS47L24_V2_IM_AIF2_ERR_EINT1                  0x4000  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_V2_IM_AIF2_ERR_EINT1_MASK             0x4000  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_V2_IM_AIF2_ERR_EINT1_SHIFT                14  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_V2_IM_AIF2_ERR_EINT1_WIDTH                 1  /* IM_AIF2_ERR_EINT1 */
#define CS47L24_V2_IM_AIF1_ERR_EINT1                  0x2000  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_V2_IM_AIF1_ERR_EINT1_MASK             0x2000  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_V2_IM_AIF1_ERR_EINT1_SHIFT                13  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_V2_IM_AIF1_ERR_EINT1_WIDTH                 1  /* IM_AIF1_ERR_EINT1 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT1                0x1000  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT1_MASK           0x1000  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT1_SHIFT              12  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT1_WIDTH               1  /* IM_CTRLIF_ERR_EINT1 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT1      0x0800  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT1_MASK 0x0800  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT1_SHIFT    11  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT1_WIDTH     1  /* IM_MIXER_DROPPED_SAMPLE_EINT1 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT1         0x0400  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT1_MASK    0x0400  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT1_SHIFT       10  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT1_WIDTH        1  /* IM_ASYNC_CLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT1            0x0200  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT1_MASK       0x0200  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT1_SHIFT           9  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT1_WIDTH           1  /* IM_SYSCLK_ENA_LOW_EINT1 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT1             0x0100  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT1_MASK        0x0100  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT1_SHIFT            8  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT1_WIDTH            1  /* IM_ISRC1_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT1             0x0080  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT1_MASK        0x0080  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT1_SHIFT            7  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT1_WIDTH            1  /* IM_ISRC2_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT1             0x0040  /* IM_ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT1_MASK        0x0040  /* IM_ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT1_SHIFT            6  /* IM_ISRC3_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT1_WIDTH            1  /* IM_ISRC3_CFG_ERR_EINT1 */

/*
 * R3340 (0xD0C) - Interrupt Status 5 Mask
 */
#define CS47L24_IM_BOOT_DONE_EINT1               0x0100  /* IM_BOOT_DONE_EINT1 */
#define CS47L24_IM_BOOT_DONE_EINT1_MASK          0x0100  /* IM_BOOT_DONE_EINT1 */
#define CS47L24_IM_BOOT_DONE_EINT1_SHIFT              8  /* IM_BOOT_DONE_EINT1 */
#define CS47L24_IM_BOOT_DONE_EINT1_WIDTH              1  /* IM_BOOT_DONE_EINT1 */
#define CS47L24_IM_DCS_DAC_DONE_EINT1            0x0080  /* IM_DCS_DAC_DONE_EINT1 */
#define CS47L24_IM_DCS_DAC_DONE_EINT1_MASK       0x0080  /* IM_DCS_DAC_DONE_EINT1 */
#define CS47L24_IM_DCS_DAC_DONE_EINT1_SHIFT           7  /* IM_DCS_DAC_DONE_EINT1 */
#define CS47L24_IM_DCS_DAC_DONE_EINT1_WIDTH           1  /* IM_DCS_DAC_DONE_EINT1 */
#define CS47L24_IM_DCS_HP_DONE_EINT1             0x0040  /* IM_DCS_HP_DONE_EINT1 */
#define CS47L24_IM_DCS_HP_DONE_EINT1_MASK        0x0040  /* IM_DCS_HP_DONE_EINT1 */
#define CS47L24_IM_DCS_HP_DONE_EINT1_SHIFT            6  /* IM_DCS_HP_DONE_EINT1 */
#define CS47L24_IM_DCS_HP_DONE_EINT1_WIDTH            1  /* IM_DCS_HP_DONE_EINT1 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT1           0x0002  /* IM_FLL2_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT1_MASK      0x0002  /* IM_FLL2_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT1_SHIFT          1  /* IM_FLL2_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT1_WIDTH          1  /* IM_FLL2_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT1           0x0001  /* IM_FLL1_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT1_MASK      0x0001  /* IM_FLL1_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT1_SHIFT          0  /* IM_FLL1_CLOCK_OK_EINT1 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT1_WIDTH          1  /* IM_FLL1_CLOCK_OK_EINT1 */

/*
 * R3340 (0xD0C) - Interrupt Status 5 Mask (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT1         0x0008  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT1_MASK    0x0008  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT1_SHIFT        3  /* IM_ASRC_CFG_ERR_EINT1 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT1_WIDTH        1  /* IM_ASRC_CFG_ERR_EINT1 */

/*
 * R3341 (0xD0D) - Interrupt Status 6 Mask
 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT1      0x8000  /* IM_DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT1_MASK 0x8000  /* IM_DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT1_SHIFT    15  /* IM_DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT1_WIDTH     1  /* IM_DSP_SHARED_WR_COLL_EINT1 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT1            0x4000  /* IM_SPK_SHUTDOWN_EINT1 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT1_MASK       0x4000  /* IM_SPK_SHUTDOWN_EINT1 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT1_SHIFT          14  /* IM_SPK_SHUTDOWN_EINT1 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT1_WIDTH           1  /* IM_SPK_SHUTDOWN_EINT1 */
#define CS47L24_IM_SPK1R_SHORT_EINT1             0x2000  /* IM_SPK1R_SHORT_EINT1 */
#define CS47L24_IM_SPK1R_SHORT_EINT1_MASK        0x2000  /* IM_SPK1R_SHORT_EINT1 */
#define CS47L24_IM_SPK1R_SHORT_EINT1_SHIFT           13  /* IM_SPK1R_SHORT_EINT1 */
#define CS47L24_IM_SPK1R_SHORT_EINT1_WIDTH            1  /* IM_SPK1R_SHORT_EINT1 */
#define CS47L24_IM_SPK1L_SHORT_EINT1             0x1000  /* IM_SPK1L_SHORT_EINT1 */
#define CS47L24_IM_SPK1L_SHORT_EINT1_MASK        0x1000  /* IM_SPK1L_SHORT_EINT1 */
#define CS47L24_IM_SPK1L_SHORT_EINT1_SHIFT           12  /* IM_SPK1L_SHORT_EINT1 */
#define CS47L24_IM_SPK1L_SHORT_EINT1_WIDTH            1  /* IM_SPK1L_SHORT_EINT1 */
#define CS47L24_IM_HP1R_SC_NEG_EINT1             0x0008  /* IM_HP1R_SC_NEG_EINT1 */
#define CS47L24_IM_HP1R_SC_NEG_EINT1_MASK        0x0008  /* IM_HP1R_SC_NEG_EINT1 */
#define CS47L24_IM_HP1R_SC_NEG_EINT1_SHIFT            3  /* IM_HP1R_SC_NEG_EINT1 */
#define CS47L24_IM_HP1R_SC_NEG_EINT1_WIDTH            1  /* IM_HP1R_SC_NEG_EINT1 */
#define CS47L24_IM_HP1R_SC_POS_EINT1             0x0004  /* IM_HP1R_SC_POS_EINT1 */
#define CS47L24_IM_HP1R_SC_POS_EINT1_MASK        0x0004  /* IM_HP1R_SC_POS_EINT1 */
#define CS47L24_IM_HP1R_SC_POS_EINT1_SHIFT            2  /* IM_HP1R_SC_POS_EINT1 */
#define CS47L24_IM_HP1R_SC_POS_EINT1_WIDTH            1  /* IM_HP1R_SC_POS_EINT1 */
#define CS47L24_IM_HP1L_SC_NEG_EINT1             0x0002  /* IM_HP1L_SC_NEG_EINT1 */
#define CS47L24_IM_HP1L_SC_NEG_EINT1_MASK        0x0002  /* IM_HP1L_SC_NEG_EINT1 */
#define CS47L24_IM_HP1L_SC_NEG_EINT1_SHIFT            1  /* IM_HP1L_SC_NEG_EINT1 */
#define CS47L24_IM_HP1L_SC_NEG_EINT1_WIDTH            1  /* IM_HP1L_SC_NEG_EINT1 */
#define CS47L24_IM_HP1L_SC_POS_EINT1             0x0001  /* IM_HP1L_SC_POS_EINT1 */
#define CS47L24_IM_HP1L_SC_POS_EINT1_MASK        0x0001  /* IM_HP1L_SC_POS_EINT1 */
#define CS47L24_IM_HP1L_SC_POS_EINT1_SHIFT            0  /* IM_HP1L_SC_POS_EINT1 */
#define CS47L24_IM_HP1L_SC_POS_EINT1_WIDTH            1  /* IM_HP1L_SC_POS_EINT1 */

/*
 * R3343 (0xD0F) - Interrupt Control
 */
#define CS47L24_IM_IRQ1                          0x0001  /* IM_IRQ1 */
#define CS47L24_IM_IRQ1_MASK                     0x0001  /* IM_IRQ1 */
#define CS47L24_IM_IRQ1_SHIFT                         0  /* IM_IRQ1 */
#define CS47L24_IM_IRQ1_WIDTH                         1  /* IM_IRQ1 */

/*
 * R3344 (0xD10) - IRQ2 Status 1
 */
#define CS47L24_GP2_EINT2                        0x0002  /* GP2_EINT2 */
#define CS47L24_GP2_EINT2_MASK                   0x0002  /* GP2_EINT2 */
#define CS47L24_GP2_EINT2_SHIFT                       1  /* GP2_EINT2 */
#define CS47L24_GP2_EINT2_WIDTH                       1  /* GP2_EINT2 */
#define CS47L24_GP1_EINT2                        0x0001  /* GP1_EINT2 */
#define CS47L24_GP1_EINT2_MASK                   0x0001  /* GP1_EINT2 */
#define CS47L24_GP1_EINT2_SHIFT                       0  /* GP1_EINT2 */
#define CS47L24_GP1_EINT2_WIDTH                       1  /* GP1_EINT2 */

/*
 * R3345 (0xD11) - IRQ2 Status 2
 */
#define CS47L24_DSP1_RAM_RDY_EINT2               0x0100  /* DSP1_RAM_RDY_EINT2 */
#define CS47L24_DSP1_RAM_RDY_EINT2_MASK          0x0100  /* DSP1_RAM_RDY_EINT2 */
#define CS47L24_DSP1_RAM_RDY_EINT2_SHIFT              8  /* DSP1_RAM_RDY_EINT2 */
#define CS47L24_DSP1_RAM_RDY_EINT2_WIDTH              1  /* DSP1_RAM_RDY_EINT2 */
#define CS47L24_DSP_IRQ2_EINT2                   0x0002  /* DSP_IRQ2_EINT2 */
#define CS47L24_DSP_IRQ2_EINT2_MASK              0x0002  /* DSP_IRQ2_EINT2 */
#define CS47L24_DSP_IRQ2_EINT2_SHIFT                  1  /* DSP_IRQ2_EINT2 */
#define CS47L24_DSP_IRQ2_EINT2_WIDTH                  1  /* DSP_IRQ2_EINT2 */
#define CS47L24_DSP_IRQ1_EINT2                   0x0001  /* DSP_IRQ1_EINT2 */
#define CS47L24_DSP_IRQ1_EINT2_MASK              0x0001  /* DSP_IRQ1_EINT2 */
#define CS47L24_DSP_IRQ1_EINT2_SHIFT                  0  /* DSP_IRQ1_EINT2 */
#define CS47L24_DSP_IRQ1_EINT2_WIDTH                  1  /* DSP_IRQ1_EINT2 */

/*
 * R3346 (0xD12) - IRQ2 Status 3
 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT2          0x8000  /* SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT2_MASK     0x8000  /* SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT2_SHIFT        15  /* SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_SPK_OVERHEAT_WARN_EINT2_WIDTH         1  /* SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_SPK_OVERHEAT_EINT2               0x4000  /* SPK_OVERHEAT_EINT2 */
#define CS47L24_SPK_OVERHEAT_EINT2_MASK          0x4000  /* SPK_OVERHEAT_EINT2 */
#define CS47L24_SPK_OVERHEAT_EINT2_SHIFT             14  /* SPK_OVERHEAT_EINT2 */
#define CS47L24_SPK_OVERHEAT_EINT2_WIDTH              1  /* SPK_OVERHEAT_EINT2 */
#define CS47L24_HPDET_EINT2                      0x2000  /* HPDET_EINT2 */
#define CS47L24_HPDET_EINT2_MASK                 0x2000  /* HPDET_EINT2 */
#define CS47L24_HPDET_EINT2_SHIFT                    13  /* HPDET_EINT2 */
#define CS47L24_HPDET_EINT2_WIDTH                     1  /* HPDET_EINT2 */
#define CS47L24_MICDET_EINT2                     0x1000  /* MICDET_EINT2 */
#define CS47L24_MICDET_EINT2_MASK                0x1000  /* MICDET_EINT2 */
#define CS47L24_MICDET_EINT2_SHIFT                   12  /* MICDET_EINT2 */
#define CS47L24_MICDET_EINT2_WIDTH                    1  /* MICDET_EINT2 */
#define CS47L24_WSEQ_DONE_EINT2                  0x0800  /* WSEQ_DONE_EINT2 */
#define CS47L24_WSEQ_DONE_EINT2_MASK             0x0800  /* WSEQ_DONE_EINT2 */
#define CS47L24_WSEQ_DONE_EINT2_SHIFT                11  /* WSEQ_DONE_EINT2 */
#define CS47L24_WSEQ_DONE_EINT2_WIDTH                 1  /* WSEQ_DONE_EINT2 */
#define CS47L24_DRC2_SIG_DET_EINT2               0x0400  /* DRC2_SIG_DET_EINT2 */
#define CS47L24_DRC2_SIG_DET_EINT2_MASK          0x0400  /* DRC2_SIG_DET_EINT2 */
#define CS47L24_DRC2_SIG_DET_EINT2_SHIFT             10  /* DRC2_SIG_DET_EINT2 */
#define CS47L24_DRC2_SIG_DET_EINT2_WIDTH              1  /* DRC2_SIG_DET_EINT2 */
#define CS47L24_DRC1_SIG_DET_EINT2               0x0200  /* DRC1_SIG_DET_EINT2 */
#define CS47L24_DRC1_SIG_DET_EINT2_MASK          0x0200  /* DRC1_SIG_DET_EINT2 */
#define CS47L24_DRC1_SIG_DET_EINT2_SHIFT              9  /* DRC1_SIG_DET_EINT2 */
#define CS47L24_DRC1_SIG_DET_EINT2_WIDTH              1  /* DRC1_SIG_DET_EINT2 */
#define CS47L24_ASRC2_LOCK_EINT2                 0x0100  /* ASRC2_LOCK_EINT2 */
#define CS47L24_ASRC2_LOCK_EINT2_MASK            0x0100  /* ASRC2_LOCK_EINT2 */
#define CS47L24_ASRC2_LOCK_EINT2_SHIFT                8  /* ASRC2_LOCK_EINT2 */
#define CS47L24_ASRC2_LOCK_EINT2_WIDTH                1  /* ASRC2_LOCK_EINT2 */
#define CS47L24_ASRC1_LOCK_EINT2                 0x0080  /* ASRC1_LOCK_EINT2 */
#define CS47L24_ASRC1_LOCK_EINT2_MASK            0x0080  /* ASRC1_LOCK_EINT2 */
#define CS47L24_ASRC1_LOCK_EINT2_SHIFT                7  /* ASRC1_LOCK_EINT2 */
#define CS47L24_ASRC1_LOCK_EINT2_WIDTH                1  /* ASRC1_LOCK_EINT2 */
#define CS47L24_UNDERCLOCKED_EINT2               0x0040  /* UNDERCLOCKED_EINT2 */
#define CS47L24_UNDERCLOCKED_EINT2_MASK          0x0040  /* UNDERCLOCKED_EINT2 */
#define CS47L24_UNDERCLOCKED_EINT2_SHIFT              6  /* UNDERCLOCKED_EINT2 */
#define CS47L24_UNDERCLOCKED_EINT2_WIDTH              1  /* UNDERCLOCKED_EINT2 */
#define CS47L24_OVERCLOCKED_EINT2                0x0020  /* OVERCLOCKED_EINT2 */
#define CS47L24_OVERCLOCKED_EINT2_MASK           0x0020  /* OVERCLOCKED_EINT2 */
#define CS47L24_OVERCLOCKED_EINT2_SHIFT               5  /* OVERCLOCKED_EINT2 */
#define CS47L24_OVERCLOCKED_EINT2_WIDTH               1  /* OVERCLOCKED_EINT2 */
#define CS47L24_FLL2_LOCK_EINT2                  0x0008  /* FLL2_LOCK_EINT2 */
#define CS47L24_FLL2_LOCK_EINT2_MASK             0x0008  /* FLL2_LOCK_EINT2 */
#define CS47L24_FLL2_LOCK_EINT2_SHIFT                 3  /* FLL2_LOCK_EINT2 */
#define CS47L24_FLL2_LOCK_EINT2_WIDTH                 1  /* FLL2_LOCK_EINT2 */
#define CS47L24_FLL1_LOCK_EINT2                  0x0004  /* FLL1_LOCK_EINT2 */
#define CS47L24_FLL1_LOCK_EINT2_MASK             0x0004  /* FLL1_LOCK_EINT2 */
#define CS47L24_FLL1_LOCK_EINT2_SHIFT                 2  /* FLL1_LOCK_EINT2 */
#define CS47L24_FLL1_LOCK_EINT2_WIDTH                 1  /* FLL1_LOCK_EINT2 */
#define CS47L24_CLKGEN_ERR_EINT2                 0x0002  /* CLKGEN_ERR_EINT2 */
#define CS47L24_CLKGEN_ERR_EINT2_MASK            0x0002  /* CLKGEN_ERR_EINT2 */
#define CS47L24_CLKGEN_ERR_EINT2_SHIFT                1  /* CLKGEN_ERR_EINT2 */
#define CS47L24_CLKGEN_ERR_EINT2_WIDTH                1  /* CLKGEN_ERR_EINT2 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT2           0x0001  /* CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT2_MASK      0x0001  /* CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT2_SHIFT          0  /* CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_CLKGEN_ERR_ASYNC_EINT2_WIDTH          1  /* CLKGEN_ERR_ASYNC_EINT2 */

/*
 * R3347 (0xD13) - IRQ2 Status 4
 */
#define CS47L24_ASRC_CFG_ERR_EINT2               0x8000  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_ASRC_CFG_ERR_EINT2_MASK          0x8000  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_ASRC_CFG_ERR_EINT2_SHIFT             15  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_ASRC_CFG_ERR_EINT2_WIDTH              1  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_AIF3_ERR_EINT2                   0x4000  /* AIF3_ERR_EINT2 */
#define CS47L24_AIF3_ERR_EINT2_MASK              0x4000  /* AIF3_ERR_EINT2 */
#define CS47L24_AIF3_ERR_EINT2_SHIFT                 14  /* AIF3_ERR_EINT2 */
#define CS47L24_AIF3_ERR_EINT2_WIDTH                  1  /* AIF3_ERR_EINT2 */
#define CS47L24_AIF2_ERR_EINT2                   0x2000  /* AIF2_ERR_EINT2 */
#define CS47L24_AIF2_ERR_EINT2_MASK              0x2000  /* AIF2_ERR_EINT2 */
#define CS47L24_AIF2_ERR_EINT2_SHIFT                 13  /* AIF2_ERR_EINT2 */
#define CS47L24_AIF2_ERR_EINT2_WIDTH                  1  /* AIF2_ERR_EINT2 */
#define CS47L24_AIF1_ERR_EINT2                   0x1000  /* AIF1_ERR_EINT2 */
#define CS47L24_AIF1_ERR_EINT2_MASK              0x1000  /* AIF1_ERR_EINT2 */
#define CS47L24_AIF1_ERR_EINT2_SHIFT                 12  /* AIF1_ERR_EINT2 */
#define CS47L24_AIF1_ERR_EINT2_WIDTH                  1  /* AIF1_ERR_EINT2 */
#define CS47L24_CTRLIF_ERR_EINT2                 0x0800  /* CTRLIF_ERR_EINT2 */
#define CS47L24_CTRLIF_ERR_EINT2_MASK            0x0800  /* CTRLIF_ERR_EINT2 */
#define CS47L24_CTRLIF_ERR_EINT2_SHIFT               11  /* CTRLIF_ERR_EINT2 */
#define CS47L24_CTRLIF_ERR_EINT2_WIDTH                1  /* CTRLIF_ERR_EINT2 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT2       0x0400  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT2_MASK  0x0400  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT2_SHIFT     10  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_MIXER_DROPPED_SAMPLE_EINT2_WIDTH      1  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT2          0x0200  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT2_MASK     0x0200  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT2_SHIFT         9  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_ASYNC_CLK_ENA_LOW_EINT2_WIDTH         1  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_SYSCLK_ENA_LOW_EINT2             0x0100  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_SYSCLK_ENA_LOW_EINT2_MASK        0x0100  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_SYSCLK_ENA_LOW_EINT2_SHIFT            8  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_SYSCLK_ENA_LOW_EINT2_WIDTH            1  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_ISRC1_CFG_ERR_EINT2              0x0080  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_ISRC1_CFG_ERR_EINT2_MASK         0x0080  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_ISRC1_CFG_ERR_EINT2_SHIFT             7  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_ISRC1_CFG_ERR_EINT2_WIDTH             1  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_ISRC2_CFG_ERR_EINT2              0x0040  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_ISRC2_CFG_ERR_EINT2_MASK         0x0040  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_ISRC2_CFG_ERR_EINT2_SHIFT             6  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_ISRC2_CFG_ERR_EINT2_WIDTH             1  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_HP1R_DONE_EINT2                  0x0002  /* HP1R_DONE_EINT2 */
#define CS47L24_HP1R_DONE_EINT2_MASK             0x0002  /* HP1R_DONE_EINT2 */
#define CS47L24_HP1R_DONE_EINT2_SHIFT                 1  /* HP1R_DONE_EINT2 */
#define CS47L24_HP1R_DONE_EINT2_WIDTH                 1  /* HP1R_DONE_EINT2 */
#define CS47L24_HP1L_DONE_EINT2                  0x0001  /* HP1L_DONE_EINT2 */
#define CS47L24_HP1L_DONE_EINT2_MASK             0x0001  /* HP1L_DONE_EINT2 */
#define CS47L24_HP1L_DONE_EINT2_SHIFT                 0  /* HP1L_DONE_EINT2 */
#define CS47L24_HP1L_DONE_EINT2_WIDTH                 1  /* HP1L_DONE_EINT2 */

/*
 * R3347 (0xD13) - IRQ2 Status 4 (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_AIF3_ERR_EINT2                     0x8000  /* AIF3_ERR_EINT2 */
#define CS47L24_V2_AIF3_ERR_EINT2_MASK                0x8000  /* AIF3_ERR_EINT2 */
#define CS47L24_V2_AIF3_ERR_EINT2_SHIFT                   15  /* AIF3_ERR_EINT2 */
#define CS47L24_V2_AIF3_ERR_EINT2_WIDTH                    1  /* AIF3_ERR_EINT2 */
#define CS47L24_V2_AIF2_ERR_EINT2                     0x4000  /* AIF2_ERR_EINT2 */
#define CS47L24_V2_AIF2_ERR_EINT2_MASK                0x4000  /* AIF2_ERR_EINT2 */
#define CS47L24_V2_AIF2_ERR_EINT2_SHIFT                   14  /* AIF2_ERR_EINT2 */
#define CS47L24_V2_AIF2_ERR_EINT2_WIDTH                    1  /* AIF2_ERR_EINT2 */
#define CS47L24_V2_AIF1_ERR_EINT2                     0x2000  /* AIF1_ERR_EINT2 */
#define CS47L24_V2_AIF1_ERR_EINT2_MASK                0x2000  /* AIF1_ERR_EINT2 */
#define CS47L24_V2_AIF1_ERR_EINT2_SHIFT                   13  /* AIF1_ERR_EINT2 */
#define CS47L24_V2_AIF1_ERR_EINT2_WIDTH                    1  /* AIF1_ERR_EINT2 */
#define CS47L24_V2_CTRLIF_ERR_EINT2                   0x1000  /* CTRLIF_ERR_EINT2 */
#define CS47L24_V2_CTRLIF_ERR_EINT2_MASK              0x1000  /* CTRLIF_ERR_EINT2 */
#define CS47L24_V2_CTRLIF_ERR_EINT2_SHIFT                 12  /* CTRLIF_ERR_EINT2 */
#define CS47L24_V2_CTRLIF_ERR_EINT2_WIDTH                  1  /* CTRLIF_ERR_EINT2 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT2         0x0800  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT2_MASK    0x0800  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT2_SHIFT       11  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_MIXER_DROPPED_SAMPLE_EINT2_WIDTH        1  /* MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT2            0x0400  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT2_MASK       0x0400  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT2_SHIFT          10  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_ASYNC_CLK_ENA_LOW_EINT2_WIDTH           1  /* ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT2               0x0200  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT2_MASK          0x0200  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT2_SHIFT              9  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_SYSCLK_ENA_LOW_EINT2_WIDTH              1  /* SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT2                0x0100  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT2_MASK           0x0100  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT2_SHIFT               8  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC1_CFG_ERR_EINT2_WIDTH               1  /* ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT2                0x0080  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT2_MASK           0x0080  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT2_SHIFT               7  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC2_CFG_ERR_EINT2_WIDTH               1  /* ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT2                0x0040  /* ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT2_MASK           0x0040  /* ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT2_SHIFT               6  /* ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_ISRC3_CFG_ERR_EINT2_WIDTH               1  /* ISRC3_CFG_ERR_EINT2 */

/*
 * R3348 (0xD14) - IRQ2 Status 5
 */
#define CS47L24_BOOT_DONE_EINT2                  0x0100  /* BOOT_DONE_EINT2 */
#define CS47L24_BOOT_DONE_EINT2_MASK             0x0100  /* BOOT_DONE_EINT2 */
#define CS47L24_BOOT_DONE_EINT2_SHIFT                 8  /* BOOT_DONE_EINT2 */
#define CS47L24_BOOT_DONE_EINT2_WIDTH                 1  /* BOOT_DONE_EINT2 */
#define CS47L24_DCS_DAC_DONE_EINT2               0x0080  /* DCS_DAC_DONE_EINT2 */
#define CS47L24_DCS_DAC_DONE_EINT2_MASK          0x0080  /* DCS_DAC_DONE_EINT2 */
#define CS47L24_DCS_DAC_DONE_EINT2_SHIFT              7  /* DCS_DAC_DONE_EINT2 */
#define CS47L24_DCS_DAC_DONE_EINT2_WIDTH              1  /* DCS_DAC_DONE_EINT2 */
#define CS47L24_DCS_HP_DONE_EINT2                0x0040  /* DCS_HP_DONE_EINT2 */
#define CS47L24_DCS_HP_DONE_EINT2_MASK           0x0040  /* DCS_HP_DONE_EINT2 */
#define CS47L24_DCS_HP_DONE_EINT2_SHIFT               6  /* DCS_HP_DONE_EINT2 */
#define CS47L24_DCS_HP_DONE_EINT2_WIDTH               1  /* DCS_HP_DONE_EINT2 */
#define CS47L24_FLL2_CLOCK_OK_EINT2              0x0002  /* FLL2_CLOCK_OK_EINT2 */
#define CS47L24_FLL2_CLOCK_OK_EINT2_MASK         0x0002  /* FLL2_CLOCK_OK_EINT2 */
#define CS47L24_FLL2_CLOCK_OK_EINT2_SHIFT             1  /* FLL2_CLOCK_OK_EINT2 */
#define CS47L24_FLL2_CLOCK_OK_EINT2_WIDTH             1  /* FLL2_CLOCK_OK_EINT2 */
#define CS47L24_FLL1_CLOCK_OK_EINT2              0x0001  /* FLL1_CLOCK_OK_EINT2 */
#define CS47L24_FLL1_CLOCK_OK_EINT2_MASK         0x0001  /* FLL1_CLOCK_OK_EINT2 */
#define CS47L24_FLL1_CLOCK_OK_EINT2_SHIFT             0  /* FLL1_CLOCK_OK_EINT2 */
#define CS47L24_FLL1_CLOCK_OK_EINT2_WIDTH             1  /* FLL1_CLOCK_OK_EINT2 */

/*
 * R3348 (0xD14) - IRQ2 Status 5 (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT2            0x0008  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT2_MASK       0x0008  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT2_SHIFT           3  /* ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_ASRC_CFG_ERR_EINT2_WIDTH           1  /* ASRC_CFG_ERR_EINT2 */

/*
 * R3349 (0xD15) - IRQ2 Status 6
 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT2         0x8000  /* DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT2_MASK    0x8000  /* DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT2_SHIFT       15  /* DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_DSP_SHARED_WR_COLL_EINT2_WIDTH        1  /* DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_SPK_SHUTDOWN_EINT2               0x4000  /* SPK_SHUTDOWN_EINT2 */
#define CS47L24_SPK_SHUTDOWN_EINT2_MASK          0x4000  /* SPK_SHUTDOWN_EINT2 */
#define CS47L24_SPK_SHUTDOWN_EINT2_SHIFT             14  /* SPK_SHUTDOWN_EINT2 */
#define CS47L24_SPK_SHUTDOWN_EINT2_WIDTH              1  /* SPK_SHUTDOWN_EINT2 */
#define CS47L24_SPK1R_SHORT_EINT2                0x2000  /* SPK1R_SHORT_EINT2 */
#define CS47L24_SPK1R_SHORT_EINT2_MASK           0x2000  /* SPK1R_SHORT_EINT2 */
#define CS47L24_SPK1R_SHORT_EINT2_SHIFT              13  /* SPK1R_SHORT_EINT2 */
#define CS47L24_SPK1R_SHORT_EINT2_WIDTH               1  /* SPK1R_SHORT_EINT2 */
#define CS47L24_SPK1L_SHORT_EINT2                0x1000  /* SPK1L_SHORT_EINT2 */
#define CS47L24_SPK1L_SHORT_EINT2_MASK           0x1000  /* SPK1L_SHORT_EINT2 */
#define CS47L24_SPK1L_SHORT_EINT2_SHIFT              12  /* SPK1L_SHORT_EINT2 */
#define CS47L24_SPK1L_SHORT_EINT2_WIDTH               1  /* SPK1L_SHORT_EINT2 */
#define CS47L24_HP1R_SC_NEG_EINT2                0x0008  /* HP1R_SC_NEG_EINT2 */
#define CS47L24_HP1R_SC_NEG_EINT2_MASK           0x0008  /* HP1R_SC_NEG_EINT2 */
#define CS47L24_HP1R_SC_NEG_EINT2_SHIFT               3  /* HP1R_SC_NEG_EINT2 */
#define CS47L24_HP1R_SC_NEG_EINT2_WIDTH               1  /* HP1R_SC_NEG_EINT2 */
#define CS47L24_HP1R_SC_POS_EINT2                0x0004  /* HP1R_SC_POS_EINT2 */
#define CS47L24_HP1R_SC_POS_EINT2_MASK           0x0004  /* HP1R_SC_POS_EINT2 */
#define CS47L24_HP1R_SC_POS_EINT2_SHIFT               2  /* HP1R_SC_POS_EINT2 */
#define CS47L24_HP1R_SC_POS_EINT2_WIDTH               1  /* HP1R_SC_POS_EINT2 */
#define CS47L24_HP1L_SC_NEG_EINT2                0x0002  /* HP1L_SC_NEG_EINT2 */
#define CS47L24_HP1L_SC_NEG_EINT2_MASK           0x0002  /* HP1L_SC_NEG_EINT2 */
#define CS47L24_HP1L_SC_NEG_EINT2_SHIFT               1  /* HP1L_SC_NEG_EINT2 */
#define CS47L24_HP1L_SC_NEG_EINT2_WIDTH               1  /* HP1L_SC_NEG_EINT2 */
#define CS47L24_HP1L_SC_POS_EINT2                0x0001  /* HP1L_SC_POS_EINT2 */
#define CS47L24_HP1L_SC_POS_EINT2_MASK           0x0001  /* HP1L_SC_POS_EINT2 */
#define CS47L24_HP1L_SC_POS_EINT2_SHIFT               0  /* HP1L_SC_POS_EINT2 */
#define CS47L24_HP1L_SC_POS_EINT2_WIDTH               1  /* HP1L_SC_POS_EINT2 */

/*
 * R3352 (0xD18) - IRQ2 Status 1 Mask
 */
#define CS47L24_IM_GP2_EINT2                     0x0002  /* IM_GP2_EINT2 */
#define CS47L24_IM_GP2_EINT2_MASK                0x0002  /* IM_GP2_EINT2 */
#define CS47L24_IM_GP2_EINT2_SHIFT                    1  /* IM_GP2_EINT2 */
#define CS47L24_IM_GP2_EINT2_WIDTH                    1  /* IM_GP2_EINT2 */
#define CS47L24_IM_GP1_EINT2                     0x0001  /* IM_GP1_EINT2 */
#define CS47L24_IM_GP1_EINT2_MASK                0x0001  /* IM_GP1_EINT2 */
#define CS47L24_IM_GP1_EINT2_SHIFT                    0  /* IM_GP1_EINT2 */
#define CS47L24_IM_GP1_EINT2_WIDTH                    1  /* IM_GP1_EINT2 */

/*
 * R3353 (0xD19) - IRQ2 Status 2 Mask
 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT2            0x0100  /* IM_DSP1_RAM_RDY_EINT2 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT2_MASK       0x0100  /* IM_DSP1_RAM_RDY_EINT2 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT2_SHIFT           8  /* IM_DSP1_RAM_RDY_EINT2 */
#define CS47L24_IM_DSP1_RAM_RDY_EINT2_WIDTH           1  /* IM_DSP1_RAM_RDY_EINT2 */
#define CS47L24_IM_DSP_IRQ2_EINT2                0x0002  /* IM_DSP_IRQ2_EINT2 */
#define CS47L24_IM_DSP_IRQ2_EINT2_MASK           0x0002  /* IM_DSP_IRQ2_EINT2 */
#define CS47L24_IM_DSP_IRQ2_EINT2_SHIFT               1  /* IM_DSP_IRQ2_EINT2 */
#define CS47L24_IM_DSP_IRQ2_EINT2_WIDTH               1  /* IM_DSP_IRQ2_EINT2 */
#define CS47L24_IM_DSP_IRQ1_EINT2                0x0001  /* IM_DSP_IRQ1_EINT2 */
#define CS47L24_IM_DSP_IRQ1_EINT2_MASK           0x0001  /* IM_DSP_IRQ1_EINT2 */
#define CS47L24_IM_DSP_IRQ1_EINT2_SHIFT               0  /* IM_DSP_IRQ1_EINT2 */
#define CS47L24_IM_DSP_IRQ1_EINT2_WIDTH               1  /* IM_DSP_IRQ1_EINT2 */

/*
 * R3354 (0xD1A) - IRQ2 Status 3 Mask
 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT2       0x8000  /* IM_SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT2_MASK  0x8000  /* IM_SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT2_SHIFT     15  /* IM_SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_WARN_EINT2_WIDTH      1  /* IM_SPK_OVERHEAT_WARN_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_EINT2            0x4000  /* IM_SPK_OVERHEAT_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_EINT2_MASK       0x4000  /* IM_SPK_OVERHEAT_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_EINT2_SHIFT          14  /* IM_SPK_OVERHEAT_EINT2 */
#define CS47L24_IM_SPK_OVERHEAT_EINT2_WIDTH           1  /* IM_SPK_OVERHEAT_EINT2 */
#define CS47L24_IM_HPDET_EINT2                   0x2000  /* IM_HPDET_EINT2 */
#define CS47L24_IM_HPDET_EINT2_MASK              0x2000  /* IM_HPDET_EINT2 */
#define CS47L24_IM_HPDET_EINT2_SHIFT                 13  /* IM_HPDET_EINT2 */
#define CS47L24_IM_HPDET_EINT2_WIDTH                  1  /* IM_HPDET_EINT2 */
#define CS47L24_IM_MICDET_EINT2                  0x1000  /* IM_MICDET_EINT2 */
#define CS47L24_IM_MICDET_EINT2_MASK             0x1000  /* IM_MICDET_EINT2 */
#define CS47L24_IM_MICDET_EINT2_SHIFT                12  /* IM_MICDET_EINT2 */
#define CS47L24_IM_MICDET_EINT2_WIDTH                 1  /* IM_MICDET_EINT2 */
#define CS47L24_IM_WSEQ_DONE_EINT2               0x0800  /* IM_WSEQ_DONE_EINT2 */
#define CS47L24_IM_WSEQ_DONE_EINT2_MASK          0x0800  /* IM_WSEQ_DONE_EINT2 */
#define CS47L24_IM_WSEQ_DONE_EINT2_SHIFT             11  /* IM_WSEQ_DONE_EINT2 */
#define CS47L24_IM_WSEQ_DONE_EINT2_WIDTH              1  /* IM_WSEQ_DONE_EINT2 */
#define CS47L24_IM_DRC2_SIG_DET_EINT2            0x0400  /* IM_DRC2_SIG_DET_EINT2 */
#define CS47L24_IM_DRC2_SIG_DET_EINT2_MASK       0x0400  /* IM_DRC2_SIG_DET_EINT2 */
#define CS47L24_IM_DRC2_SIG_DET_EINT2_SHIFT          10  /* IM_DRC2_SIG_DET_EINT2 */
#define CS47L24_IM_DRC2_SIG_DET_EINT2_WIDTH           1  /* IM_DRC2_SIG_DET_EINT2 */
#define CS47L24_IM_DRC1_SIG_DET_EINT2            0x0200  /* IM_DRC1_SIG_DET_EINT2 */
#define CS47L24_IM_DRC1_SIG_DET_EINT2_MASK       0x0200  /* IM_DRC1_SIG_DET_EINT2 */
#define CS47L24_IM_DRC1_SIG_DET_EINT2_SHIFT           9  /* IM_DRC1_SIG_DET_EINT2 */
#define CS47L24_IM_DRC1_SIG_DET_EINT2_WIDTH           1  /* IM_DRC1_SIG_DET_EINT2 */
#define CS47L24_IM_ASRC2_LOCK_EINT2              0x0100  /* IM_ASRC2_LOCK_EINT2 */
#define CS47L24_IM_ASRC2_LOCK_EINT2_MASK         0x0100  /* IM_ASRC2_LOCK_EINT2 */
#define CS47L24_IM_ASRC2_LOCK_EINT2_SHIFT             8  /* IM_ASRC2_LOCK_EINT2 */
#define CS47L24_IM_ASRC2_LOCK_EINT2_WIDTH             1  /* IM_ASRC2_LOCK_EINT2 */
#define CS47L24_IM_ASRC1_LOCK_EINT2              0x0080  /* IM_ASRC1_LOCK_EINT2 */
#define CS47L24_IM_ASRC1_LOCK_EINT2_MASK         0x0080  /* IM_ASRC1_LOCK_EINT2 */
#define CS47L24_IM_ASRC1_LOCK_EINT2_SHIFT             7  /* IM_ASRC1_LOCK_EINT2 */
#define CS47L24_IM_ASRC1_LOCK_EINT2_WIDTH             1  /* IM_ASRC1_LOCK_EINT2 */
#define CS47L24_IM_UNDERCLOCKED_EINT2            0x0040  /* IM_UNDERCLOCKED_EINT2 */
#define CS47L24_IM_UNDERCLOCKED_EINT2_MASK       0x0040  /* IM_UNDERCLOCKED_EINT2 */
#define CS47L24_IM_UNDERCLOCKED_EINT2_SHIFT           6  /* IM_UNDERCLOCKED_EINT2 */
#define CS47L24_IM_UNDERCLOCKED_EINT2_WIDTH           1  /* IM_UNDERCLOCKED_EINT2 */
#define CS47L24_IM_OVERCLOCKED_EINT2             0x0020  /* IM_OVERCLOCKED_EINT2 */
#define CS47L24_IM_OVERCLOCKED_EINT2_MASK        0x0020  /* IM_OVERCLOCKED_EINT2 */
#define CS47L24_IM_OVERCLOCKED_EINT2_SHIFT            5  /* IM_OVERCLOCKED_EINT2 */
#define CS47L24_IM_OVERCLOCKED_EINT2_WIDTH            1  /* IM_OVERCLOCKED_EINT2 */
#define CS47L24_IM_FLL2_LOCK_EINT2               0x0008  /* IM_FLL2_LOCK_EINT2 */
#define CS47L24_IM_FLL2_LOCK_EINT2_MASK          0x0008  /* IM_FLL2_LOCK_EINT2 */
#define CS47L24_IM_FLL2_LOCK_EINT2_SHIFT              3  /* IM_FLL2_LOCK_EINT2 */
#define CS47L24_IM_FLL2_LOCK_EINT2_WIDTH              1  /* IM_FLL2_LOCK_EINT2 */
#define CS47L24_IM_FLL1_LOCK_EINT2               0x0004  /* IM_FLL1_LOCK_EINT2 */
#define CS47L24_IM_FLL1_LOCK_EINT2_MASK          0x0004  /* IM_FLL1_LOCK_EINT2 */
#define CS47L24_IM_FLL1_LOCK_EINT2_SHIFT              2  /* IM_FLL1_LOCK_EINT2 */
#define CS47L24_IM_FLL1_LOCK_EINT2_WIDTH              1  /* IM_FLL1_LOCK_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_EINT2              0x0002  /* IM_CLKGEN_ERR_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_EINT2_MASK         0x0002  /* IM_CLKGEN_ERR_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_EINT2_SHIFT             1  /* IM_CLKGEN_ERR_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_EINT2_WIDTH             1  /* IM_CLKGEN_ERR_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT2        0x0001  /* IM_CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT2_MASK   0x0001  /* IM_CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT2_SHIFT       0  /* IM_CLKGEN_ERR_ASYNC_EINT2 */
#define CS47L24_IM_CLKGEN_ERR_ASYNC_EINT2_WIDTH       1  /* IM_CLKGEN_ERR_ASYNC_EINT2 */

/*
 * R3355 (0xD1B) - IRQ2 Status 4 Mask
 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT2                 0x8000  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT2_MASK            0x8000  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT2_SHIFT               15  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_IM_ASRC_CFG_ERR_EINT2_WIDTH                1  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_IM_AIF3_ERR_EINT2                     0x4000  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_IM_AIF3_ERR_EINT2_MASK                0x4000  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_IM_AIF3_ERR_EINT2_SHIFT                   14  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_IM_AIF3_ERR_EINT2_WIDTH                    1  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_IM_AIF2_ERR_EINT2                     0x2000  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_IM_AIF2_ERR_EINT2_MASK                0x2000  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_IM_AIF2_ERR_EINT2_SHIFT                   13  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_IM_AIF2_ERR_EINT2_WIDTH                    1  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_IM_AIF1_ERR_EINT2                     0x1000  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_IM_AIF1_ERR_EINT2_MASK                0x1000  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_IM_AIF1_ERR_EINT2_SHIFT                   12  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_IM_AIF1_ERR_EINT2_WIDTH                    1  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_IM_CTRLIF_ERR_EINT2                   0x0800  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_IM_CTRLIF_ERR_EINT2_MASK              0x0800  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_IM_CTRLIF_ERR_EINT2_SHIFT                 11  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_IM_CTRLIF_ERR_EINT2_WIDTH                  1  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT2         0x0400  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT2_MASK    0x0400  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT2_SHIFT       10  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_IM_MIXER_DROPPED_SAMPLE_EINT2_WIDTH        1  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT2            0x0200  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT2_MASK       0x0200  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT2_SHIFT           9  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_IM_ASYNC_CLK_ENA_LOW_EINT2_WIDTH           1  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT2               0x0100  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT2_MASK          0x0100  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT2_SHIFT              8  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_IM_SYSCLK_ENA_LOW_EINT2_WIDTH              1  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT2                0x0080  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT2_MASK           0x0080  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT2_SHIFT               7  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC1_CFG_ERR_EINT2_WIDTH               1  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT2                0x0040  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT2_MASK           0x0040  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT2_SHIFT               6  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_IM_ISRC2_CFG_ERR_EINT2_WIDTH               1  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_IM_HP1R_DONE_EINT2                    0x0002  /* IM_HP1R_DONE_EINT2 */
#define CS47L24_IM_HP1R_DONE_EINT2_MASK               0x0002  /* IM_HP1R_DONE_EINT2 */
#define CS47L24_IM_HP1R_DONE_EINT2_SHIFT                   1  /* IM_HP1R_DONE_EINT2 */
#define CS47L24_IM_HP1R_DONE_EINT2_WIDTH                   1  /* IM_HP1R_DONE_EINT2 */
#define CS47L24_IM_HP1L_DONE_EINT2                    0x0001  /* IM_HP1L_DONE_EINT2 */
#define CS47L24_IM_HP1L_DONE_EINT2_MASK               0x0001  /* IM_HP1L_DONE_EINT2 */
#define CS47L24_IM_HP1L_DONE_EINT2_SHIFT                   0  /* IM_HP1L_DONE_EINT2 */
#define CS47L24_IM_HP1L_DONE_EINT2_WIDTH                   1  /* IM_HP1L_DONE_EINT2 */

/*
 * R3355 (0xD1B) - IRQ2 Status 4 Mask (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_IM_AIF3_ERR_EINT2                  0x8000  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_V2_IM_AIF3_ERR_EINT2_MASK             0x8000  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_V2_IM_AIF3_ERR_EINT2_SHIFT                15  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_V2_IM_AIF3_ERR_EINT2_WIDTH                 1  /* IM_AIF3_ERR_EINT2 */
#define CS47L24_V2_IM_AIF2_ERR_EINT2                  0x4000  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_V2_IM_AIF2_ERR_EINT2_MASK             0x4000  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_V2_IM_AIF2_ERR_EINT2_SHIFT                14  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_V2_IM_AIF2_ERR_EINT2_WIDTH                 1  /* IM_AIF2_ERR_EINT2 */
#define CS47L24_V2_IM_AIF1_ERR_EINT2                  0x2000  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_V2_IM_AIF1_ERR_EINT2_MASK             0x2000  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_V2_IM_AIF1_ERR_EINT2_SHIFT                13  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_V2_IM_AIF1_ERR_EINT2_WIDTH                 1  /* IM_AIF1_ERR_EINT2 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT2                0x1000  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT2_MASK           0x1000  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT2_SHIFT              12  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_V2_IM_CTRLIF_ERR_EINT2_WIDTH               1  /* IM_CTRLIF_ERR_EINT2 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT2      0x0800  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT2_MASK 0x0800  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT2_SHIFT    11  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_IM_MIXER_DROPPED_SAMPLE_EINT2_WIDTH     1  /* IM_MIXER_DROPPED_SAMPLE_EINT2 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT2         0x0400  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT2_MASK    0x0400  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT2_SHIFT       10  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_ASYNC_CLK_ENA_LOW_EINT2_WIDTH        1  /* IM_ASYNC_CLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT2            0x0200  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT2_MASK       0x0200  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT2_SHIFT           9  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_SYSCLK_ENA_LOW_EINT2_WIDTH           1  /* IM_SYSCLK_ENA_LOW_EINT2 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT2             0x0100  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT2_MASK        0x0100  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT2_SHIFT            8  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC1_CFG_ERR_EINT2_WIDTH            1  /* IM_ISRC1_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT2             0x0080  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT2_MASK        0x0080  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT2_SHIFT            7  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC2_CFG_ERR_EINT2_WIDTH            1  /* IM_ISRC2_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT2             0x0040  /* IM_ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT2_MASK        0x0040  /* IM_ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT2_SHIFT            6  /* IM_ISRC3_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ISRC3_CFG_ERR_EINT2_WIDTH            1  /* IM_ISRC3_CFG_ERR_EINT2 */

/*
 * R3356 (0xD1C) - IRQ2 Status 5 Mask
 */

#define CS47L24_IM_BOOT_DONE_EINT2               0x0100  /* IM_BOOT_DONE_EINT2 */
#define CS47L24_IM_BOOT_DONE_EINT2_MASK          0x0100  /* IM_BOOT_DONE_EINT2 */
#define CS47L24_IM_BOOT_DONE_EINT2_SHIFT              8  /* IM_BOOT_DONE_EINT2 */
#define CS47L24_IM_BOOT_DONE_EINT2_WIDTH              1  /* IM_BOOT_DONE_EINT2 */
#define CS47L24_IM_DCS_DAC_DONE_EINT2            0x0080  /* IM_DCS_DAC_DONE_EINT2 */
#define CS47L24_IM_DCS_DAC_DONE_EINT2_MASK       0x0080  /* IM_DCS_DAC_DONE_EINT2 */
#define CS47L24_IM_DCS_DAC_DONE_EINT2_SHIFT           7  /* IM_DCS_DAC_DONE_EINT2 */
#define CS47L24_IM_DCS_DAC_DONE_EINT2_WIDTH           1  /* IM_DCS_DAC_DONE_EINT2 */
#define CS47L24_IM_DCS_HP_DONE_EINT2             0x0040  /* IM_DCS_HP_DONE_EINT2 */
#define CS47L24_IM_DCS_HP_DONE_EINT2_MASK        0x0040  /* IM_DCS_HP_DONE_EINT2 */
#define CS47L24_IM_DCS_HP_DONE_EINT2_SHIFT            6  /* IM_DCS_HP_DONE_EINT2 */
#define CS47L24_IM_DCS_HP_DONE_EINT2_WIDTH            1  /* IM_DCS_HP_DONE_EINT2 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT2           0x0002  /* IM_FLL2_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT2_MASK      0x0002  /* IM_FLL2_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT2_SHIFT          1  /* IM_FLL2_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL2_CLOCK_OK_EINT2_WIDTH          1  /* IM_FLL2_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT2           0x0001  /* IM_FLL1_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT2_MASK      0x0001  /* IM_FLL1_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT2_SHIFT          0  /* IM_FLL1_CLOCK_OK_EINT2 */
#define CS47L24_IM_FLL1_CLOCK_OK_EINT2_WIDTH          1  /* IM_FLL1_CLOCK_OK_EINT2 */

/*
 * R3340 (0xD0C) - Interrupt Status 5 Mask (Alternate layout)
 *
 * Alternate layout used on later devices, note only fields that have moved
 * are specified
 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT2         0x0008  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT2_MASK    0x0008  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT2_SHIFT        3  /* IM_ASRC_CFG_ERR_EINT2 */
#define CS47L24_V2_IM_ASRC_CFG_ERR_EINT2_WIDTH        1  /* IM_ASRC_CFG_ERR_EINT2 */

/*
 * R3357 (0xD1D) - IRQ2 Status 6 Mask
 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT2      0x8000  /* IM_DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT2_MASK 0x8000  /* IM_DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT2_SHIFT    15  /* IM_DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_IM_DSP_SHARED_WR_COLL_EINT2_WIDTH     1  /* IM_DSP_SHARED_WR_COLL_EINT2 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT2            0x4000  /* IM_SPK_SHUTDOWN_EINT2 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT2_MASK       0x4000  /* IM_SPK_SHUTDOWN_EINT2 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT2_SHIFT          14  /* IM_SPK_SHUTDOWN_EINT2 */
#define CS47L24_IM_SPK_SHUTDOWN_EINT2_WIDTH           1  /* IM_SPK_SHUTDOWN_EINT2 */
#define CS47L24_IM_SPK1R_SHORT_EINT2             0x2000  /* IM_SPK1R_SHORT_EINT2 */
#define CS47L24_IM_SPK1R_SHORT_EINT2_MASK        0x2000  /* IM_SPK1R_SHORT_EINT2 */
#define CS47L24_IM_SPK1R_SHORT_EINT2_SHIFT           13  /* IM_SPK1R_SHORT_EINT2 */
#define CS47L24_IM_SPK1R_SHORT_EINT2_WIDTH            1  /* IM_SPK1R_SHORT_EINT2 */
#define CS47L24_IM_SPK1L_SHORT_EINT2             0x1000  /* IM_SPK1L_SHORT_EINT2 */
#define CS47L24_IM_SPK1L_SHORT_EINT2_MASK        0x1000  /* IM_SPK1L_SHORT_EINT2 */
#define CS47L24_IM_SPK1L_SHORT_EINT2_SHIFT           12  /* IM_SPK1L_SHORT_EINT2 */
#define CS47L24_IM_SPK1L_SHORT_EINT2_WIDTH            1  /* IM_SPK1L_SHORT_EINT2 */
#define CS47L24_IM_HP1R_SC_NEG_EINT2             0x0008  /* IM_HP1R_SC_NEG_EINT2 */
#define CS47L24_IM_HP1R_SC_NEG_EINT2_MASK        0x0008  /* IM_HP1R_SC_NEG_EINT2 */
#define CS47L24_IM_HP1R_SC_NEG_EINT2_SHIFT            3  /* IM_HP1R_SC_NEG_EINT2 */
#define CS47L24_IM_HP1R_SC_NEG_EINT2_WIDTH            1  /* IM_HP1R_SC_NEG_EINT2 */
#define CS47L24_IM_HP1R_SC_POS_EINT2             0x0004  /* IM_HP1R_SC_POS_EINT2 */
#define CS47L24_IM_HP1R_SC_POS_EINT2_MASK        0x0004  /* IM_HP1R_SC_POS_EINT2 */
#define CS47L24_IM_HP1R_SC_POS_EINT2_SHIFT            2  /* IM_HP1R_SC_POS_EINT2 */
#define CS47L24_IM_HP1R_SC_POS_EINT2_WIDTH            1  /* IM_HP1R_SC_POS_EINT2 */
#define CS47L24_IM_HP1L_SC_NEG_EINT2             0x0002  /* IM_HP1L_SC_NEG_EINT2 */
#define CS47L24_IM_HP1L_SC_NEG_EINT2_MASK        0x0002  /* IM_HP1L_SC_NEG_EINT2 */
#define CS47L24_IM_HP1L_SC_NEG_EINT2_SHIFT            1  /* IM_HP1L_SC_NEG_EINT2 */
#define CS47L24_IM_HP1L_SC_NEG_EINT2_WIDTH            1  /* IM_HP1L_SC_NEG_EINT2 */
#define CS47L24_IM_HP1L_SC_POS_EINT2             0x0001  /* IM_HP1L_SC_POS_EINT2 */
#define CS47L24_IM_HP1L_SC_POS_EINT2_MASK        0x0001  /* IM_HP1L_SC_POS_EINT2 */
#define CS47L24_IM_HP1L_SC_POS_EINT2_SHIFT            0  /* IM_HP1L_SC_POS_EINT2 */
#define CS47L24_IM_HP1L_SC_POS_EINT2_WIDTH            1  /* IM_HP1L_SC_POS_EINT2 */

/*
 * R3359 (0xD1F) - IRQ2 Control
 */
#define CS47L24_IM_IRQ2                          0x0001  /* IM_IRQ2 */
#define CS47L24_IM_IRQ2_MASK                     0x0001  /* IM_IRQ2 */
#define CS47L24_IM_IRQ2_SHIFT                         0  /* IM_IRQ2 */
#define CS47L24_IM_IRQ2_WIDTH                         1  /* IM_IRQ2 */

/*
 * R3360 (0xD20) - Interrupt Raw Status 1
 */
#define CS47L24_DSP3_RAM_RDY_STS                 0x0400  /* DSP3_RAM_RDY_STS */
#define CS47L24_DSP3_RAM_RDY_STS_MASK            0x0400  /* DSP3_RAM_RDY_STS */
#define CS47L24_DSP3_RAM_RDY_STS_SHIFT               10  /* DSP3_RAM_RDY_STS */
#define CS47L24_DSP3_RAM_RDY_STS_WIDTH                1  /* DSP3_RAM_RDY_STS */
#define CS47L24_DSP2_RAM_RDY_STS                 0x0200  /* DSP2_RAM_RDY_STS */
#define CS47L24_DSP2_RAM_RDY_STS_MASK            0x0200  /* DSP2_RAM_RDY_STS */
#define CS47L24_DSP2_RAM_RDY_STS_SHIFT                9  /* DSP2_RAM_RDY_STS */
#define CS47L24_DSP2_RAM_RDY_STS_WIDTH                1  /* DSP2_RAM_RDY_STS */
#define CS47L24_DSP_IRQ4_STS                     0x0008  /* DSP_IRQ4_STS */
#define CS47L24_DSP_IRQ4_STS_MASK                0x0008  /* DSP_IRQ4_STS */
#define CS47L24_DSP_IRQ4_STS_SHIFT                    3  /* DSP_IRQ4_STS */
#define CS47L24_DSP_IRQ4_STS_WIDTH                    1  /* DSP_IRQ4_STS */
#define CS47L24_DSP_IRQ3_STS                     0x0004  /* DSP_IRQ3_STS */
#define CS47L24_DSP_IRQ3_STS_MASK                0x0004  /* DSP_IRQ3_STS */
#define CS47L24_DSP_IRQ3_STS_SHIFT                    2  /* DSP_IRQ3_STS */
#define CS47L24_DSP_IRQ3_STS_WIDTH                    1  /* DSP_IRQ3_STS */
#define CS47L24_DSP_IRQ2_STS                     0x0002  /* DSP_IRQ2_STS */
#define CS47L24_DSP_IRQ2_STS_MASK                0x0002  /* DSP_IRQ2_STS */
#define CS47L24_DSP_IRQ2_STS_SHIFT                    1  /* DSP_IRQ2_STS */
#define CS47L24_DSP_IRQ2_STS_WIDTH                    1  /* DSP_IRQ2_STS */
#define CS47L24_DSP_IRQ1_STS                     0x0001  /* DSP_IRQ1_STS */
#define CS47L24_DSP_IRQ1_STS_MASK                0x0001  /* DSP_IRQ1_STS */
#define CS47L24_DSP_IRQ1_STS_SHIFT                    0  /* DSP_IRQ1_STS */
#define CS47L24_DSP_IRQ1_STS_WIDTH                    1  /* DSP_IRQ1_STS */

/*
 * R3361 (0xD21) - Interrupt Raw Status 2
 */
#define CS47L24_SPK_OVERHEAT_WARN_STS            0x8000  /* SPK_OVERHEAT_WARN_STS */
#define CS47L24_SPK_OVERHEAT_WARN_STS_MASK       0x8000  /* SPK_OVERHEAT_WARN_STS */
#define CS47L24_SPK_OVERHEAT_WARN_STS_SHIFT          15  /* SPK_OVERHEAT_WARN_STS */
#define CS47L24_SPK_OVERHEAT_WARN_STS_WIDTH           1  /* SPK_OVERHEAT_WARN_STS */
#define CS47L24_SPK_OVERHEAT_STS                 0x4000  /* SPK_OVERHEAT_STS */
#define CS47L24_SPK_OVERHEAT_STS_MASK            0x4000  /* SPK_OVERHEAT_STS */
#define CS47L24_SPK_OVERHEAT_STS_SHIFT               14  /* SPK_OVERHEAT_STS */
#define CS47L24_SPK_OVERHEAT_STS_WIDTH                1  /* SPK_OVERHEAT_STS */
#define CS47L24_WSEQ_DONE_STS                    0x0800  /* WSEQ_DONE_STS */
#define CS47L24_WSEQ_DONE_STS_MASK               0x0800  /* WSEQ_DONE_STS */
#define CS47L24_WSEQ_DONE_STS_SHIFT                  11  /* WSEQ_DONE_STS */
#define CS47L24_WSEQ_DONE_STS_WIDTH                   1  /* WSEQ_DONE_STS */
#define CS47L24_DRC2_SIG_DET_STS                 0x0400  /* DRC2_SIG_DET_STS */
#define CS47L24_DRC2_SIG_DET_STS_MASK            0x0400  /* DRC2_SIG_DET_STS */
#define CS47L24_DRC2_SIG_DET_STS_SHIFT               10  /* DRC2_SIG_DET_STS */
#define CS47L24_DRC2_SIG_DET_STS_WIDTH                1  /* DRC2_SIG_DET_STS */
#define CS47L24_DRC1_SIG_DET_STS                 0x0200  /* DRC1_SIG_DET_STS */
#define CS47L24_DRC1_SIG_DET_STS_MASK            0x0200  /* DRC1_SIG_DET_STS */
#define CS47L24_DRC1_SIG_DET_STS_SHIFT                9  /* DRC1_SIG_DET_STS */
#define CS47L24_DRC1_SIG_DET_STS_WIDTH                1  /* DRC1_SIG_DET_STS */
#define CS47L24_ASRC2_LOCK_STS                   0x0100  /* ASRC2_LOCK_STS */
#define CS47L24_ASRC2_LOCK_STS_MASK              0x0100  /* ASRC2_LOCK_STS */
#define CS47L24_ASRC2_LOCK_STS_SHIFT                  8  /* ASRC2_LOCK_STS */
#define CS47L24_ASRC2_LOCK_STS_WIDTH                  1  /* ASRC2_LOCK_STS */
#define CS47L24_ASRC1_LOCK_STS                   0x0080  /* ASRC1_LOCK_STS */
#define CS47L24_ASRC1_LOCK_STS_MASK              0x0080  /* ASRC1_LOCK_STS */
#define CS47L24_ASRC1_LOCK_STS_SHIFT                  7  /* ASRC1_LOCK_STS */
#define CS47L24_ASRC1_LOCK_STS_WIDTH                  1  /* ASRC1_LOCK_STS */
#define CS47L24_UNDERCLOCKED_STS                 0x0040  /* UNDERCLOCKED_STS */
#define CS47L24_UNDERCLOCKED_STS_MASK            0x0040  /* UNDERCLOCKED_STS */
#define CS47L24_UNDERCLOCKED_STS_SHIFT                6  /* UNDERCLOCKED_STS */
#define CS47L24_UNDERCLOCKED_STS_WIDTH                1  /* UNDERCLOCKED_STS */
#define CS47L24_OVERCLOCKED_STS                  0x0020  /* OVERCLOCKED_STS */
#define CS47L24_OVERCLOCKED_STS_MASK             0x0020  /* OVERCLOCKED_STS */
#define CS47L24_OVERCLOCKED_STS_SHIFT                 5  /* OVERCLOCKED_STS */
#define CS47L24_OVERCLOCKED_STS_WIDTH                 1  /* OVERCLOCKED_STS */
#define CS47L24_FLL2_LOCK_STS                    0x0008  /* FLL2_LOCK_STS */
#define CS47L24_FLL2_LOCK_STS_MASK               0x0008  /* FLL2_LOCK_STS */
#define CS47L24_FLL2_LOCK_STS_SHIFT                   3  /* FLL2_LOCK_STS */
#define CS47L24_FLL2_LOCK_STS_WIDTH                   1  /* FLL2_LOCK_STS */
#define CS47L24_FLL1_LOCK_STS                    0x0004  /* FLL1_LOCK_STS */
#define CS47L24_FLL1_LOCK_STS_MASK               0x0004  /* FLL1_LOCK_STS */
#define CS47L24_FLL1_LOCK_STS_SHIFT                   2  /* FLL1_LOCK_STS */
#define CS47L24_FLL1_LOCK_STS_WIDTH                   1  /* FLL1_LOCK_STS */
#define CS47L24_CLKGEN_ERR_STS                   0x0002  /* CLKGEN_ERR_STS */
#define CS47L24_CLKGEN_ERR_STS_MASK              0x0002  /* CLKGEN_ERR_STS */
#define CS47L24_CLKGEN_ERR_STS_SHIFT                  1  /* CLKGEN_ERR_STS */
#define CS47L24_CLKGEN_ERR_STS_WIDTH                  1  /* CLKGEN_ERR_STS */
#define CS47L24_CLKGEN_ERR_ASYNC_STS             0x0001  /* CLKGEN_ERR_ASYNC_STS */
#define CS47L24_CLKGEN_ERR_ASYNC_STS_MASK        0x0001  /* CLKGEN_ERR_ASYNC_STS */
#define CS47L24_CLKGEN_ERR_ASYNC_STS_SHIFT            0  /* CLKGEN_ERR_ASYNC_STS */
#define CS47L24_CLKGEN_ERR_ASYNC_STS_WIDTH            1  /* CLKGEN_ERR_ASYNC_STS */

/*
 * R3362 (0xD22) - Interrupt Raw Status 3
 */
#define CS47L24_CTRLIF_ERR_STS                   0x1000  /* CTRLIF_ERR_STS */
#define CS47L24_CTRLIF_ERR_STS_MASK              0x1000  /* CTRLIF_ERR_STS */
#define CS47L24_CTRLIF_ERR_STS_SHIFT                 12  /* CTRLIF_ERR_STS */
#define CS47L24_CTRLIF_ERR_STS_WIDTH                  1  /* CTRLIF_ERR_STS */
#define CS47L24_MIXER_DROPPED_SAMPLE_STS         0x0800  /* MIXER_DROPPED_SAMPLE_STS */
#define CS47L24_MIXER_DROPPED_SAMPLE_STS_MASK    0x0800  /* MIXER_DROPPED_SAMPLE_STS */
#define CS47L24_MIXER_DROPPED_SAMPLE_STS_SHIFT       11  /* MIXER_DROPPED_SAMPLE_STS */
#define CS47L24_MIXER_DROPPED_SAMPLE_STS_WIDTH        1  /* MIXER_DROPPED_SAMPLE_STS */
#define CS47L24_ASYNC_CLK_ENA_LOW_STS            0x0400  /* ASYNC_CLK_ENA_LOW_STS */
#define CS47L24_ASYNC_CLK_ENA_LOW_STS_MASK       0x0400  /* ASYNC_CLK_ENA_LOW_STS */
#define CS47L24_ASYNC_CLK_ENA_LOW_STS_SHIFT          10  /* ASYNC_CLK_ENA_LOW_STS */
#define CS47L24_ASYNC_CLK_ENA_LOW_STS_WIDTH           1  /* ASYNC_CLK_ENA_LOW_STS */
#define CS47L24_SYSCLK_ENA_LOW_STS               0x0200  /* SYSCLK_ENA_LOW_STS */
#define CS47L24_SYSCLK_ENA_LOW_STS_MASK          0x0200  /* SYSCLK_ENA_LOW_STS */
#define CS47L24_SYSCLK_ENA_LOW_STS_SHIFT              9  /* SYSCLK_ENA_LOW_STS */
#define CS47L24_SYSCLK_ENA_LOW_STS_WIDTH              1  /* SYSCLK_ENA_LOW_STS */
#define CS47L24_ISRC1_CFG_ERR_STS                0x0100  /* ISRC1_CFG_ERR_STS */
#define CS47L24_ISRC1_CFG_ERR_STS_MASK           0x0100  /* ISRC1_CFG_ERR_STS */
#define CS47L24_ISRC1_CFG_ERR_STS_SHIFT               8  /* ISRC1_CFG_ERR_STS */
#define CS47L24_ISRC1_CFG_ERR_STS_WIDTH               1  /* ISRC1_CFG_ERR_STS */
#define CS47L24_ISRC2_CFG_ERR_STS                0x0080  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC2_CFG_ERR_STS_MASK           0x0080  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC2_CFG_ERR_STS_SHIFT               7  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC2_CFG_ERR_STS_WIDTH               1  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC3_CFG_ERR_STS                0x0040  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC3_CFG_ERR_STS_MASK           0x0040  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC3_CFG_ERR_STS_SHIFT               6  /* ISRC2_CFG_ERR_STS */
#define CS47L24_ISRC3_CFG_ERR_STS_WIDTH               1  /* ISRC2_CFG_ERR_STS */

/*
 * R3363 (0xD23) - Interrupt Raw Status 4
 */
#define CS47L24_BOOT_DONE_STS                    0x0100  /* BOOT_DONE_STS     */
#define CS47L24_BOOT_DONE_STS_MASK               0x0100  /* BOOT_DONE_STS     */
#define CS47L24_BOOT_DONE_STS_SHIFT                   8  /* BOOT_DONE_STS     */
#define CS47L24_BOOT_DONE_STS_WIDTH                   1  /* BOOT_DONE_STS     */
#define CS47L24_ASRC_CFG_ERR_STS                 0x0008  /* ASRC_CFG_ERR_STS  */
#define CS47L24_ASRC_CFG_ERR_STS_MASK            0x0008  /* ASRC_CFG_ERR_STS  */
#define CS47L24_ASRC_CFG_ERR_STS_SHIFT                3  /* ASRC_CFG_ERR_STS  */
#define CS47L24_ASRC_CFG_ERR_STS_WIDTH                1  /* ASRC_CFG_ERR_STS  */
#define CS47L24_FLL2_CLOCK_OK_STS                0x0002  /* FLL2_CLOCK_OK_STS */
#define CS47L24_FLL2_CLOCK_OK_STS_MASK           0x0002  /* FLL2_CLOCK_OK_STS */
#define CS47L24_FLL2_CLOCK_OK_STS_SHIFT               1  /* FLL2_CLOCK_OK_STS */
#define CS47L24_FLL2_CLOCK_OK_STS_WIDTH               1  /* FLL2_CLOCK_OK_STS */
#define CS47L24_FLL1_CLOCK_OK_STS                0x0001  /* FLL1_CLOCK_OK_STS */
#define CS47L24_FLL1_CLOCK_OK_STS_MASK           0x0001  /* FLL1_CLOCK_OK_STS */
#define CS47L24_FLL1_CLOCK_OK_STS_SHIFT               0  /* FLL1_CLOCK_OK_STS */
#define CS47L24_FLL1_CLOCK_OK_STS_WIDTH               1  /* FLL1_CLOCK_OK_STS */

/*
 * R3364 (0xD24) - Interrupt Raw Status 5
 */
#define CS47L24_PWM_OVERCLOCKED_STS              0x2000  /* PWM_OVERCLOCKED_STS */
#define CS47L24_PWM_OVERCLOCKED_STS_MASK         0x2000  /* PWM_OVERCLOCKED_STS */
#define CS47L24_PWM_OVERCLOCKED_STS_SHIFT            13  /* PWM_OVERCLOCKED_STS */
#define CS47L24_PWM_OVERCLOCKED_STS_WIDTH             1  /* PWM_OVERCLOCKED_STS */
#define CS47L24_FX_CORE_OVERCLOCKED_STS          0x1000  /* FX_CORE_OVERCLOCKED_STS */
#define CS47L24_FX_CORE_OVERCLOCKED_STS_MASK     0x1000  /* FX_CORE_OVERCLOCKED_STS */
#define CS47L24_FX_CORE_OVERCLOCKED_STS_SHIFT        12  /* FX_CORE_OVERCLOCKED_STS */
#define CS47L24_FX_CORE_OVERCLOCKED_STS_WIDTH         1  /* FX_CORE_OVERCLOCKED_STS */
#define CS47L24_DAC_SYS_OVERCLOCKED_STS          0x0400  /* DAC_SYS_OVERCLOCKED_STS */
#define CS47L24_DAC_SYS_OVERCLOCKED_STS_MASK     0x0400  /* DAC_SYS_OVERCLOCKED_STS */
#define CS47L24_DAC_SYS_OVERCLOCKED_STS_SHIFT        10  /* DAC_SYS_OVERCLOCKED_STS */
#define CS47L24_DAC_SYS_OVERCLOCKED_STS_WIDTH         1  /* DAC_SYS_OVERCLOCKED_STS */
#define CS47L24_DAC_WARP_OVERCLOCKED_STS         0x0200  /* DAC_WARP_OVERCLOCKED_STS */
#define CS47L24_DAC_WARP_OVERCLOCKED_STS_MASK    0x0200  /* DAC_WARP_OVERCLOCKED_STS */
#define CS47L24_DAC_WARP_OVERCLOCKED_STS_SHIFT        9  /* DAC_WARP_OVERCLOCKED_STS */
#define CS47L24_DAC_WARP_OVERCLOCKED_STS_WIDTH        1  /* DAC_WARP_OVERCLOCKED_STS */
#define CS47L24_ADC_OVERCLOCKED_STS              0x0100  /* ADC_OVERCLOCKED_STS */
#define CS47L24_ADC_OVERCLOCKED_STS_MASK         0x0100  /* ADC_OVERCLOCKED_STS */
#define CS47L24_ADC_OVERCLOCKED_STS_SHIFT             8  /* ADC_OVERCLOCKED_STS */
#define CS47L24_ADC_OVERCLOCKED_STS_WIDTH             1  /* ADC_OVERCLOCKED_STS */
#define CS47L24_MIXER_OVERCLOCKED_STS            0x0080  /* MIXER_OVERCLOCKED_STS */
#define CS47L24_MIXER_OVERCLOCKED_STS_MASK       0x0080  /* MIXER_OVERCLOCKED_STS */
#define CS47L24_MIXER_OVERCLOCKED_STS_SHIFT           7  /* MIXER_OVERCLOCKED_STS */
#define CS47L24_MIXER_OVERCLOCKED_STS_WIDTH           1  /* MIXER_OVERCLOCKED_STS */
#define CS47L24_AIF3_ASYNC_OVERCLOCKED_STS       0x0040  /* AIF3_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_ASYNC_OVERCLOCKED_STS_MASK  0x0040  /* AIF3_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_ASYNC_OVERCLOCKED_STS_SHIFT      6  /* AIF3_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_ASYNC_OVERCLOCKED_STS_WIDTH      1  /* AIF3_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_ASYNC_OVERCLOCKED_STS       0x0020  /* AIF2_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_ASYNC_OVERCLOCKED_STS_MASK  0x0020  /* AIF2_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_ASYNC_OVERCLOCKED_STS_SHIFT      5  /* AIF2_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_ASYNC_OVERCLOCKED_STS_WIDTH      1  /* AIF2_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_ASYNC_OVERCLOCKED_STS       0x0010  /* AIF1_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_ASYNC_OVERCLOCKED_STS_MASK  0x0010  /* AIF1_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_ASYNC_OVERCLOCKED_STS_SHIFT      4  /* AIF1_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_ASYNC_OVERCLOCKED_STS_WIDTH      1  /* AIF1_ASYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_SYNC_OVERCLOCKED_STS        0x0008  /* AIF3_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_SYNC_OVERCLOCKED_STS_MASK   0x0008  /* AIF3_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_SYNC_OVERCLOCKED_STS_SHIFT       3  /* AIF3_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF3_SYNC_OVERCLOCKED_STS_WIDTH       1  /* AIF3_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_SYNC_OVERCLOCKED_STS        0x0004  /* AIF2_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_SYNC_OVERCLOCKED_STS_MASK   0x0004  /* AIF2_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_SYNC_OVERCLOCKED_STS_SHIFT       2  /* AIF2_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF2_SYNC_OVERCLOCKED_STS_WIDTH       1  /* AIF2_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_SYNC_OVERCLOCKED_STS        0x0002  /* AIF1_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_SYNC_OVERCLOCKED_STS_MASK   0x0002  /* AIF1_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_SYNC_OVERCLOCKED_STS_SHIFT       1  /* AIF1_SYNC_OVERCLOCKED_STS */
#define CS47L24_AIF1_SYNC_OVERCLOCKED_STS_WIDTH       1  /* AIF1_SYNC_OVERCLOCKED_STS */
#define CS47L24_PAD_CTRL_OVERCLOCKED_STS         0x0001  /* PAD_CTRL_OVERCLOCKED_STS */
#define CS47L24_PAD_CTRL_OVERCLOCKED_STS_MASK    0x0001  /* PAD_CTRL_OVERCLOCKED_STS */
#define CS47L24_PAD_CTRL_OVERCLOCKED_STS_SHIFT        0  /* PAD_CTRL_OVERCLOCKED_STS */
#define CS47L24_PAD_CTRL_OVERCLOCKED_STS_WIDTH        1  /* PAD_CTRL_OVERCLOCKED_STS */

/*
 * R3365 (0xD25) - Interrupt Raw Status 6
 */
#define CS47L24_ASRC_ASYNC_SYS_OVERCLOCKED_STS       0x1000  /* ASRC_ASYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_SYS_OVERCLOCKED_STS_MASK  0x1000  /* ASRC_ASYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_SYS_OVERCLOCKED_STS_SHIFT     12  /* ASRC_ASYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_SYS_OVERCLOCKED_STS_WIDTH      1  /* ASRC_ASYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_WARP_OVERCLOCKED_STS      0x0800  /* ASRC_ASYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_WARP_OVERCLOCKED_STS_MASK 0x0800  /* ASRC_ASYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_WARP_OVERCLOCKED_STS_SHIFT    11  /* ASRC_ASYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_ASYNC_WARP_OVERCLOCKED_STS_WIDTH     1  /* ASRC_ASYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_SYS_OVERCLOCKED_STS        0x0400  /* ASRC_SYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_SYS_OVERCLOCKED_STS_MASK   0x0400  /* ASRC_SYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_SYS_OVERCLOCKED_STS_SHIFT      10  /* ASRC_SYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_SYS_OVERCLOCKED_STS_WIDTH       1  /* ASRC_SYNC_SYS_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_WARP_OVERCLOCKED_STS       0x0200  /* ASRC_SYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_WARP_OVERCLOCKED_STS_MASK  0x0200  /* ASRC_SYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_WARP_OVERCLOCKED_STS_SHIFT      9  /* ASRC_SYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ASRC_SYNC_WARP_OVERCLOCKED_STS_WIDTH      1  /* ASRC_SYNC_WARP_OVERCLOCKED_STS */
#define CS47L24_ADSP2_1_OVERCLOCKED_STS              0x0008  /* ADSP2_1_OVERCLOCKED_STS */
#define CS47L24_ADSP2_1_OVERCLOCKED_STS_MASK         0x0008  /* ADSP2_1_OVERCLOCKED_STS */
#define CS47L24_ADSP2_1_OVERCLOCKED_STS_SHIFT             3  /* ADSP2_1_OVERCLOCKED_STS */
#define CS47L24_ADSP2_1_OVERCLOCKED_STS_WIDTH             1  /* ADSP2_1_OVERCLOCKED_STS */
#define CS47L24_ISRC3_OVERCLOCKED_STS                0x0004  /* ISRC3_OVERCLOCKED_STS */
#define CS47L24_ISRC3_OVERCLOCKED_STS_MASK           0x0004  /* ISRC3_OVERCLOCKED_STS */
#define CS47L24_ISRC3_OVERCLOCKED_STS_SHIFT               2  /* ISRC3_OVERCLOCKED_STS */
#define CS47L24_ISRC3_OVERCLOCKED_STS_WIDTH               1  /* ISRC3_OVERCLOCKED_STS */
#define CS47L24_ISRC2_OVERCLOCKED_STS                0x0002  /* ISRC2_OVERCLOCKED_STS */
#define CS47L24_ISRC2_OVERCLOCKED_STS_MASK           0x0002  /* ISRC2_OVERCLOCKED_STS */
#define CS47L24_ISRC2_OVERCLOCKED_STS_SHIFT               1  /* ISRC2_OVERCLOCKED_STS */
#define CS47L24_ISRC2_OVERCLOCKED_STS_WIDTH               1  /* ISRC2_OVERCLOCKED_STS */
#define CS47L24_ISRC1_OVERCLOCKED_STS                0x0001  /* ISRC1_OVERCLOCKED_STS */
#define CS47L24_ISRC1_OVERCLOCKED_STS_MASK           0x0001  /* ISRC1_OVERCLOCKED_STS */
#define CS47L24_ISRC1_OVERCLOCKED_STS_SHIFT               0  /* ISRC1_OVERCLOCKED_STS */
#define CS47L24_ISRC1_OVERCLOCKED_STS_WIDTH               1  /* ISRC1_OVERCLOCKED_STS */

/*
 * R3366 (0xD26) - Interrupt Raw Status 7
 */
#define CS47L24_AIF3_UNDERCLOCKED_STS            0x0400  /* AIF3_UNDERCLOCKED_STS */
#define CS47L24_AIF3_UNDERCLOCKED_STS_MASK       0x0400  /* AIF3_UNDERCLOCKED_STS */
#define CS47L24_AIF3_UNDERCLOCKED_STS_SHIFT          10  /* AIF3_UNDERCLOCKED_STS */
#define CS47L24_AIF3_UNDERCLOCKED_STS_WIDTH           1  /* AIF3_UNDERCLOCKED_STS */
#define CS47L24_AIF2_UNDERCLOCKED_STS            0x0200  /* AIF2_UNDERCLOCKED_STS */
#define CS47L24_AIF2_UNDERCLOCKED_STS_MASK       0x0200  /* AIF2_UNDERCLOCKED_STS */
#define CS47L24_AIF2_UNDERCLOCKED_STS_SHIFT           9  /* AIF2_UNDERCLOCKED_STS */
#define CS47L24_AIF2_UNDERCLOCKED_STS_WIDTH           1  /* AIF2_UNDERCLOCKED_STS */
#define CS47L24_AIF1_UNDERCLOCKED_STS            0x0100  /* AIF1_UNDERCLOCKED_STS */
#define CS47L24_AIF1_UNDERCLOCKED_STS_MASK       0x0100  /* AIF1_UNDERCLOCKED_STS */
#define CS47L24_AIF1_UNDERCLOCKED_STS_SHIFT           8  /* AIF1_UNDERCLOCKED_STS */
#define CS47L24_AIF1_UNDERCLOCKED_STS_WIDTH           1  /* AIF1_UNDERCLOCKED_STS */
#define CS47L24_ISRC3_UNDERCLOCKED_STS           0x0080  /* ISRC3_UNDERCLOCKED_STS */
#define CS47L24_ISRC3_UNDERCLOCKED_STS_MASK      0x0080  /* ISRC3_UNDERCLOCKED_STS */
#define CS47L24_ISRC3_UNDERCLOCKED_STS_SHIFT          7  /* ISRC3_UNDERCLOCKED_STS */
#define CS47L24_ISRC3_UNDERCLOCKED_STS_WIDTH          1  /* ISRC3_UNDERCLOCKED_STS */
#define CS47L24_ISRC2_UNDERCLOCKED_STS           0x0040  /* ISRC2_UNDERCLOCKED_STS */
#define CS47L24_ISRC2_UNDERCLOCKED_STS_MASK      0x0040  /* ISRC2_UNDERCLOCKED_STS */
#define CS47L24_ISRC2_UNDERCLOCKED_STS_SHIFT          6  /* ISRC2_UNDERCLOCKED_STS */
#define CS47L24_ISRC2_UNDERCLOCKED_STS_WIDTH          1  /* ISRC2_UNDERCLOCKED_STS */
#define CS47L24_ISRC1_UNDERCLOCKED_STS           0x0020  /* ISRC1_UNDERCLOCKED_STS */
#define CS47L24_ISRC1_UNDERCLOCKED_STS_MASK      0x0020  /* ISRC1_UNDERCLOCKED_STS */
#define CS47L24_ISRC1_UNDERCLOCKED_STS_SHIFT          5  /* ISRC1_UNDERCLOCKED_STS */
#define CS47L24_ISRC1_UNDERCLOCKED_STS_WIDTH          1  /* ISRC1_UNDERCLOCKED_STS */
#define CS47L24_FX_UNDERCLOCKED_STS              0x0010  /* FX_UNDERCLOCKED_STS */
#define CS47L24_FX_UNDERCLOCKED_STS_MASK         0x0010  /* FX_UNDERCLOCKED_STS */
#define CS47L24_FX_UNDERCLOCKED_STS_SHIFT             4  /* FX_UNDERCLOCKED_STS */
#define CS47L24_FX_UNDERCLOCKED_STS_WIDTH             1  /* FX_UNDERCLOCKED_STS */
#define CS47L24_ASRC_UNDERCLOCKED_STS            0x0008  /* ASRC_UNDERCLOCKED_STS */
#define CS47L24_ASRC_UNDERCLOCKED_STS_MASK       0x0008  /* ASRC_UNDERCLOCKED_STS */
#define CS47L24_ASRC_UNDERCLOCKED_STS_SHIFT           3  /* ASRC_UNDERCLOCKED_STS */
#define CS47L24_ASRC_UNDERCLOCKED_STS_WIDTH           1  /* ASRC_UNDERCLOCKED_STS */
#define CS47L24_DAC_UNDERCLOCKED_STS             0x0004  /* DAC_UNDERCLOCKED_STS */
#define CS47L24_DAC_UNDERCLOCKED_STS_MASK        0x0004  /* DAC_UNDERCLOCKED_STS */
#define CS47L24_DAC_UNDERCLOCKED_STS_SHIFT            2  /* DAC_UNDERCLOCKED_STS */
#define CS47L24_DAC_UNDERCLOCKED_STS_WIDTH            1  /* DAC_UNDERCLOCKED_STS */
#define CS47L24_ADC_UNDERCLOCKED_STS             0x0002  /* ADC_UNDERCLOCKED_STS */
#define CS47L24_ADC_UNDERCLOCKED_STS_MASK        0x0002  /* ADC_UNDERCLOCKED_STS */
#define CS47L24_ADC_UNDERCLOCKED_STS_SHIFT            1  /* ADC_UNDERCLOCKED_STS */
#define CS47L24_ADC_UNDERCLOCKED_STS_WIDTH            1  /* ADC_UNDERCLOCKED_STS */
#define CS47L24_MIXER_UNDERCLOCKED_STS           0x0001  /* MIXER_UNDERCLOCKED_STS */
#define CS47L24_MIXER_UNDERCLOCKED_STS_MASK      0x0001  /* MIXER_UNDERCLOCKED_STS */
#define CS47L24_MIXER_UNDERCLOCKED_STS_SHIFT          0  /* MIXER_UNDERCLOCKED_STS */
#define CS47L24_MIXER_UNDERCLOCKED_STS_WIDTH          1  /* MIXER_UNDERCLOCKED_STS */

/*
 * R3368 (0xD28) - Interrupt Raw Status 8
 */
#define CS47L24_DSP_SHARED_WR_COLL_STS           0x8000  /* DSP_SHARED_WR_COLL_STS */
#define CS47L24_DSP_SHARED_WR_COLL_STS_MASK      0x8000  /* DSP_SHARED_WR_COLL_STS */
#define CS47L24_DSP_SHARED_WR_COLL_STS_SHIFT         15  /* DSP_SHARED_WR_COLL_STS */
#define CS47L24_DSP_SHARED_WR_COLL_STS_WIDTH          1  /* DSP_SHARED_WR_COLL_STS */
#define CS47L24_SPK_SHUTDOWN_STS                 0x4000  /* SPK_SHUTDOWN_STS */
#define CS47L24_SPK_SHUTDOWN_STS_MASK            0x4000  /* SPK_SHUTDOWN_STS */
#define CS47L24_SPK_SHUTDOWN_STS_SHIFT               14  /* SPK_SHUTDOWN_STS */
#define CS47L24_SPK_SHUTDOWN_STS_WIDTH                1  /* SPK_SHUTDOWN_STS */
#define CS47L24_SPK1R_SHORT_STS                  0x2000  /* SPK1R_SHORT_STS */
#define CS47L24_SPK1R_SHORT_STS_MASK             0x2000  /* SPK1R_SHORT_STS */
#define CS47L24_SPK1R_SHORT_STS_SHIFT                13  /* SPK1R_SHORT_STS */
#define CS47L24_SPK1R_SHORT_STS_WIDTH                 1  /* SPK1R_SHORT_STS */
#define CS47L24_SPK1L_SHORT_STS                  0x1000  /* SPK1L_SHORT_STS */
#define CS47L24_SPK1L_SHORT_STS_MASK             0x1000  /* SPK1L_SHORT_STS */
#define CS47L24_SPK1L_SHORT_STS_SHIFT                12  /* SPK1L_SHORT_STS */
#define CS47L24_SPK1L_SHORT_STS_WIDTH                 1  /* SPK1L_SHORT_STS */
#define CS47L24_HP1R_SC_NEG_STS                  0x0008  /* HP1R_SC_NEG_STS */
#define CS47L24_HP1R_SC_NEG_STS_MASK             0x0008  /* HP1R_SC_NEG_STS */
#define CS47L24_HP1R_SC_NEG_STS_SHIFT                 3  /* HP1R_SC_NEG_STS */
#define CS47L24_HP1R_SC_NEG_STS_WIDTH                 1  /* HP1R_SC_NEG_STS */
#define CS47L24_HP1R_SC_POS_STS                  0x0004  /* HP1R_SC_POS_STS */
#define CS47L24_HP1R_SC_POS_STS_MASK             0x0004  /* HP1R_SC_POS_STS */
#define CS47L24_HP1R_SC_POS_STS_SHIFT                 2  /* HP1R_SC_POS_STS */
#define CS47L24_HP1R_SC_POS_STS_WIDTH                 1  /* HP1R_SC_POS_STS */
#define CS47L24_HP1L_SC_NEG_STS                  0x0002  /* HP1L_SC_NEG_STS */
#define CS47L24_HP1L_SC_NEG_STS_MASK             0x0002  /* HP1L_SC_NEG_STS */
#define CS47L24_HP1L_SC_NEG_STS_SHIFT                 1  /* HP1L_SC_NEG_STS */
#define CS47L24_HP1L_SC_NEG_STS_WIDTH                 1  /* HP1L_SC_NEG_STS */
#define CS47L24_HP1L_SC_POS_STS                  0x0001  /* HP1L_SC_POS_STS */
#define CS47L24_HP1L_SC_POS_STS_MASK             0x0001  /* HP1L_SC_POS_STS */
#define CS47L24_HP1L_SC_POS_STS_SHIFT                 0  /* HP1L_SC_POS_STS */
#define CS47L24_HP1L_SC_POS_STS_WIDTH                 1  /* HP1L_SC_POS_STS */

/*
 * R3392 (0xD40) - IRQ Pin Status
 */
#define CS47L24_IRQ2_STS                         0x0002  /* IRQ2_STS */
#define CS47L24_IRQ2_STS_MASK                    0x0002  /* IRQ2_STS */
#define CS47L24_IRQ2_STS_SHIFT                        1  /* IRQ2_STS */
#define CS47L24_IRQ2_STS_WIDTH                        1  /* IRQ2_STS */
#define CS47L24_IRQ1_STS                         0x0001  /* IRQ1_STS */
#define CS47L24_IRQ1_STS_MASK                    0x0001  /* IRQ1_STS */
#define CS47L24_IRQ1_STS_SHIFT                        0  /* IRQ1_STS */
#define CS47L24_IRQ1_STS_WIDTH                        1  /* IRQ1_STS */

/*
 * R3393 (0xD41) - ADSP2 IRQ0
 */
#define CS47L24_DSP_IRQ2                         0x0002  /* DSP_IRQ2 */
#define CS47L24_DSP_IRQ2_MASK                    0x0002  /* DSP_IRQ2 */
#define CS47L24_DSP_IRQ2_SHIFT                        1  /* DSP_IRQ2 */
#define CS47L24_DSP_IRQ2_WIDTH                        1  /* DSP_IRQ2 */
#define CS47L24_DSP_IRQ1                         0x0001  /* DSP_IRQ1 */
#define CS47L24_DSP_IRQ1_MASK                    0x0001  /* DSP_IRQ1 */
#define CS47L24_DSP_IRQ1_SHIFT                        0  /* DSP_IRQ1 */
#define CS47L24_DSP_IRQ1_WIDTH                        1  /* DSP_IRQ1 */

/*
 * R3394 (0xD42) - ADSP2 IRQ1
 */
#define CS47L24_DSP_IRQ4                         0x0002  /* DSP_IRQ4 */
#define CS47L24_DSP_IRQ4_MASK                    0x0002  /* DSP_IRQ4 */
#define CS47L24_DSP_IRQ4_SHIFT                        1  /* DSP_IRQ4 */
#define CS47L24_DSP_IRQ4_WIDTH                        1  /* DSP_IRQ4 */
#define CS47L24_DSP_IRQ3                         0x0001  /* DSP_IRQ3 */
#define CS47L24_DSP_IRQ3_MASK                    0x0001  /* DSP_IRQ3 */
#define CS47L24_DSP_IRQ3_SHIFT                        0  /* DSP_IRQ3 */
#define CS47L24_DSP_IRQ3_WIDTH                        1  /* DSP_IRQ3 */

/*
 * R3395 (0xD43) - ADSP2 IRQ2
 */
#define CS47L24_DSP_IRQ6                         0x0002  /* DSP_IRQ6 */
#define CS47L24_DSP_IRQ6_MASK                    0x0002  /* DSP_IRQ6 */
#define CS47L24_DSP_IRQ6_SHIFT                        1  /* DSP_IRQ6 */
#define CS47L24_DSP_IRQ6_WIDTH                        1  /* DSP_IRQ6 */
#define CS47L24_DSP_IRQ5                         0x0001  /* DSP_IRQ5 */
#define CS47L24_DSP_IRQ5_MASK                    0x0001  /* DSP_IRQ5 */
#define CS47L24_DSP_IRQ5_SHIFT                        0  /* DSP_IRQ5 */
#define CS47L24_DSP_IRQ5_WIDTH                        1  /* DSP_IRQ5 */

/*
 * R3395 (0xD44) - ADSP2 IRQ3
 */
#define CS47L24_DSP_IRQ8                         0x0002  /* DSP_IRQ8 */
#define CS47L24_DSP_IRQ8_MASK                    0x0002  /* DSP_IRQ8 */
#define CS47L24_DSP_IRQ8_SHIFT                        1  /* DSP_IRQ8 */
#define CS47L24_DSP_IRQ8_WIDTH                        1  /* DSP_IRQ8 */
#define CS47L24_DSP_IRQ7                         0x0001  /* DSP_IRQ7 */
#define CS47L24_DSP_IRQ7_MASK                    0x0001  /* DSP_IRQ7 */
#define CS47L24_DSP_IRQ7_SHIFT                        0  /* DSP_IRQ7 */
#define CS47L24_DSP_IRQ7_WIDTH                        1  /* DSP_IRQ7 */

/*
 * R3584 (0xE00) - FX_Ctrl1
 */
#define CS47L24_FX_RATE_MASK                     0x7800  /* FX_RATE - [14:11] */
#define CS47L24_FX_RATE_SHIFT                        11  /* FX_RATE - [14:11] */
#define CS47L24_FX_RATE_WIDTH                         4  /* FX_RATE - [14:11] */

/*
 * R3585 (0xE01) - FX_Ctrl2
 */
#define CS47L24_FX_STS_MASK                      0xFFF0  /* FX_STS - [15:4] */
#define CS47L24_FX_STS_SHIFT                          4  /* FX_STS - [15:4] */
#define CS47L24_FX_STS_WIDTH                         12  /* FX_STS - [15:4] */

/*
 * R3600 (0xE10) - EQ1_1
 */
#define CS47L24_EQ1_B1_GAIN_MASK                 0xF800  /* EQ1_B1_GAIN - [15:11] */
#define CS47L24_EQ1_B1_GAIN_SHIFT                    11  /* EQ1_B1_GAIN - [15:11] */
#define CS47L24_EQ1_B1_GAIN_WIDTH                     5  /* EQ1_B1_GAIN - [15:11] */
#define CS47L24_EQ1_B2_GAIN_MASK                 0x07C0  /* EQ1_B2_GAIN - [10:6] */
#define CS47L24_EQ1_B2_GAIN_SHIFT                     6  /* EQ1_B2_GAIN - [10:6] */
#define CS47L24_EQ1_B2_GAIN_WIDTH                     5  /* EQ1_B2_GAIN - [10:6] */
#define CS47L24_EQ1_B3_GAIN_MASK                 0x003E  /* EQ1_B3_GAIN - [5:1] */
#define CS47L24_EQ1_B3_GAIN_SHIFT                     1  /* EQ1_B3_GAIN - [5:1] */
#define CS47L24_EQ1_B3_GAIN_WIDTH                     5  /* EQ1_B3_GAIN - [5:1] */
#define CS47L24_EQ1_ENA                          0x0001  /* EQ1_ENA */
#define CS47L24_EQ1_ENA_MASK                     0x0001  /* EQ1_ENA */
#define CS47L24_EQ1_ENA_SHIFT                         0  /* EQ1_ENA */
#define CS47L24_EQ1_ENA_WIDTH                         1  /* EQ1_ENA */

/*
 * R3601 (0xE11) - EQ1_2
 */
#define CS47L24_EQ1_B4_GAIN_MASK                 0xF800  /* EQ1_B4_GAIN - [15:11] */
#define CS47L24_EQ1_B4_GAIN_SHIFT                    11  /* EQ1_B4_GAIN - [15:11] */
#define CS47L24_EQ1_B4_GAIN_WIDTH                     5  /* EQ1_B4_GAIN - [15:11] */
#define CS47L24_EQ1_B5_GAIN_MASK                 0x07C0  /* EQ1_B5_GAIN - [10:6] */
#define CS47L24_EQ1_B5_GAIN_SHIFT                     6  /* EQ1_B5_GAIN - [10:6] */
#define CS47L24_EQ1_B5_GAIN_WIDTH                     5  /* EQ1_B5_GAIN - [10:6] */
#define CS47L24_EQ1_B1_MODE                      0x0001  /* EQ1_B1_MODE */
#define CS47L24_EQ1_B1_MODE_MASK                 0x0001  /* EQ1_B1_MODE */
#define CS47L24_EQ1_B1_MODE_SHIFT                     0  /* EQ1_B1_MODE */
#define CS47L24_EQ1_B1_MODE_WIDTH                     1  /* EQ1_B1_MODE */

/*
 * R3602 (0xE12) - EQ1_3
 */
#define CS47L24_EQ1_B1_A_MASK                    0xFFFF  /* EQ1_B1_A - [15:0] */
#define CS47L24_EQ1_B1_A_SHIFT                        0  /* EQ1_B1_A - [15:0] */
#define CS47L24_EQ1_B1_A_WIDTH                       16  /* EQ1_B1_A - [15:0] */

/*
 * R3603 (0xE13) - EQ1_4
 */
#define CS47L24_EQ1_B1_B_MASK                    0xFFFF  /* EQ1_B1_B - [15:0] */
#define CS47L24_EQ1_B1_B_SHIFT                        0  /* EQ1_B1_B - [15:0] */
#define CS47L24_EQ1_B1_B_WIDTH                       16  /* EQ1_B1_B - [15:0] */

/*
 * R3604 (0xE14) - EQ1_5
 */
#define CS47L24_EQ1_B1_PG_MASK                   0xFFFF  /* EQ1_B1_PG - [15:0] */
#define CS47L24_EQ1_B1_PG_SHIFT                       0  /* EQ1_B1_PG - [15:0] */
#define CS47L24_EQ1_B1_PG_WIDTH                      16  /* EQ1_B1_PG - [15:0] */

/*
 * R3605 (0xE15) - EQ1_6
 */
#define CS47L24_EQ1_B2_A_MASK                    0xFFFF  /* EQ1_B2_A - [15:0] */
#define CS47L24_EQ1_B2_A_SHIFT                        0  /* EQ1_B2_A - [15:0] */
#define CS47L24_EQ1_B2_A_WIDTH                       16  /* EQ1_B2_A - [15:0] */

/*
 * R3606 (0xE16) - EQ1_7
 */
#define CS47L24_EQ1_B2_B_MASK                    0xFFFF  /* EQ1_B2_B - [15:0] */
#define CS47L24_EQ1_B2_B_SHIFT                        0  /* EQ1_B2_B - [15:0] */
#define CS47L24_EQ1_B2_B_WIDTH                       16  /* EQ1_B2_B - [15:0] */

/*
 * R3607 (0xE17) - EQ1_8
 */
#define CS47L24_EQ1_B2_C_MASK                    0xFFFF  /* EQ1_B2_C - [15:0] */
#define CS47L24_EQ1_B2_C_SHIFT                        0  /* EQ1_B2_C - [15:0] */
#define CS47L24_EQ1_B2_C_WIDTH                       16  /* EQ1_B2_C - [15:0] */

/*
 * R3608 (0xE18) - EQ1_9
 */
#define CS47L24_EQ1_B2_PG_MASK                   0xFFFF  /* EQ1_B2_PG - [15:0] */
#define CS47L24_EQ1_B2_PG_SHIFT                       0  /* EQ1_B2_PG - [15:0] */
#define CS47L24_EQ1_B2_PG_WIDTH                      16  /* EQ1_B2_PG - [15:0] */

/*
 * R3609 (0xE19) - EQ1_10
 */
#define CS47L24_EQ1_B3_A_MASK                    0xFFFF  /* EQ1_B3_A - [15:0] */
#define CS47L24_EQ1_B3_A_SHIFT                        0  /* EQ1_B3_A - [15:0] */
#define CS47L24_EQ1_B3_A_WIDTH                       16  /* EQ1_B3_A - [15:0] */

/*
 * R3610 (0xE1A) - EQ1_11
 */
#define CS47L24_EQ1_B3_B_MASK                    0xFFFF  /* EQ1_B3_B - [15:0] */
#define CS47L24_EQ1_B3_B_SHIFT                        0  /* EQ1_B3_B - [15:0] */
#define CS47L24_EQ1_B3_B_WIDTH                       16  /* EQ1_B3_B - [15:0] */

/*
 * R3611 (0xE1B) - EQ1_12
 */
#define CS47L24_EQ1_B3_C_MASK                    0xFFFF  /* EQ1_B3_C - [15:0] */
#define CS47L24_EQ1_B3_C_SHIFT                        0  /* EQ1_B3_C - [15:0] */
#define CS47L24_EQ1_B3_C_WIDTH                       16  /* EQ1_B3_C - [15:0] */

/*
 * R3612 (0xE1C) - EQ1_13
 */
#define CS47L24_EQ1_B3_PG_MASK                   0xFFFF  /* EQ1_B3_PG - [15:0] */
#define CS47L24_EQ1_B3_PG_SHIFT                       0  /* EQ1_B3_PG - [15:0] */
#define CS47L24_EQ1_B3_PG_WIDTH                      16  /* EQ1_B3_PG - [15:0] */

/*
 * R3613 (0xE1D) - EQ1_14
 */
#define CS47L24_EQ1_B4_A_MASK                    0xFFFF  /* EQ1_B4_A - [15:0] */
#define CS47L24_EQ1_B4_A_SHIFT                        0  /* EQ1_B4_A - [15:0] */
#define CS47L24_EQ1_B4_A_WIDTH                       16  /* EQ1_B4_A - [15:0] */

/*
 * R3614 (0xE1E) - EQ1_15
 */
#define CS47L24_EQ1_B4_B_MASK                    0xFFFF  /* EQ1_B4_B - [15:0] */
#define CS47L24_EQ1_B4_B_SHIFT                        0  /* EQ1_B4_B - [15:0] */
#define CS47L24_EQ1_B4_B_WIDTH                       16  /* EQ1_B4_B - [15:0] */

/*
 * R3615 (0xE1F) - EQ1_16
 */
#define CS47L24_EQ1_B4_C_MASK                    0xFFFF  /* EQ1_B4_C - [15:0] */
#define CS47L24_EQ1_B4_C_SHIFT                        0  /* EQ1_B4_C - [15:0] */
#define CS47L24_EQ1_B4_C_WIDTH                       16  /* EQ1_B4_C - [15:0] */

/*
 * R3616 (0xE20) - EQ1_17
 */
#define CS47L24_EQ1_B4_PG_MASK                   0xFFFF  /* EQ1_B4_PG - [15:0] */
#define CS47L24_EQ1_B4_PG_SHIFT                       0  /* EQ1_B4_PG - [15:0] */
#define CS47L24_EQ1_B4_PG_WIDTH                      16  /* EQ1_B4_PG - [15:0] */

/*
 * R3617 (0xE21) - EQ1_18
 */
#define CS47L24_EQ1_B5_A_MASK                    0xFFFF  /* EQ1_B5_A - [15:0] */
#define CS47L24_EQ1_B5_A_SHIFT                        0  /* EQ1_B5_A - [15:0] */
#define CS47L24_EQ1_B5_A_WIDTH                       16  /* EQ1_B5_A - [15:0] */

/*
 * R3618 (0xE22) - EQ1_19
 */
#define CS47L24_EQ1_B5_B_MASK                    0xFFFF  /* EQ1_B5_B - [15:0] */
#define CS47L24_EQ1_B5_B_SHIFT                        0  /* EQ1_B5_B - [15:0] */
#define CS47L24_EQ1_B5_B_WIDTH                       16  /* EQ1_B5_B - [15:0] */

/*
 * R3619 (0xE23) - EQ1_20
 */
#define CS47L24_EQ1_B5_PG_MASK                   0xFFFF  /* EQ1_B5_PG - [15:0] */
#define CS47L24_EQ1_B5_PG_SHIFT                       0  /* EQ1_B5_PG - [15:0] */
#define CS47L24_EQ1_B5_PG_WIDTH                      16  /* EQ1_B5_PG - [15:0] */

/*
 * R3620 (0xE24) - EQ1_21
 */
#define CS47L24_EQ1_B1_C_MASK                    0xFFFF  /* EQ1_B1_C - [15:0] */
#define CS47L24_EQ1_B1_C_SHIFT                        0  /* EQ1_B1_C - [15:0] */
#define CS47L24_EQ1_B1_C_WIDTH                       16  /* EQ1_B1_C - [15:0] */

/*
 * R3622 (0xE26) - EQ2_1
 */
#define CS47L24_EQ2_B1_GAIN_MASK                 0xF800  /* EQ2_B1_GAIN - [15:11] */
#define CS47L24_EQ2_B1_GAIN_SHIFT                    11  /* EQ2_B1_GAIN - [15:11] */
#define CS47L24_EQ2_B1_GAIN_WIDTH                     5  /* EQ2_B1_GAIN - [15:11] */
#define CS47L24_EQ2_B2_GAIN_MASK                 0x07C0  /* EQ2_B2_GAIN - [10:6] */
#define CS47L24_EQ2_B2_GAIN_SHIFT                     6  /* EQ2_B2_GAIN - [10:6] */
#define CS47L24_EQ2_B2_GAIN_WIDTH                     5  /* EQ2_B2_GAIN - [10:6] */
#define CS47L24_EQ2_B3_GAIN_MASK                 0x003E  /* EQ2_B3_GAIN - [5:1] */
#define CS47L24_EQ2_B3_GAIN_SHIFT                     1  /* EQ2_B3_GAIN - [5:1] */
#define CS47L24_EQ2_B3_GAIN_WIDTH                     5  /* EQ2_B3_GAIN - [5:1] */
#define CS47L24_EQ2_ENA                          0x0001  /* EQ2_ENA */
#define CS47L24_EQ2_ENA_MASK                     0x0001  /* EQ2_ENA */
#define CS47L24_EQ2_ENA_SHIFT                         0  /* EQ2_ENA */
#define CS47L24_EQ2_ENA_WIDTH                         1  /* EQ2_ENA */

/*
 * R3623 (0xE27) - EQ2_2
 */
#define CS47L24_EQ2_B4_GAIN_MASK                 0xF800  /* EQ2_B4_GAIN - [15:11] */
#define CS47L24_EQ2_B4_GAIN_SHIFT                    11  /* EQ2_B4_GAIN - [15:11] */
#define CS47L24_EQ2_B4_GAIN_WIDTH                     5  /* EQ2_B4_GAIN - [15:11] */
#define CS47L24_EQ2_B5_GAIN_MASK                 0x07C0  /* EQ2_B5_GAIN - [10:6] */
#define CS47L24_EQ2_B5_GAIN_SHIFT                     6  /* EQ2_B5_GAIN - [10:6] */
#define CS47L24_EQ2_B5_GAIN_WIDTH                     5  /* EQ2_B5_GAIN - [10:6] */
#define CS47L24_EQ2_B1_MODE                      0x0001  /* EQ2_B1_MODE */
#define CS47L24_EQ2_B1_MODE_MASK                 0x0001  /* EQ2_B1_MODE */
#define CS47L24_EQ2_B1_MODE_SHIFT                     0  /* EQ2_B1_MODE */
#define CS47L24_EQ2_B1_MODE_WIDTH                     1  /* EQ2_B1_MODE */

/*
 * R3624 (0xE28) - EQ2_3
 */
#define CS47L24_EQ2_B1_A_MASK                    0xFFFF  /* EQ2_B1_A - [15:0] */
#define CS47L24_EQ2_B1_A_SHIFT                        0  /* EQ2_B1_A - [15:0] */
#define CS47L24_EQ2_B1_A_WIDTH                       16  /* EQ2_B1_A - [15:0] */

/*
 * R3625 (0xE29) - EQ2_4
 */
#define CS47L24_EQ2_B1_B_MASK                    0xFFFF  /* EQ2_B1_B - [15:0] */
#define CS47L24_EQ2_B1_B_SHIFT                        0  /* EQ2_B1_B - [15:0] */
#define CS47L24_EQ2_B1_B_WIDTH                       16  /* EQ2_B1_B - [15:0] */

/*
 * R3626 (0xE2A) - EQ2_5
 */
#define CS47L24_EQ2_B1_PG_MASK                   0xFFFF  /* EQ2_B1_PG - [15:0] */
#define CS47L24_EQ2_B1_PG_SHIFT                       0  /* EQ2_B1_PG - [15:0] */
#define CS47L24_EQ2_B1_PG_WIDTH                      16  /* EQ2_B1_PG - [15:0] */

/*
 * R3627 (0xE2B) - EQ2_6
 */
#define CS47L24_EQ2_B2_A_MASK                    0xFFFF  /* EQ2_B2_A - [15:0] */
#define CS47L24_EQ2_B2_A_SHIFT                        0  /* EQ2_B2_A - [15:0] */
#define CS47L24_EQ2_B2_A_WIDTH                       16  /* EQ2_B2_A - [15:0] */

/*
 * R3628 (0xE2C) - EQ2_7
 */
#define CS47L24_EQ2_B2_B_MASK                    0xFFFF  /* EQ2_B2_B - [15:0] */
#define CS47L24_EQ2_B2_B_SHIFT                        0  /* EQ2_B2_B - [15:0] */
#define CS47L24_EQ2_B2_B_WIDTH                       16  /* EQ2_B2_B - [15:0] */

/*
 * R3629 (0xE2D) - EQ2_8
 */
#define CS47L24_EQ2_B2_C_MASK                    0xFFFF  /* EQ2_B2_C - [15:0] */
#define CS47L24_EQ2_B2_C_SHIFT                        0  /* EQ2_B2_C - [15:0] */
#define CS47L24_EQ2_B2_C_WIDTH                       16  /* EQ2_B2_C - [15:0] */

/*
 * R3630 (0xE2E) - EQ2_9
 */
#define CS47L24_EQ2_B2_PG_MASK                   0xFFFF  /* EQ2_B2_PG - [15:0] */
#define CS47L24_EQ2_B2_PG_SHIFT                       0  /* EQ2_B2_PG - [15:0] */
#define CS47L24_EQ2_B2_PG_WIDTH                      16  /* EQ2_B2_PG - [15:0] */

/*
 * R3631 (0xE2F) - EQ2_10
 */
#define CS47L24_EQ2_B3_A_MASK                    0xFFFF  /* EQ2_B3_A - [15:0] */
#define CS47L24_EQ2_B3_A_SHIFT                        0  /* EQ2_B3_A - [15:0] */
#define CS47L24_EQ2_B3_A_WIDTH                       16  /* EQ2_B3_A - [15:0] */

/*
 * R3632 (0xE30) - EQ2_11
 */
#define CS47L24_EQ2_B3_B_MASK                    0xFFFF  /* EQ2_B3_B - [15:0] */
#define CS47L24_EQ2_B3_B_SHIFT                        0  /* EQ2_B3_B - [15:0] */
#define CS47L24_EQ2_B3_B_WIDTH                       16  /* EQ2_B3_B - [15:0] */

/*
 * R3633 (0xE31) - EQ2_12
 */
#define CS47L24_EQ2_B3_C_MASK                    0xFFFF  /* EQ2_B3_C - [15:0] */
#define CS47L24_EQ2_B3_C_SHIFT                        0  /* EQ2_B3_C - [15:0] */
#define CS47L24_EQ2_B3_C_WIDTH                       16  /* EQ2_B3_C - [15:0] */

/*
 * R3634 (0xE32) - EQ2_13
 */
#define CS47L24_EQ2_B3_PG_MASK                   0xFFFF  /* EQ2_B3_PG - [15:0] */
#define CS47L24_EQ2_B3_PG_SHIFT                       0  /* EQ2_B3_PG - [15:0] */
#define CS47L24_EQ2_B3_PG_WIDTH                      16  /* EQ2_B3_PG - [15:0] */

/*
 * R3635 (0xE33) - EQ2_14
 */
#define CS47L24_EQ2_B4_A_MASK                    0xFFFF  /* EQ2_B4_A - [15:0] */
#define CS47L24_EQ2_B4_A_SHIFT                        0  /* EQ2_B4_A - [15:0] */
#define CS47L24_EQ2_B4_A_WIDTH                       16  /* EQ2_B4_A - [15:0] */

/*
 * R3636 (0xE34) - EQ2_15
 */
#define CS47L24_EQ2_B4_B_MASK                    0xFFFF  /* EQ2_B4_B - [15:0] */
#define CS47L24_EQ2_B4_B_SHIFT                        0  /* EQ2_B4_B - [15:0] */
#define CS47L24_EQ2_B4_B_WIDTH                       16  /* EQ2_B4_B - [15:0] */

/*
 * R3637 (0xE35) - EQ2_16
 */
#define CS47L24_EQ2_B4_C_MASK                    0xFFFF  /* EQ2_B4_C - [15:0] */
#define CS47L24_EQ2_B4_C_SHIFT                        0  /* EQ2_B4_C - [15:0] */
#define CS47L24_EQ2_B4_C_WIDTH                       16  /* EQ2_B4_C - [15:0] */

/*
 * R3638 (0xE36) - EQ2_17
 */
#define CS47L24_EQ2_B4_PG_MASK                   0xFFFF  /* EQ2_B4_PG - [15:0] */
#define CS47L24_EQ2_B4_PG_SHIFT                       0  /* EQ2_B4_PG - [15:0] */
#define CS47L24_EQ2_B4_PG_WIDTH                      16  /* EQ2_B4_PG - [15:0] */

/*
 * R3639 (0xE37) - EQ2_18
 */
#define CS47L24_EQ2_B5_A_MASK                    0xFFFF  /* EQ2_B5_A - [15:0] */
#define CS47L24_EQ2_B5_A_SHIFT                        0  /* EQ2_B5_A - [15:0] */
#define CS47L24_EQ2_B5_A_WIDTH                       16  /* EQ2_B5_A - [15:0] */

/*
 * R3640 (0xE38) - EQ2_19
 */
#define CS47L24_EQ2_B5_B_MASK                    0xFFFF  /* EQ2_B5_B - [15:0] */
#define CS47L24_EQ2_B5_B_SHIFT                        0  /* EQ2_B5_B - [15:0] */
#define CS47L24_EQ2_B5_B_WIDTH                       16  /* EQ2_B5_B - [15:0] */

/*
 * R3641 (0xE39) - EQ2_20
 */
#define CS47L24_EQ2_B5_PG_MASK                   0xFFFF  /* EQ2_B5_PG - [15:0] */
#define CS47L24_EQ2_B5_PG_SHIFT                       0  /* EQ2_B5_PG - [15:0] */
#define CS47L24_EQ2_B5_PG_WIDTH                      16  /* EQ2_B5_PG - [15:0] */

/*
 * R3642 (0xE3A) - EQ2_21
 */
#define CS47L24_EQ2_B1_C_MASK                    0xFFFF  /* EQ2_B1_C - [15:0] */
#define CS47L24_EQ2_B1_C_SHIFT                        0  /* EQ2_B1_C - [15:0] */
#define CS47L24_EQ2_B1_C_WIDTH                       16  /* EQ2_B1_C - [15:0] */

/*
 * R3712 (0xE80) - DRC1 ctrl1
 */
#define CS47L24_DRC1_SIG_DET_RMS_MASK            0xF800  /* DRC1_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC1_SIG_DET_RMS_SHIFT               11  /* DRC1_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC1_SIG_DET_RMS_WIDTH                5  /* DRC1_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC1_SIG_DET_PK_MASK             0x0600  /* DRC1_SIG_DET_PK - [10:9] */
#define CS47L24_DRC1_SIG_DET_PK_SHIFT                 9  /* DRC1_SIG_DET_PK - [10:9] */
#define CS47L24_DRC1_SIG_DET_PK_WIDTH                 2  /* DRC1_SIG_DET_PK - [10:9] */
#define CS47L24_DRC1_NG_ENA                      0x0100  /* DRC1_NG_ENA */
#define CS47L24_DRC1_NG_ENA_MASK                 0x0100  /* DRC1_NG_ENA */
#define CS47L24_DRC1_NG_ENA_SHIFT                     8  /* DRC1_NG_ENA */
#define CS47L24_DRC1_NG_ENA_WIDTH                     1  /* DRC1_NG_ENA */
#define CS47L24_DRC1_SIG_DET_MODE                0x0080  /* DRC1_SIG_DET_MODE */
#define CS47L24_DRC1_SIG_DET_MODE_MASK           0x0080  /* DRC1_SIG_DET_MODE */
#define CS47L24_DRC1_SIG_DET_MODE_SHIFT               7  /* DRC1_SIG_DET_MODE */
#define CS47L24_DRC1_SIG_DET_MODE_WIDTH               1  /* DRC1_SIG_DET_MODE */
#define CS47L24_DRC1_SIG_DET                     0x0040  /* DRC1_SIG_DET */
#define CS47L24_DRC1_SIG_DET_MASK                0x0040  /* DRC1_SIG_DET */
#define CS47L24_DRC1_SIG_DET_SHIFT                    6  /* DRC1_SIG_DET */
#define CS47L24_DRC1_SIG_DET_WIDTH                    1  /* DRC1_SIG_DET */
#define CS47L24_DRC1_KNEE2_OP_ENA                0x0020  /* DRC1_KNEE2_OP_ENA */
#define CS47L24_DRC1_KNEE2_OP_ENA_MASK           0x0020  /* DRC1_KNEE2_OP_ENA */
#define CS47L24_DRC1_KNEE2_OP_ENA_SHIFT               5  /* DRC1_KNEE2_OP_ENA */
#define CS47L24_DRC1_KNEE2_OP_ENA_WIDTH               1  /* DRC1_KNEE2_OP_ENA */
#define CS47L24_DRC1_QR                          0x0010  /* DRC1_QR */
#define CS47L24_DRC1_QR_MASK                     0x0010  /* DRC1_QR */
#define CS47L24_DRC1_QR_SHIFT                         4  /* DRC1_QR */
#define CS47L24_DRC1_QR_WIDTH                         1  /* DRC1_QR */
#define CS47L24_DRC1_ANTICLIP                    0x0008  /* DRC1_ANTICLIP */
#define CS47L24_DRC1_ANTICLIP_MASK               0x0008  /* DRC1_ANTICLIP */
#define CS47L24_DRC1_ANTICLIP_SHIFT                   3  /* DRC1_ANTICLIP */
#define CS47L24_DRC1_ANTICLIP_WIDTH                   1  /* DRC1_ANTICLIP */
#define CS47L24_DRC1L_ENA                        0x0002  /* DRC1L_ENA */
#define CS47L24_DRC1L_ENA_MASK                   0x0002  /* DRC1L_ENA */
#define CS47L24_DRC1L_ENA_SHIFT                       1  /* DRC1L_ENA */
#define CS47L24_DRC1L_ENA_WIDTH                       1  /* DRC1L_ENA */
#define CS47L24_DRC1R_ENA                        0x0001  /* DRC1R_ENA */
#define CS47L24_DRC1R_ENA_MASK                   0x0001  /* DRC1R_ENA */
#define CS47L24_DRC1R_ENA_SHIFT                       0  /* DRC1R_ENA */
#define CS47L24_DRC1R_ENA_WIDTH                       1  /* DRC1R_ENA */

/*
 * R3713 (0xE81) - DRC1 ctrl2
 */
#define CS47L24_DRC1_ATK_MASK                    0x1E00  /* DRC1_ATK - [12:9] */
#define CS47L24_DRC1_ATK_SHIFT                        9  /* DRC1_ATK - [12:9] */
#define CS47L24_DRC1_ATK_WIDTH                        4  /* DRC1_ATK - [12:9] */
#define CS47L24_DRC1_DCY_MASK                    0x01E0  /* DRC1_DCY - [8:5] */
#define CS47L24_DRC1_DCY_SHIFT                        5  /* DRC1_DCY - [8:5] */
#define CS47L24_DRC1_DCY_WIDTH                        4  /* DRC1_DCY - [8:5] */
#define CS47L24_DRC1_MINGAIN_MASK                0x001C  /* DRC1_MINGAIN - [4:2] */
#define CS47L24_DRC1_MINGAIN_SHIFT                    2  /* DRC1_MINGAIN - [4:2] */
#define CS47L24_DRC1_MINGAIN_WIDTH                    3  /* DRC1_MINGAIN - [4:2] */
#define CS47L24_DRC1_MAXGAIN_MASK                0x0003  /* DRC1_MAXGAIN - [1:0] */
#define CS47L24_DRC1_MAXGAIN_SHIFT                    0  /* DRC1_MAXGAIN - [1:0] */
#define CS47L24_DRC1_MAXGAIN_WIDTH                    2  /* DRC1_MAXGAIN - [1:0] */

/*
 * R3714 (0xE82) - DRC1 ctrl3
 */
#define CS47L24_DRC1_NG_MINGAIN_MASK             0xF000  /* DRC1_NG_MINGAIN - [15:12] */
#define CS47L24_DRC1_NG_MINGAIN_SHIFT                12  /* DRC1_NG_MINGAIN - [15:12] */
#define CS47L24_DRC1_NG_MINGAIN_WIDTH                 4  /* DRC1_NG_MINGAIN - [15:12] */
#define CS47L24_DRC1_NG_EXP_MASK                 0x0C00  /* DRC1_NG_EXP - [11:10] */
#define CS47L24_DRC1_NG_EXP_SHIFT                    10  /* DRC1_NG_EXP - [11:10] */
#define CS47L24_DRC1_NG_EXP_WIDTH                     2  /* DRC1_NG_EXP - [11:10] */
#define CS47L24_DRC1_QR_THR_MASK                 0x0300  /* DRC1_QR_THR - [9:8] */
#define CS47L24_DRC1_QR_THR_SHIFT                     8  /* DRC1_QR_THR - [9:8] */
#define CS47L24_DRC1_QR_THR_WIDTH                     2  /* DRC1_QR_THR - [9:8] */
#define CS47L24_DRC1_QR_DCY_MASK                 0x00C0  /* DRC1_QR_DCY - [7:6] */
#define CS47L24_DRC1_QR_DCY_SHIFT                     6  /* DRC1_QR_DCY - [7:6] */
#define CS47L24_DRC1_QR_DCY_WIDTH                     2  /* DRC1_QR_DCY - [7:6] */
#define CS47L24_DRC1_HI_COMP_MASK                0x0038  /* DRC1_HI_COMP - [5:3] */
#define CS47L24_DRC1_HI_COMP_SHIFT                    3  /* DRC1_HI_COMP - [5:3] */
#define CS47L24_DRC1_HI_COMP_WIDTH                    3  /* DRC1_HI_COMP - [5:3] */
#define CS47L24_DRC1_LO_COMP_MASK                0x0007  /* DRC1_LO_COMP - [2:0] */
#define CS47L24_DRC1_LO_COMP_SHIFT                    0  /* DRC1_LO_COMP - [2:0] */
#define CS47L24_DRC1_LO_COMP_WIDTH                    3  /* DRC1_LO_COMP - [2:0] */

/*
 * R3715 (0xE83) - DRC1 ctrl4
 */
#define CS47L24_DRC1_KNEE_IP_MASK                0x07E0  /* DRC1_KNEE_IP - [10:5] */
#define CS47L24_DRC1_KNEE_IP_SHIFT                    5  /* DRC1_KNEE_IP - [10:5] */
#define CS47L24_DRC1_KNEE_IP_WIDTH                    6  /* DRC1_KNEE_IP - [10:5] */
#define CS47L24_DRC1_KNEE_OP_MASK                0x001F  /* DRC1_KNEE_OP - [4:0] */
#define CS47L24_DRC1_KNEE_OP_SHIFT                    0  /* DRC1_KNEE_OP - [4:0] */
#define CS47L24_DRC1_KNEE_OP_WIDTH                    5  /* DRC1_KNEE_OP - [4:0] */

/*
 * R3716 (0xE84) - DRC1 ctrl5
 */
#define CS47L24_DRC1_KNEE2_IP_MASK               0x03E0  /* DRC1_KNEE2_IP - [9:5] */
#define CS47L24_DRC1_KNEE2_IP_SHIFT                   5  /* DRC1_KNEE2_IP - [9:5] */
#define CS47L24_DRC1_KNEE2_IP_WIDTH                   5  /* DRC1_KNEE2_IP - [9:5] */
#define CS47L24_DRC1_KNEE2_OP_MASK               0x001F  /* DRC1_KNEE2_OP - [4:0] */
#define CS47L24_DRC1_KNEE2_OP_SHIFT                   0  /* DRC1_KNEE2_OP - [4:0] */
#define CS47L24_DRC1_KNEE2_OP_WIDTH                   5  /* DRC1_KNEE2_OP - [4:0] */

/*
 * R3721 (0xE89) - DRC2 ctrl1
 */
#define CS47L24_DRC2_SIG_DET_RMS_MASK            0xF800  /* DRC2_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC2_SIG_DET_RMS_SHIFT               11  /* DRC2_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC2_SIG_DET_RMS_WIDTH                5  /* DRC2_SIG_DET_RMS - [15:11] */
#define CS47L24_DRC2_SIG_DET_PK_MASK             0x0600  /* DRC2_SIG_DET_PK - [10:9] */
#define CS47L24_DRC2_SIG_DET_PK_SHIFT                 9  /* DRC2_SIG_DET_PK - [10:9] */
#define CS47L24_DRC2_SIG_DET_PK_WIDTH                 2  /* DRC2_SIG_DET_PK - [10:9] */
#define CS47L24_DRC2_NG_ENA                      0x0100  /* DRC2_NG_ENA */
#define CS47L24_DRC2_NG_ENA_MASK                 0x0100  /* DRC2_NG_ENA */
#define CS47L24_DRC2_NG_ENA_SHIFT                     8  /* DRC2_NG_ENA */
#define CS47L24_DRC2_NG_ENA_WIDTH                     1  /* DRC2_NG_ENA */
#define CS47L24_DRC2_SIG_DET_MODE                0x0080  /* DRC2_SIG_DET_MODE */
#define CS47L24_DRC2_SIG_DET_MODE_MASK           0x0080  /* DRC2_SIG_DET_MODE */
#define CS47L24_DRC2_SIG_DET_MODE_SHIFT               7  /* DRC2_SIG_DET_MODE */
#define CS47L24_DRC2_SIG_DET_MODE_WIDTH               1  /* DRC2_SIG_DET_MODE */
#define CS47L24_DRC2_SIG_DET                     0x0040  /* DRC2_SIG_DET */
#define CS47L24_DRC2_SIG_DET_MASK                0x0040  /* DRC2_SIG_DET */
#define CS47L24_DRC2_SIG_DET_SHIFT                    6  /* DRC2_SIG_DET */
#define CS47L24_DRC2_SIG_DET_WIDTH                    1  /* DRC2_SIG_DET */
#define CS47L24_DRC2_KNEE2_OP_ENA                0x0020  /* DRC2_KNEE2_OP_ENA */
#define CS47L24_DRC2_KNEE2_OP_ENA_MASK           0x0020  /* DRC2_KNEE2_OP_ENA */
#define CS47L24_DRC2_KNEE2_OP_ENA_SHIFT               5  /* DRC2_KNEE2_OP_ENA */
#define CS47L24_DRC2_KNEE2_OP_ENA_WIDTH               1  /* DRC2_KNEE2_OP_ENA */
#define CS47L24_DRC2_QR                          0x0010  /* DRC2_QR */
#define CS47L24_DRC2_QR_MASK                     0x0010  /* DRC2_QR */
#define CS47L24_DRC2_QR_SHIFT                         4  /* DRC2_QR */
#define CS47L24_DRC2_QR_WIDTH                         1  /* DRC2_QR */
#define CS47L24_DRC2_ANTICLIP                    0x0008  /* DRC2_ANTICLIP */
#define CS47L24_DRC2_ANTICLIP_MASK               0x0008  /* DRC2_ANTICLIP */
#define CS47L24_DRC2_ANTICLIP_SHIFT                   3  /* DRC2_ANTICLIP */
#define CS47L24_DRC2_ANTICLIP_WIDTH                   1  /* DRC2_ANTICLIP */
#define CS47L24_DRC2L_ENA                        0x0002  /* DRC2L_ENA */
#define CS47L24_DRC2L_ENA_MASK                   0x0002  /* DRC2L_ENA */
#define CS47L24_DRC2L_ENA_SHIFT                       1  /* DRC2L_ENA */
#define CS47L24_DRC2L_ENA_WIDTH                       1  /* DRC2L_ENA */
#define CS47L24_DRC2R_ENA                        0x0001  /* DRC2R_ENA */
#define CS47L24_DRC2R_ENA_MASK                   0x0001  /* DRC2R_ENA */
#define CS47L24_DRC2R_ENA_SHIFT                       0  /* DRC2R_ENA */
#define CS47L24_DRC2R_ENA_WIDTH                       1  /* DRC2R_ENA */

/*
 * R3722 (0xE8A) - DRC2 ctrl2
 */
#define CS47L24_DRC2_ATK_MASK                    0x1E00  /* DRC2_ATK - [12:9] */
#define CS47L24_DRC2_ATK_SHIFT                        9  /* DRC2_ATK - [12:9] */
#define CS47L24_DRC2_ATK_WIDTH                        4  /* DRC2_ATK - [12:9] */
#define CS47L24_DRC2_DCY_MASK                    0x01E0  /* DRC2_DCY - [8:5] */
#define CS47L24_DRC2_DCY_SHIFT                        5  /* DRC2_DCY - [8:5] */
#define CS47L24_DRC2_DCY_WIDTH                        4  /* DRC2_DCY - [8:5] */
#define CS47L24_DRC2_MINGAIN_MASK                0x001C  /* DRC2_MINGAIN - [4:2] */
#define CS47L24_DRC2_MINGAIN_SHIFT                    2  /* DRC2_MINGAIN - [4:2] */
#define CS47L24_DRC2_MINGAIN_WIDTH                    3  /* DRC2_MINGAIN - [4:2] */
#define CS47L24_DRC2_MAXGAIN_MASK                0x0003  /* DRC2_MAXGAIN - [1:0] */
#define CS47L24_DRC2_MAXGAIN_SHIFT                    0  /* DRC2_MAXGAIN - [1:0] */
#define CS47L24_DRC2_MAXGAIN_WIDTH                    2  /* DRC2_MAXGAIN - [1:0] */

/*
 * R3723 (0xE8B) - DRC2 ctrl3
 */
#define CS47L24_DRC2_NG_MINGAIN_MASK             0xF000  /* DRC2_NG_MINGAIN - [15:12] */
#define CS47L24_DRC2_NG_MINGAIN_SHIFT                12  /* DRC2_NG_MINGAIN - [15:12] */
#define CS47L24_DRC2_NG_MINGAIN_WIDTH                 4  /* DRC2_NG_MINGAIN - [15:12] */
#define CS47L24_DRC2_NG_EXP_MASK                 0x0C00  /* DRC2_NG_EXP - [11:10] */
#define CS47L24_DRC2_NG_EXP_SHIFT                    10  /* DRC2_NG_EXP - [11:10] */
#define CS47L24_DRC2_NG_EXP_WIDTH                     2  /* DRC2_NG_EXP - [11:10] */
#define CS47L24_DRC2_QR_THR_MASK                 0x0300  /* DRC2_QR_THR - [9:8] */
#define CS47L24_DRC2_QR_THR_SHIFT                     8  /* DRC2_QR_THR - [9:8] */
#define CS47L24_DRC2_QR_THR_WIDTH                     2  /* DRC2_QR_THR - [9:8] */
#define CS47L24_DRC2_QR_DCY_MASK                 0x00C0  /* DRC2_QR_DCY - [7:6] */
#define CS47L24_DRC2_QR_DCY_SHIFT                     6  /* DRC2_QR_DCY - [7:6] */
#define CS47L24_DRC2_QR_DCY_WIDTH                     2  /* DRC2_QR_DCY - [7:6] */
#define CS47L24_DRC2_HI_COMP_MASK                0x0038  /* DRC2_HI_COMP - [5:3] */
#define CS47L24_DRC2_HI_COMP_SHIFT                    3  /* DRC2_HI_COMP - [5:3] */
#define CS47L24_DRC2_HI_COMP_WIDTH                    3  /* DRC2_HI_COMP - [5:3] */
#define CS47L24_DRC2_LO_COMP_MASK                0x0007  /* DRC2_LO_COMP - [2:0] */
#define CS47L24_DRC2_LO_COMP_SHIFT                    0  /* DRC2_LO_COMP - [2:0] */
#define CS47L24_DRC2_LO_COMP_WIDTH                    3  /* DRC2_LO_COMP - [2:0] */

/*
 * R3724 (0xE8C) - DRC2 ctrl4
 */
#define CS47L24_DRC2_KNEE_IP_MASK                0x07E0  /* DRC2_KNEE_IP - [10:5] */
#define CS47L24_DRC2_KNEE_IP_SHIFT                    5  /* DRC2_KNEE_IP - [10:5] */
#define CS47L24_DRC2_KNEE_IP_WIDTH                    6  /* DRC2_KNEE_IP - [10:5] */
#define CS47L24_DRC2_KNEE_OP_MASK                0x001F  /* DRC2_KNEE_OP - [4:0] */
#define CS47L24_DRC2_KNEE_OP_SHIFT                    0  /* DRC2_KNEE_OP - [4:0] */
#define CS47L24_DRC2_KNEE_OP_WIDTH                    5  /* DRC2_KNEE_OP - [4:0] */

/*
 * R3725 (0xE8D) - DRC2 ctrl5
 */
#define CS47L24_DRC2_KNEE2_IP_MASK               0x03E0  /* DRC2_KNEE2_IP - [9:5] */
#define CS47L24_DRC2_KNEE2_IP_SHIFT                   5  /* DRC2_KNEE2_IP - [9:5] */
#define CS47L24_DRC2_KNEE2_IP_WIDTH                   5  /* DRC2_KNEE2_IP - [9:5] */
#define CS47L24_DRC2_KNEE2_OP_MASK               0x001F  /* DRC2_KNEE2_OP - [4:0] */
#define CS47L24_DRC2_KNEE2_OP_SHIFT                   0  /* DRC2_KNEE2_OP - [4:0] */
#define CS47L24_DRC2_KNEE2_OP_WIDTH                   5  /* DRC2_KNEE2_OP - [4:0] */

/*
 * R3776 (0xEC0) - HPLPF1_1
 */
#define CS47L24_LHPF1_MODE                       0x0002  /* LHPF1_MODE */
#define CS47L24_LHPF1_MODE_MASK                  0x0002  /* LHPF1_MODE */
#define CS47L24_LHPF1_MODE_SHIFT                      1  /* LHPF1_MODE */
#define CS47L24_LHPF1_MODE_WIDTH                      1  /* LHPF1_MODE */
#define CS47L24_LHPF1_ENA                        0x0001  /* LHPF1_ENA */
#define CS47L24_LHPF1_ENA_MASK                   0x0001  /* LHPF1_ENA */
#define CS47L24_LHPF1_ENA_SHIFT                       0  /* LHPF1_ENA */
#define CS47L24_LHPF1_ENA_WIDTH                       1  /* LHPF1_ENA */

/*
 * R3777 (0xEC1) - HPLPF1_2
 */
#define CS47L24_LHPF1_COEFF_MASK                 0xFFFF  /* LHPF1_COEFF - [15:0] */
#define CS47L24_LHPF1_COEFF_SHIFT                     0  /* LHPF1_COEFF - [15:0] */
#define CS47L24_LHPF1_COEFF_WIDTH                    16  /* LHPF1_COEFF - [15:0] */

/*
 * R3780 (0xEC4) - HPLPF2_1
 */
#define CS47L24_LHPF2_MODE                       0x0002  /* LHPF2_MODE */
#define CS47L24_LHPF2_MODE_MASK                  0x0002  /* LHPF2_MODE */
#define CS47L24_LHPF2_MODE_SHIFT                      1  /* LHPF2_MODE */
#define CS47L24_LHPF2_MODE_WIDTH                      1  /* LHPF2_MODE */
#define CS47L24_LHPF2_ENA                        0x0001  /* LHPF2_ENA */
#define CS47L24_LHPF2_ENA_MASK                   0x0001  /* LHPF2_ENA */
#define CS47L24_LHPF2_ENA_SHIFT                       0  /* LHPF2_ENA */
#define CS47L24_LHPF2_ENA_WIDTH                       1  /* LHPF2_ENA */

/*
 * R3781 (0xEC5) - HPLPF2_2
 */
#define CS47L24_LHPF2_COEFF_MASK                 0xFFFF  /* LHPF2_COEFF - [15:0] */
#define CS47L24_LHPF2_COEFF_SHIFT                     0  /* LHPF2_COEFF - [15:0] */
#define CS47L24_LHPF2_COEFF_WIDTH                    16  /* LHPF2_COEFF - [15:0] */

/*
 * R3784 (0xEC8) - HPLPF3_1
 */
#define CS47L24_LHPF3_MODE                       0x0002  /* LHPF3_MODE */
#define CS47L24_LHPF3_MODE_MASK                  0x0002  /* LHPF3_MODE */
#define CS47L24_LHPF3_MODE_SHIFT                      1  /* LHPF3_MODE */
#define CS47L24_LHPF3_MODE_WIDTH                      1  /* LHPF3_MODE */
#define CS47L24_LHPF3_ENA                        0x0001  /* LHPF3_ENA */
#define CS47L24_LHPF3_ENA_MASK                   0x0001  /* LHPF3_ENA */
#define CS47L24_LHPF3_ENA_SHIFT                       0  /* LHPF3_ENA */
#define CS47L24_LHPF3_ENA_WIDTH                       1  /* LHPF3_ENA */

/*
 * R3785 (0xEC9) - HPLPF3_2
 */
#define CS47L24_LHPF3_COEFF_MASK                 0xFFFF  /* LHPF3_COEFF - [15:0] */
#define CS47L24_LHPF3_COEFF_SHIFT                     0  /* LHPF3_COEFF - [15:0] */
#define CS47L24_LHPF3_COEFF_WIDTH                    16  /* LHPF3_COEFF - [15:0] */

/*
 * R3788 (0xECC) - HPLPF4_1
 */
#define CS47L24_LHPF4_MODE                       0x0002  /* LHPF4_MODE */
#define CS47L24_LHPF4_MODE_MASK                  0x0002  /* LHPF4_MODE */
#define CS47L24_LHPF4_MODE_SHIFT                      1  /* LHPF4_MODE */
#define CS47L24_LHPF4_MODE_WIDTH                      1  /* LHPF4_MODE */
#define CS47L24_LHPF4_ENA                        0x0001  /* LHPF4_ENA */
#define CS47L24_LHPF4_ENA_MASK                   0x0001  /* LHPF4_ENA */
#define CS47L24_LHPF4_ENA_SHIFT                       0  /* LHPF4_ENA */
#define CS47L24_LHPF4_ENA_WIDTH                       1  /* LHPF4_ENA */

/*
 * R3789 (0xECD) - HPLPF4_2
 */
#define CS47L24_LHPF4_COEFF_MASK                 0xFFFF  /* LHPF4_COEFF - [15:0] */
#define CS47L24_LHPF4_COEFF_SHIFT                     0  /* LHPF4_COEFF - [15:0] */
#define CS47L24_LHPF4_COEFF_WIDTH                    16  /* LHPF4_COEFF - [15:0] */

/*
 * R3808 (0xEE0) - ASRC_ENABLE
 */
#define CS47L24_ASRC2L_ENA                       0x0008  /* ASRC2L_ENA */
#define CS47L24_ASRC2L_ENA_MASK                  0x0008  /* ASRC2L_ENA */
#define CS47L24_ASRC2L_ENA_SHIFT                      3  /* ASRC2L_ENA */
#define CS47L24_ASRC2L_ENA_WIDTH                      1  /* ASRC2L_ENA */
#define CS47L24_ASRC2R_ENA                       0x0004  /* ASRC2R_ENA */
#define CS47L24_ASRC2R_ENA_MASK                  0x0004  /* ASRC2R_ENA */
#define CS47L24_ASRC2R_ENA_SHIFT                      2  /* ASRC2R_ENA */
#define CS47L24_ASRC2R_ENA_WIDTH                      1  /* ASRC2R_ENA */
#define CS47L24_ASRC1L_ENA                       0x0002  /* ASRC1L_ENA */
#define CS47L24_ASRC1L_ENA_MASK                  0x0002  /* ASRC1L_ENA */
#define CS47L24_ASRC1L_ENA_SHIFT                      1  /* ASRC1L_ENA */
#define CS47L24_ASRC1L_ENA_WIDTH                      1  /* ASRC1L_ENA */
#define CS47L24_ASRC1R_ENA                       0x0001  /* ASRC1R_ENA */
#define CS47L24_ASRC1R_ENA_MASK                  0x0001  /* ASRC1R_ENA */
#define CS47L24_ASRC1R_ENA_SHIFT                      0  /* ASRC1R_ENA */
#define CS47L24_ASRC1R_ENA_WIDTH                      1  /* ASRC1R_ENA */

/*
 * R3809 (0xEE1) - ASRC_ENABLE_STATUS
 */
#define CS47L24_ASRC2L_ENA_STS                   0x0008  /* ASRC2L_ENA_STS */
#define CS47L24_ASRC2L_ENA_STS_MASK              0x0008  /* ASRC2L_ENA_STS */
#define CS47L24_ASRC2L_ENA_STS_SHIFT                  3  /* ASRC2L_ENA_STS */
#define CS47L24_ASRC2L_ENA_STS_WIDTH                  1  /* ASRC2L_ENA_STS */
#define CS47L24_ASRC2R_ENA_STS                   0x0004  /* ASRC2R_ENA_STS */
#define CS47L24_ASRC2R_ENA_STS_MASK              0x0004  /* ASRC2R_ENA_STS */
#define CS47L24_ASRC2R_ENA_STS_SHIFT                  2  /* ASRC2R_ENA_STS */
#define CS47L24_ASRC2R_ENA_STS_WIDTH                  1  /* ASRC2R_ENA_STS */
#define CS47L24_ASRC1L_ENA_STS                   0x0002  /* ASRC1L_ENA_STS */
#define CS47L24_ASRC1L_ENA_STS_MASK              0x0002  /* ASRC1L_ENA_STS */
#define CS47L24_ASRC1L_ENA_STS_SHIFT                  1  /* ASRC1L_ENA_STS */
#define CS47L24_ASRC1L_ENA_STS_WIDTH                  1  /* ASRC1L_ENA_STS */
#define CS47L24_ASRC1R_ENA_STS                   0x0001  /* ASRC1R_ENA_STS */
#define CS47L24_ASRC1R_ENA_STS_MASK              0x0001  /* ASRC1R_ENA_STS */
#define CS47L24_ASRC1R_ENA_STS_SHIFT                  0  /* ASRC1R_ENA_STS */
#define CS47L24_ASRC1R_ENA_STS_WIDTH                  1  /* ASRC1R_ENA_STS */

/*
 * R3810 (0xEE2) - ASRC_RATE1
 */
#define CS47L24_ASRC_RATE1_MASK                  0x7800  /* ASRC_RATE1 - [14:11] */
#define CS47L24_ASRC_RATE1_SHIFT                     11  /* ASRC_RATE1 - [14:11] */
#define CS47L24_ASRC_RATE1_WIDTH                      4  /* ASRC_RATE1 - [14:11] */

/*
 * R3811 (0xEE3) - ASRC_RATE2
 */
#define CS47L24_ASRC_RATE2_MASK                  0x7800  /* ASRC_RATE2 - [14:11] */
#define CS47L24_ASRC_RATE2_SHIFT                     11  /* ASRC_RATE2 - [14:11] */
#define CS47L24_ASRC_RATE2_WIDTH                      4  /* ASRC_RATE2 - [14:11] */

/*
 * R3824 (0xEF0) - ISRC 1 CTRL 1
 */
#define CS47L24_ISRC1_FSH_MASK                   0x7800  /* ISRC1_FSH - [14:11] */
#define CS47L24_ISRC1_FSH_SHIFT                      11  /* ISRC1_FSH - [14:11] */
#define CS47L24_ISRC1_FSH_WIDTH                       4  /* ISRC1_FSH - [14:11] */

/*
 * R3825 (0xEF1) - ISRC 1 CTRL 2
 */
#define CS47L24_ISRC1_FSL_MASK                   0x7800  /* ISRC1_FSL - [14:11] */
#define CS47L24_ISRC1_FSL_SHIFT                      11  /* ISRC1_FSL - [14:11] */
#define CS47L24_ISRC1_FSL_WIDTH                       4  /* ISRC1_FSL - [14:11] */

/*
 * R3826 (0xEF2) - ISRC 1 CTRL 3
 */
#define CS47L24_ISRC1_INT1_ENA                   0x8000  /* ISRC1_INT0_ENA */
#define CS47L24_ISRC1_INT1_ENA_MASK              0x8000  /* ISRC1_INT0_ENA */
#define CS47L24_ISRC1_INT1_ENA_SHIFT                 15  /* ISRC1_INT0_ENA */
#define CS47L24_ISRC1_INT1_ENA_WIDTH                  1  /* ISRC1_INT0_ENA */
#define CS47L24_ISRC1_INT2_ENA                   0x4000  /* ISRC1_INT1_ENA */
#define CS47L24_ISRC1_INT2_ENA_MASK              0x4000  /* ISRC1_INT1_ENA */
#define CS47L24_ISRC1_INT2_ENA_SHIFT                 14  /* ISRC1_INT1_ENA */
#define CS47L24_ISRC1_INT2_ENA_WIDTH                  1  /* ISRC1_INT1_ENA */
#define CS47L24_ISRC1_INT3_ENA                   0x2000  /* ISRC1_INT2_ENA */
#define CS47L24_ISRC1_INT3_ENA_MASK              0x2000  /* ISRC1_INT2_ENA */
#define CS47L24_ISRC1_INT3_ENA_SHIFT                 13  /* ISRC1_INT2_ENA */
#define CS47L24_ISRC1_INT3_ENA_WIDTH                  1  /* ISRC1_INT2_ENA */
#define CS47L24_ISRC1_INT4_ENA                   0x1000  /* ISRC1_INT3_ENA */
#define CS47L24_ISRC1_INT4_ENA_MASK              0x1000  /* ISRC1_INT3_ENA */
#define CS47L24_ISRC1_INT4_ENA_SHIFT                 12  /* ISRC1_INT3_ENA */
#define CS47L24_ISRC1_INT4_ENA_WIDTH                  1  /* ISRC1_INT3_ENA */
#define CS47L24_ISRC1_DEC1_ENA                   0x0200  /* ISRC1_DEC0_ENA */
#define CS47L24_ISRC1_DEC1_ENA_MASK              0x0200  /* ISRC1_DEC0_ENA */
#define CS47L24_ISRC1_DEC1_ENA_SHIFT                  9  /* ISRC1_DEC0_ENA */
#define CS47L24_ISRC1_DEC1_ENA_WIDTH                  1  /* ISRC1_DEC0_ENA */
#define CS47L24_ISRC1_DEC2_ENA                   0x0100  /* ISRC1_DEC1_ENA */
#define CS47L24_ISRC1_DEC2_ENA_MASK              0x0100  /* ISRC1_DEC1_ENA */
#define CS47L24_ISRC1_DEC2_ENA_SHIFT                  8  /* ISRC1_DEC1_ENA */
#define CS47L24_ISRC1_DEC2_ENA_WIDTH                  1  /* ISRC1_DEC1_ENA */
#define CS47L24_ISRC1_DEC3_ENA                   0x0080  /* ISRC1_DEC2_ENA */
#define CS47L24_ISRC1_DEC3_ENA_MASK              0x0080  /* ISRC1_DEC2_ENA */
#define CS47L24_ISRC1_DEC3_ENA_SHIFT                  7  /* ISRC1_DEC2_ENA */
#define CS47L24_ISRC1_DEC3_ENA_WIDTH                  1  /* ISRC1_DEC2_ENA */
#define CS47L24_ISRC1_DEC4_ENA                   0x0040  /* ISRC1_DEC3_ENA */
#define CS47L24_ISRC1_DEC4_ENA_MASK              0x0040  /* ISRC1_DEC3_ENA */
#define CS47L24_ISRC1_DEC4_ENA_SHIFT                  6  /* ISRC1_DEC3_ENA */
#define CS47L24_ISRC1_DEC4_ENA_WIDTH                  1  /* ISRC1_DEC3_ENA */
#define CS47L24_ISRC1_NOTCH_ENA                  0x0001  /* ISRC1_NOTCH_ENA */
#define CS47L24_ISRC1_NOTCH_ENA_MASK             0x0001  /* ISRC1_NOTCH_ENA */
#define CS47L24_ISRC1_NOTCH_ENA_SHIFT                 0  /* ISRC1_NOTCH_ENA */
#define CS47L24_ISRC1_NOTCH_ENA_WIDTH                 1  /* ISRC1_NOTCH_ENA */

/*
 * R3827 (0xEF3) - ISRC 2 CTRL 1
 */
#define CS47L24_ISRC2_FSH_MASK                   0x7800  /* ISRC2_FSH - [14:11] */
#define CS47L24_ISRC2_FSH_SHIFT                      11  /* ISRC2_FSH - [14:11] */
#define CS47L24_ISRC2_FSH_WIDTH                       4  /* ISRC2_FSH - [14:11] */

/*
 * R3828 (0xEF4) - ISRC 2 CTRL 2
 */
#define CS47L24_ISRC2_FSL_MASK                   0x7800  /* ISRC2_FSL - [14:11] */
#define CS47L24_ISRC2_FSL_SHIFT                      11  /* ISRC2_FSL - [14:11] */
#define CS47L24_ISRC2_FSL_WIDTH                       4  /* ISRC2_FSL - [14:11] */

/*
 * R3829 (0xEF5) - ISRC 2 CTRL 3
 */
#define CS47L24_ISRC2_INT1_ENA                   0x8000  /* ISRC2_INT0_ENA */
#define CS47L24_ISRC2_INT1_ENA_MASK              0x8000  /* ISRC2_INT0_ENA */
#define CS47L24_ISRC2_INT1_ENA_SHIFT                 15  /* ISRC2_INT0_ENA */
#define CS47L24_ISRC2_INT1_ENA_WIDTH                  1  /* ISRC2_INT0_ENA */
#define CS47L24_ISRC2_INT2_ENA                   0x4000  /* ISRC2_INT1_ENA */
#define CS47L24_ISRC2_INT2_ENA_MASK              0x4000  /* ISRC2_INT1_ENA */
#define CS47L24_ISRC2_INT2_ENA_SHIFT                 14  /* ISRC2_INT1_ENA */
#define CS47L24_ISRC2_INT2_ENA_WIDTH                  1  /* ISRC2_INT1_ENA */
#define CS47L24_ISRC2_INT3_ENA                   0x2000  /* ISRC2_INT2_ENA */
#define CS47L24_ISRC2_INT3_ENA_MASK              0x2000  /* ISRC2_INT2_ENA */
#define CS47L24_ISRC2_INT3_ENA_SHIFT                 13  /* ISRC2_INT2_ENA */
#define CS47L24_ISRC2_INT3_ENA_WIDTH                  1  /* ISRC2_INT2_ENA */
#define CS47L24_ISRC2_INT4_ENA                   0x1000  /* ISRC2_INT3_ENA */
#define CS47L24_ISRC2_INT4_ENA_MASK              0x1000  /* ISRC2_INT3_ENA */
#define CS47L24_ISRC2_INT4_ENA_SHIFT                 12  /* ISRC2_INT3_ENA */
#define CS47L24_ISRC2_INT4_ENA_WIDTH                  1  /* ISRC2_INT3_ENA */
#define CS47L24_ISRC2_DEC1_ENA                   0x0200  /* ISRC2_DEC0_ENA */
#define CS47L24_ISRC2_DEC1_ENA_MASK              0x0200  /* ISRC2_DEC0_ENA */
#define CS47L24_ISRC2_DEC1_ENA_SHIFT                  9  /* ISRC2_DEC0_ENA */
#define CS47L24_ISRC2_DEC1_ENA_WIDTH                  1  /* ISRC2_DEC0_ENA */
#define CS47L24_ISRC2_DEC2_ENA                   0x0100  /* ISRC2_DEC1_ENA */
#define CS47L24_ISRC2_DEC2_ENA_MASK              0x0100  /* ISRC2_DEC1_ENA */
#define CS47L24_ISRC2_DEC2_ENA_SHIFT                  8  /* ISRC2_DEC1_ENA */
#define CS47L24_ISRC2_DEC2_ENA_WIDTH                  1  /* ISRC2_DEC1_ENA */
#define CS47L24_ISRC2_DEC3_ENA                   0x0080  /* ISRC2_DEC2_ENA */
#define CS47L24_ISRC2_DEC3_ENA_MASK              0x0080  /* ISRC2_DEC2_ENA */
#define CS47L24_ISRC2_DEC3_ENA_SHIFT                  7  /* ISRC2_DEC2_ENA */
#define CS47L24_ISRC2_DEC3_ENA_WIDTH                  1  /* ISRC2_DEC2_ENA */
#define CS47L24_ISRC2_DEC4_ENA                   0x0040  /* ISRC2_DEC3_ENA */
#define CS47L24_ISRC2_DEC4_ENA_MASK              0x0040  /* ISRC2_DEC3_ENA */
#define CS47L24_ISRC2_DEC4_ENA_SHIFT                  6  /* ISRC2_DEC3_ENA */
#define CS47L24_ISRC2_DEC4_ENA_WIDTH                  1  /* ISRC2_DEC3_ENA */
#define CS47L24_ISRC2_NOTCH_ENA                  0x0001  /* ISRC2_NOTCH_ENA */
#define CS47L24_ISRC2_NOTCH_ENA_MASK             0x0001  /* ISRC2_NOTCH_ENA */
#define CS47L24_ISRC2_NOTCH_ENA_SHIFT                 0  /* ISRC2_NOTCH_ENA */
#define CS47L24_ISRC2_NOTCH_ENA_WIDTH                 1  /* ISRC2_NOTCH_ENA */

/*
 * R3830 (0xEF6) - ISRC 3 CTRL 1
 */
#define CS47L24_ISRC3_FSH_MASK                   0x7800  /* ISRC3_FSH - [14:11] */
#define CS47L24_ISRC3_FSH_SHIFT                      11  /* ISRC3_FSH - [14:11] */
#define CS47L24_ISRC3_FSH_WIDTH                       4  /* ISRC3_FSH - [14:11] */

/*
 * R3831 (0xEF7) - ISRC 3 CTRL 2
 */
#define CS47L24_ISRC3_FSL_MASK                   0x7800  /* ISRC3_FSL - [14:11] */
#define CS47L24_ISRC3_FSL_SHIFT                      11  /* ISRC3_FSL - [14:11] */
#define CS47L24_ISRC3_FSL_WIDTH                       4  /* ISRC3_FSL - [14:11] */

/*
 * R3832 (0xEF8) - ISRC 3 CTRL 3
 */
#define CS47L24_ISRC3_INT1_ENA                   0x8000  /* ISRC3_INT0_ENA */
#define CS47L24_ISRC3_INT1_ENA_MASK              0x8000  /* ISRC3_INT0_ENA */
#define CS47L24_ISRC3_INT1_ENA_SHIFT                 15  /* ISRC3_INT0_ENA */
#define CS47L24_ISRC3_INT1_ENA_WIDTH                  1  /* ISRC3_INT0_ENA */
#define CS47L24_ISRC3_INT2_ENA                   0x4000  /* ISRC3_INT1_ENA */
#define CS47L24_ISRC3_INT2_ENA_MASK              0x4000  /* ISRC3_INT1_ENA */
#define CS47L24_ISRC3_INT2_ENA_SHIFT                 14  /* ISRC3_INT1_ENA */
#define CS47L24_ISRC3_INT2_ENA_WIDTH                  1  /* ISRC3_INT1_ENA */
#define CS47L24_ISRC3_INT3_ENA                   0x2000  /* ISRC3_INT2_ENA */
#define CS47L24_ISRC3_INT3_ENA_MASK              0x2000  /* ISRC3_INT2_ENA */
#define CS47L24_ISRC3_INT3_ENA_SHIFT                 13  /* ISRC3_INT2_ENA */
#define CS47L24_ISRC3_INT3_ENA_WIDTH                  1  /* ISRC3_INT2_ENA */
#define CS47L24_ISRC3_INT4_ENA                   0x1000  /* ISRC3_INT3_ENA */
#define CS47L24_ISRC3_INT4_ENA_MASK              0x1000  /* ISRC3_INT3_ENA */
#define CS47L24_ISRC3_INT4_ENA_SHIFT                 12  /* ISRC3_INT3_ENA */
#define CS47L24_ISRC3_INT4_ENA_WIDTH                  1  /* ISRC3_INT3_ENA */
#define CS47L24_ISRC3_DEC1_ENA                   0x0200  /* ISRC3_DEC0_ENA */
#define CS47L24_ISRC3_DEC1_ENA_MASK              0x0200  /* ISRC3_DEC0_ENA */
#define CS47L24_ISRC3_DEC1_ENA_SHIFT                  9  /* ISRC3_DEC0_ENA */
#define CS47L24_ISRC3_DEC1_ENA_WIDTH                  1  /* ISRC3_DEC0_ENA */
#define CS47L24_ISRC3_DEC2_ENA                   0x0100  /* ISRC3_DEC1_ENA */
#define CS47L24_ISRC3_DEC2_ENA_MASK              0x0100  /* ISRC3_DEC1_ENA */
#define CS47L24_ISRC3_DEC2_ENA_SHIFT                  8  /* ISRC3_DEC1_ENA */
#define CS47L24_ISRC3_DEC2_ENA_WIDTH                  1  /* ISRC3_DEC1_ENA */
#define CS47L24_ISRC3_DEC3_ENA                   0x0080  /* ISRC3_DEC2_ENA */
#define CS47L24_ISRC3_DEC3_ENA_MASK              0x0080  /* ISRC3_DEC2_ENA */
#define CS47L24_ISRC3_DEC3_ENA_SHIFT                  7  /* ISRC3_DEC2_ENA */
#define CS47L24_ISRC3_DEC3_ENA_WIDTH                  1  /* ISRC3_DEC2_ENA */
#define CS47L24_ISRC3_DEC4_ENA                   0x0040  /* ISRC3_DEC3_ENA */
#define CS47L24_ISRC3_DEC4_ENA_MASK              0x0040  /* ISRC3_DEC3_ENA */
#define CS47L24_ISRC3_DEC4_ENA_SHIFT                  6  /* ISRC3_DEC3_ENA */
#define CS47L24_ISRC3_DEC4_ENA_WIDTH                  1  /* ISRC3_DEC3_ENA */
#define CS47L24_ISRC3_NOTCH_ENA                  0x0001  /* ISRC3_NOTCH_ENA */
#define CS47L24_ISRC3_NOTCH_ENA_MASK             0x0001  /* ISRC3_NOTCH_ENA */
#define CS47L24_ISRC3_NOTCH_ENA_SHIFT                 0  /* ISRC3_NOTCH_ENA */
#define CS47L24_ISRC3_NOTCH_ENA_WIDTH                 1  /* ISRC3_NOTCH_ENA */


/*
 * R4608/R4864 (0x1200/0x1300) - DSP Control
 */
#define CS47L24_DSP_RATE_MASK                    0x7800  /* DSP_RATE - [14:11] */
#define CS47L24_DSP_RATE_SHIFT                       11  /* DSP_RATE - [14:11] */
#define CS47L24_DSP_RATE_WIDTH                        4  /* DSP_RATE - [14:11] */
#define CS47L24_DSP_MEM_ENA                      0x0010  /* DSP_MEM_ENA */
#define CS47L24_DSP_MEM_ENA_MASK                 0x0010  /* DSP_MEM_ENA */
#define CS47L24_DSP_MEM_ENA_SHIFT                     4  /* DSP_MEM_ENA */
#define CS47L24_DSP_MEM_ENA_WIDTH                     1  /* DSP_MEM_ENA */
#define CS47L24_DSP_DBG_CLK_ENA                  0x0008  /* DSP_DBG_CLK_ENA */
#define CS47L24_DSP_DBG_CLK_ENA_MASK             0x0008  /* DSP_DBG_CLK_ENA */
#define CS47L24_DSP_DBG_CLK_ENA_SHIFT                 3  /* DSP_DBG_CLK_ENA */
#define CS47L24_DSP_DBG_CLK_ENA_WIDTH                 1  /* DSP_DBG_CLK_ENA */
#define CS47L24_DSP_SYS_ENA                      0x0004  /* DSP_SYS_ENA */
#define CS47L24_DSP_SYS_ENA_MASK                 0x0004  /* DSP_SYS_ENA */
#define CS47L24_DSP_SYS_ENA_SHIFT                     2  /* DSP_SYS_ENA */
#define CS47L24_DSP_SYS_ENA_WIDTH                     1  /* DSP_SYS_ENA */
#define CS47L24_DSP_CORE_ENA                     0x0002  /* DSP_CORE_ENA */
#define CS47L24_DSP_CORE_ENA_MASK                0x0002  /* DSP_CORE_ENA */
#define CS47L24_DSP_CORE_ENA_SHIFT                    1  /* DSP_CORE_ENA */
#define CS47L24_DSP_CORE_ENA_WIDTH                    1  /* DSP_CORE_ENA */
#define CS47L24_DSP_START                        0x0001  /* DSP_START */
#define CS47L24_DSP_START_MASK                   0x0001  /* DSP_START */
#define CS47L24_DSP_START_SHIFT                       0  /* DSP_START */
#define CS47L24_DSP_START_WIDTH                       1  /* DSP_START */

/*
 * R4609/R4865 (0x1201/0x1301) - DSP clocking
 */
#define CS47L24_DSP_CLK_SEL_MASK                 0x0007  /* CLK_SEL_ENA */
#define CS47L24_DSP_CLK_SEL_SHIFT                     0  /* CLK_SEL_ENA */
#define CS47L24_DSP_CLK_SEL_WIDTH                     3  /* CLK_SEL_ENA */

/*
 * R4612/R4868 (0x1204/0x1304) - DSP Status 1
 */
#define CS47L24_DSP_RAM_RDY                      0x0001
#define CS47L24_DSP_RAM_RDY_MASK                 0x0001
#define CS47L24_DSP_RAM_RDY_SHIFT                     0
#define CS47L24_DSP_RAM_RDY_WIDTH                     1

/*
 * R4614/R4870 (0x1206/0x1306) - DSP Status 3
 */
#define CS47L24_DSP_CLK_SEL_STS_MASK             0x000E
#define CS47L24_DSP_CLK_SEL_STS_SHIFT                 1
#define CS47L24_DSP_CLK_SEL_STS_WIDTH                 3


/*
 * R465/R4912 (0x1230/0x1330) DSP WDMA Config 1
 */
#define CS47L24_DSP_WDMA_BUFFER_LENGTH_MASK      0x3FFF
#define CS47L24_DSP_WDMA_BUFFER_LENGTH_SHIFT          0
#define CS47L24_DSP_WDMA_BUFFER_LENGTH_WIDTH         14

/*
 * R4657/R4913 (0x1231/0x1331) DSP WDMA Config 2
 */
#define CS47L24_DSP_WDMA_CHANNEL_ENA             0x00FF
#define CS47L24_DSP_WDMA_CHANNEL_ENA_MASK        0x00FF
#define CS47L24_DSP_WDMA_CHANNEL_ENA_SHIFT            0
#define CS47L24_DSP_WDMA_CHANNEL_ENA_WIDTH            8

/*
 * R4657/R4916 (0x1234/0x1334) DSP RDMA Config 1
 */
#define CS47L24_DSP_RDMA_CHANNEL_ENA             0x003F
#define CS47L24_DSP_RDMA_CHANNEL_ENA_MASK        0x003F
#define CS47L24_DSP_RDMA_CHANNEL_ENA_SHIFT            0
#define CS47L24_DSP_RDMA_CHANNEL_ENA_WIDTH            6

/*
 * DSP2 and DSP 3 PM/XM/YM/ZM memory address
 */
#define CS47L24_DSP2_PM_START                    0x200000
#define CS47L24_DSP2_ZM_START                    0x280000
#define CS47L24_DSP2_XM_START                    0x290000
#define CS47L24_DSP2_YM_START                    0x2A8000
#define CS47L24_DSP3_PM_START                    0x300000
#define CS47L24_DSP3_ZM_START                    0x380000
#define CS47L24_DSP3_XM_START                    0x390000
#define CS47L24_DSP3_YM_START                    0x3A8000

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
