#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_u8g

GLOBAL_INCLUDES := .

ifeq ($(GCC_VERSION_GREATER_THAN_5),1)
$(NAME)_CFLAGS += -Wno-unused-variable
endif

$(NAME)_SOURCES := u8g_arm.c u8g_bitmap.c u8g_circle.c u8g_clip.c u8g_com_api_16gr.c \
                   u8g_com_api.c u8g_com_i2c.c u8g_com_io.c u8g_com_null.c \
                   u8g_cursor.c u8g_delay.c u8g_virtual_screen.c \
                   u8g_ellipse.c u8g_font_data.c u8g_font.c u8g_line.c \
                   u8g_ll_api.c u8g_page.c u8g_pb.c u8g_pb14v1.c u8g_pb16h1.c \
                   u8g_pb16h2.c u8g_pb16v1.c u8g_pb16v2.c u8g_pb32h1.c \
                   u8g_pb8h1.c u8g_pb8h1f.c u8g_pb8h2.c u8g_pb8h8.c u8g_pb8v1.c \
                   u8g_pb8v2.c u8g_pbxh16.c u8g_pbxh24.c u8g_polygon.c u8g_rect.c \
                   u8g_rot.c u8g_scale.c u8g_state.c u8g_u16toa.c u8g_u8toa.c

$(NAME)_SOURCES += u8g_dev_a2_micro_printer.c \
                   u8g_dev_flipdisc_2x7.c u8g_dev_gprof.c u8g_dev_ht1632.c \
                   u8g_dev_ili9325d_320x240.c u8g_dev_ks0108_128x64.c \
                   u8g_dev_lc7981_160x80.c u8g_dev_lc7981_240x128.c \
                   u8g_dev_lc7981_240x64.c u8g_dev_lc7981_320x64.c \
                   u8g_dev_ld7032_60x32.c u8g_dev_null.c u8g_dev_pcd8544_84x48.c \
                   u8g_dev_pcf8812_96x65.c u8g_dev_sbn1661_122x32.c \
                   u8g_dev_ssd1306_128x32.c u8g_dev_ssd1306_128x64.c \
                   u8g_dev_ssd1309_128x64.c u8g_dev_ssd1322_nhd31oled_bw.c \
                   u8g_dev_ssd1322_nhd31oled_gr.c u8g_dev_ssd1325_nhd27oled_bw_new.c \
                   u8g_dev_ssd1325_nhd27oled_bw.c u8g_dev_ssd1325_nhd27oled_gr_new.c \
                   u8g_dev_ssd1325_nhd27oled_gr.c u8g_dev_ssd1327_96x96_gr.c \
                   u8g_dev_ssd1351_128x128.c u8g_dev_st7565_64128n.c \
                   u8g_dev_st7565_dogm128.c u8g_dev_st7565_dogm132.c \
                   u8g_dev_st7565_lm6059.c \
                   u8g_dev_st7565_lm6063.c u8g_dev_st7565_nhd_c12832.c \
                   u8g_dev_st7565_nhd_c12864.c u8g_dev_st7687_c144mvgd.c \
                   u8g_dev_st7920_128x64.c u8g_dev_st7920_192x32.c \
                   u8g_dev_st7920_202x32.c u8g_dev_t6963_128x128.c \
                   u8g_dev_t6963_128x64.c u8g_dev_t6963_240x128.c \
                   u8g_dev_t6963_240x64.c u8g_dev_tls8204_84x48.c \
                   u8g_dev_uc1601_c128032.c u8g_dev_uc1608_240x128.c \
                   u8g_dev_uc1608_240x64.c u8g_dev_uc1610_dogxl160.c \
                   u8g_dev_uc1611_dogm240.c u8g_dev_uc1611_dogxl240.c \
                   u8g_dev_uc1701_dogs102.c u8g_dev_uc1701_mini12864.c

KEEP_LIST := u8g_arm.h u8g.h