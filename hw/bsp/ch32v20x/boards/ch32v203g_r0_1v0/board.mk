MCU_VARIANT = D6

CFLAGS += \
  -DSYSCLK_FREQ_144MHz_HSI=144000000 \
  -DCFG_EXAMPLE_MSC_DUAL_READONLY \

LDFLAGS += \
  -Wl,--defsym=__FLASH_SIZE=32K \
  -Wl,--defsym=__RAM_SIZE=10K \