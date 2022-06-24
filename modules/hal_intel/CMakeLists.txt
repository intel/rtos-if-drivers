# Copyright (c) 2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_HAS_INTEL)
  zephyr_library()

  set(INTEL_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/)
  set(INC_DIR ${INTEL_DIR}/)
  set(SRC_DIR ${INTEL_DIR}/drivers/)

  zephyr_include_directories(${INC_DIR}/include)
  zephyr_include_directories(${INC_DIR}/soc/ehl/include)
  zephyr_include_directories(${INTEL_DIR}/service/ia_services/include)

  zephyr_library_sources(${SRC_DIR}/common/hal_driver_common.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_DMA    ${SRC_DIR}/dma/dma_ann_1p0.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_UART    ${SRC_DIR}/uart/dw_uart.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_SPI    ${SRC_DIR}/spi/dw_spi.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_TGPIO    ${SRC_DIR}/tgpio/intel_tgpio.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_PWM    ${SRC_DIR}/pwm/intel_pwm.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_EMMC    ${SRC_DIR}/emmc/emmc_5p1.c)
  zephyr_library_sources_ifdef(CONFIG_INTEL_CAN    ${SRC_DIR}/can/intel_can.c)
  zephyr_library_sources_ifdef(CONFIG_IPC_INTEL    ${SRC_DIR}/ipc/ipc_bsp.c)
endif()
