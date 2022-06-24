# Copyright (c) 2021-2022 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/../zephyr/drivers/spi)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/../zephyr/drivers/gpio)
add_subdirectory(drivers)
add_subdirectory(subsys)
