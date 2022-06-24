# Copyright (c) 2021 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_SPI_INTEL drivers__spi PRIVATE spi_intel.c)
