# Copyright (c) 2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_EMMC_INTEL drivers__disk PRIVATE emmc_intel.c)
