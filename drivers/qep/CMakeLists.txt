# Copyright (c) 2021 Intel Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_USERSPACE   qep_handlers.c)
zephyr_sources_ifdef(CONFIG_QEP_SEDI qep_sedi.c)
