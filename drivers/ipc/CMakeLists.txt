# Copyright (c) 2019-2021 Intel Corporation.
#
# SPDX-License-Identifier: Apache-2.0
zephyr_sources_ifdef(CONFIG_USERSPACE ipc_handlers.c)
zephyr_sources_ifdef(CONFIG_IPC_INTEL ipc_intel.c)