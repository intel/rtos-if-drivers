# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_GPIO_TIMED_INTEL tgpio_ia_intel.c)
if(CONFIG_GPIO_TIMED_INTEL)
zephyr_library_sources_ifdef(CONFIG_USERSPACE   gpio_timed_handlers.c)
endif()
