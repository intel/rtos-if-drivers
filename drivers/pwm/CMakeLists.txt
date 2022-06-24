# SPDX-License-Identifier: Apache-2.0
target_sources_ifdef(CONFIG_PWM_INTEL drivers__pwm PRIVATE pwm_intel.c)
