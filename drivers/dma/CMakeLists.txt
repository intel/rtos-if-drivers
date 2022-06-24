# SPDX-License-Identifier: Apache-2.0

target_sources_ifdef(CONFIG_DMA_INTEL drivers__dma PRIVATE dma_intel.c)
