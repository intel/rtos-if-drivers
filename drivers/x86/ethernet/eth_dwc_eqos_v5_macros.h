/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_ETHERNET_ETH_DWC_EQOS_MACROS_H_
#define DRIVERS_ETHERNET_ETH_DWC_EQOS_MACROS_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ETH_DWC_EQOS_TBS
#define TX_DESC_TAIL_PTR(c, q) ({                                              \
	volatile void *pdesc_tail = (c)->tbs_enabled[q] ?                      \
				    (void *)&(c)->enh_tx_desc[q][0] :          \
				    (void *)&(c)->tx_desc[q][0];               \
	pdesc_tail = (volatile void *)(POINTER_TO_UINT(pdesc_tail) +           \
		     ((c)->tdesc_ring_wr_ptr[q] * ((c)->tbs_enabled[q] ?       \
		     sizeof(enh_desc_t) : sizeof(struct eth_tx_desc))));       \
	pdesc_tail;                                                            \
})

#define TX_DESC_INDEX(ctxt, q, desc) ((ctxt)->tbs_enabled[q] ?                 \
				      ((enh_desc_t *)desc -                    \
				       (ctxt)->enh_tx_desc[q]) :               \
				      ((struct eth_tx_desc *)desc -            \
				       (ctxt)->tx_desc[q]))
#else
#define TX_DESC_TAIL_PTR(c, q) ({                                              \
	volatile void *pdesc_tail = &(c)->tx_desc[q][0];                       \
	pdesc_tail = (volatile void *)(POINTER_TO_UINT(pdesc_tail) +           \
		     ((c)->tdesc_ring_wr_ptr[q] *                              \
		     sizeof(struct eth_tx_desc)));                             \
	pdesc_tail;                                                            \
})

#define TX_DESC_INDEX(ctxt, q, desc) (desc - (ctxt)->tx_desc[q])
#endif /* CONFIG_ETH_DWC_EQOS_TBS */

#define RX_DESC_TAIL_PTR(c, q) ({                                              \
	volatile void *pdesc_tail = &(c)->rx_desc[q][0];                       \
	pdesc_tail = (volatile void *)(POINTER_TO_UINT(pdesc_tail) +           \
		     ((((c)->rdesc_ring_wr_ptr[q] - 1 +                        \
		     CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE) %                      \
		     CONFIG_ETH_DWC_EQOS_DMA_RING_SIZE) *                      \
		     sizeof(struct eth_rx_desc)));                             \
	pdesc_tail;                                                            \
})

#define RX_DESC_INDEX(ctxt, q, desc) (desc - (ctxt)->rx_desc[q])

#define ETH_TX_IRQ_QUEUE(q)                                                    \
	static void eth_tx_irq_queue##q(struct k_work *item)                   \
	{                                                                      \
		struct eth_runtime *ctxt = CONTAINER_OF(item,                  \
							struct eth_runtime,    \
							tx_irq_work[q]);       \
\
		eth_tx_clean(ctxt, q, eth_completed_tx_desc_num(ctxt, q));     \
	}                                                                      \
	static void eth_tx_irq_queue##q(struct k_work *item)

#define ETH_RX_IRQ_QUEUE(q)                                                    \
	static void eth_rx_irq_queue##q(struct k_work *item)                   \
	{                                                                      \
		struct eth_runtime *ctxt = CONTAINER_OF(item,                  \
							struct eth_runtime,    \
							rx_irq_work[q]);       \
\
		eth_rx_data(ctxt, q, eth_completed_rx_desc_num(ctxt, q));      \
	}                                                                      \
	static void eth_rx_irq_queue##q(struct k_work *item)

#define INIT_ETH_TX_IRQ_WORK(q)                                                \
	k_work_init(&context->tx_irq_work[q], eth_tx_irq_queue##q)

#define INIT_ETH_RX_IRQ_WORK(q)                                                \
	k_work_init_delayable(&context->rx_irq_work[q], eth_rx_irq_queue##q)

#define ETH_DWC_EQOS_TX_ISR(q)                                                 \
	static void eth_dwc_eqos_tx##q##_isr(struct device *port)              \
	{                                                                      \
		eth_tx_irq(port, q);                                           \
	}                                                                      \
	static void eth_dwc_eqos_tx##q##_isr(struct device *port)

#define ETH_DWC_EQOS_RX_ISR(q)                                                 \
	static void eth_dwc_eqos_rx##q##_isr(struct device *port)              \
	{                                                                      \
		eth_rx_irq(port, q);                                           \
	}                                                                      \
	static void eth_dwc_eqos_rx##q##_isr(struct device *port)


static inline int setup_pcie(const struct device *port, int prio,
			     void (*routine)(const void *parameter),
			     const void *parameter)
{
	struct eth_runtime *data = port->data;
	unsigned int irq;

	irq = pcie_alloc_irq(data->bdf);
	if (irq == PCIE_CONF_INTR_IRQ_NONE) {
		return -ENOENT;
	}

	irq_connect_dynamic(irq, prio, routine, parameter, 0);
	pcie_irq_enable(data->bdf, irq);

	return 0;
}

#if defined(CONFIG_X86_64)
#define SETUP_TXQ_DIRECT_IRQ(idx, q)					\
	do {								\
		setup_pcie(ETH_DWC_EQOS_##idx##_TX##q##_IRQ,		\
			   CONFIG_ETH_DWC_EQOS_##idx##_IRQ_PRI,		\
			   eth_dwc_eqos_tx##q##_isr,			\
			   DEVICE_GET(eth_dwc_eqos_##idx)		\
			);						\
	} while (0)

#else
#define SETUP_TXQ_DIRECT_IRQ(idx, q)                                           \
	do {                                                                   \
		IRQ_CONNECT(ETH_DWC_EQOS_##idx##_TX##q##_IRQ,                  \
			    CONFIG_ETH_DWC_EQOS_##idx##_IRQ_PRI,               \
			    eth_dwc_eqos_tx##q##_isr,                          \
			    DEVICE_GET(eth_dwc_eqos_##idx), 0);                \
		irq_enable(ETH_DWC_EQOS_##idx##_TX##q##_IRQ);                  \
	} while (0)
#endif

#if defined(CONFIG_X86_64)
#define SETUP_RXQ_DIRECT_IRQ(idx, q)					\
	do {								\
		setup_pcie(ETH_DWC_EQOS_##idx##_TX##q##_IRQ,		\
			   CONFIG_ETH_DWC_EQOS_##idx##_IRQ_PRI,		\
			   eth_dwc_eqos_tx##q##_isr,			\
			   DEVICE_GET(eth_dwc_eqos_##idx)		\
			);						\
	} while (0)
#else
#define SETUP_RXQ_DIRECT_IRQ(idx, q)                                           \
	do {                                                                   \
		IRQ_CONNECT(ETH_DWC_EQOS_##idx##_RX##q##_IRQ,                  \
			    CONFIG_ETH_DWC_EQOS_##idx##_IRQ_PRI,               \
			    eth_dwc_eqos_rx##q##_isr,                          \
			    DEVICE_GET(eth_dwc_eqos_##idx), 0);                \
		irq_enable(ETH_DWC_EQOS_##idx##_RX##q##_IRQ);                  \
	} while (0)
#endif

#define SETUP_TXQ_SHARED_IRQ(q)                                                \
	shared_irq_isr_register(shared_irq_dev,                                \
				(isr_t)eth_dwc_eqos_tx##q##_isr,               \
				port)

#define SETUP_RXQ_SHARED_IRQ(q)                                                \
	shared_irq_isr_register(shared_irq_dev,                                \
				(isr_t)eth_dwc_eqos_rx##q##_isr,               \
				port)

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_ETHERNET_ETH_DWC_EQOS_MACROS_H_ */
