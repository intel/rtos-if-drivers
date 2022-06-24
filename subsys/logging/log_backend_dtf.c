/*
 * Copyright (c) 2021-2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log_backend.h>
#include <logging/log_backend_std.h>
#include <logging/log_core.h>
#include <logging/log_msg.h>
#include <logging/log_output.h>
#include <device.h>
#include <assert.h>
#include "driver/sedi_driver_dtf.h"

#define DTF_STRING_BUF_SIZE 1024

static uint8_t level;
static uint8_t buf[DTF_STRING_BUF_SIZE];
static sven_hdr_t sven_hdr;

static uint8_t log_level_zep2svn[] = {
	SVEN_SEVERITY_USER1,
	SVEN_SEVERITY_ERROR,
	SVEN_SEVERITY_WARNING,
	SVEN_SEVERITY_NORMAL,
	SVEN_SEVERITY_NORMAL
};

static int dtf_out(uint8_t *data, size_t length, void *ctx)
{
	__ASSERT(level < sizeof(log_level_zep2svn), "invalid event type\n");
	sven_hdr.et_severity = log_level_zep2svn[level];

	ARG_UNUSED(ctx);

	data[length] = 0;
	sedi_dtf_send(data, length + 1, sven_hdr);
	return length;
}

LOG_OUTPUT_DEFINE(log_output, dtf_out, buf, DTF_STRING_BUF_SIZE - 1);

static void put(const struct log_backend *const backend,
		struct log_msg *msg)
{
	log_msg_get(msg);

	uint32_t flags = LOG_OUTPUT_FLAG_TIMESTAMP
			 | LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;

	level = log_msg_level_get(msg);
	log_output_msg_process(&log_output, msg, flags);
	level = 0;
	log_msg_put(msg);
}

static void log_backend_dtf_init(struct log_backend const *const backend)
{
	sven_hdr.et_type = SVEN_EVENT_TYPE_DEBUG_STRING;
	sven_hdr.et_subtype = SVEN_DEBUGSTR_GENERIC;
	sven_hdr.et_sequence = 0;
	sven_hdr.et_location = 0;
	sven_hdr.et_length = 1;
	sven_hdr.et_ext = 1;
	sven_hdr.et_unit = 0;
	sven_hdr.et_module = 0;
	sedi_dtf_init();
}

static void panic(struct log_backend const *const backend)
{
	log_output_flush(&log_output);
}

static void dtf_dropped(const struct log_backend *const backend, uint32_t cnt)
{
	ARG_UNUSED(backend);

	static char dropping_msg[30];
	int len;

	cnt = MIN(cnt, 9999);
	len = snprintk(dropping_msg, sizeof(dropping_msg),
		       "--- %4d messages dropped ---", cnt);

	dtf_out(dropping_msg, len, NULL);
}

static void sync_string(const struct log_backend *const backend,
			struct log_msg_ids src_level, uint32_t timestamp,
			const char *fmt, va_list ap)
{
	uint32_t flags = LOG_OUTPUT_FLAG_TIMESTAMP
			 | LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	uint32_t key;

	key = irq_lock();
	log_output_string(&log_output, src_level,
			  timestamp, fmt, ap, flags);
	irq_unlock(key);
}

static void sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, uint32_t timestamp,
			 const char *metadata, const uint8_t *data,
			 uint32_t length)
{
	uint32_t flags = LOG_OUTPUT_FLAG_TIMESTAMP
			 | LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	uint32_t key;

	key = irq_lock();
	log_output_hexdump(&log_output, src_level, timestamp,
			   metadata, data, length, flags);
	irq_unlock(key);
}

const struct log_backend_api log_backend_dtf_api = {
	.put = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : put,
	.put_sync_string = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			   sync_string : NULL,
	.put_sync_hexdump = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			    sync_hexdump : NULL,
	.panic = panic,
	.init = log_backend_dtf_init,
	.dropped = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : dtf_dropped,
};

LOG_BACKEND_DEFINE(log_backend_dtf, log_backend_dtf_api, true);
