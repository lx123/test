/**
 * @file hx_stream.c
 *
 * A simple serial line framing protocol based on HDLC
 * with 32-bit CRC protection.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <crc32.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "perf_counter.h"

#include "hx_stream.h"


struct hx_stream {
	/* RX state */  //接受状态
	uint8_t			rx_buf[HX_STREAM_MAX_FRAME + 4];  //接收缓冲区
	unsigned		rx_frame_bytes;
	bool			rx_escaped;
	hx_stream_rx_callback	rx_callback;
	void			*rx_callback_arg;

	/* TX state */  //发送状态
	int			fd;   //句柄
	bool			tx_error;  //发送错误
	const uint8_t		*tx_buf;  //发送缓冲区
	unsigned		tx_resid;
	uint32_t		tx_crc;
	enum {  //发送状态
		TX_IDLE = 0,
		TX_SEND_START,  //开始发送
		TX_SEND_DATA,   //发送数据
		TX_SENT_ESCAPE,  //发送溢出
		TX_SEND_END
	}			tx_state;

	perf_counter_t		pc_tx_frames; //发送帧
	perf_counter_t		pc_rx_frames;  //接收帧
	perf_counter_t		pc_rx_errors;  //发送错误
};

/*
 * Protocol magic numbers, straight out of HDLC.  协议魔法数字
 */
#define FBO	0x7e	/**< Frame Boundary Octet */  //8位帧边界
#define CEO	0x7c	/**< Control Escape Octet */  //8位控制溢出

static void	hx_tx_raw(hx_stream_t stream, uint8_t c);
static void	hx_tx_raw(hx_stream_t stream, uint8_t c);
static int	hx_rx_frame(hx_stream_t stream);

static void
hx_tx_raw(hx_stream_t stream, uint8_t c) //发送一个字节
{
	if (write(stream->fd, &c, 1) != 1) {
		stream->tx_error = true;
	}
}

static int
hx_rx_frame(hx_stream_t stream)  //接收一帧数据
{
	union {
		uint8_t	b[4];
		uint32_t w;
	} u;
	unsigned length = stream->rx_frame_bytes;  //需要接收数据长度

	/* reset the stream */  //复位数据流
	stream->rx_frame_bytes = 0;   //接收帧字节清零
	stream->rx_escaped = false; //接收溢出标志清零

	/* not a real frame - too short */
	if (length < 4) {  //数据长度不足一帧数据
		if (length > 1) { //长度大于1
			perf_count(stream->pc_rx_errors);  //接收错误计数
		}

		return 0;
	}

	length -= 4;  //去掉帧尾crc数值

	/* compute expected CRC */
	u.w = crc32(&stream->rx_buf[0], length);  //crc校验

	/* compare computed and actual CRC */  //与帧尾部的crc校验比较
	for (unsigned i = 0; i < 4; i++) {
		if (u.b[i] != stream->rx_buf[length + i]) {
			perf_count(stream->pc_rx_errors);
			return 0;
		}
	}

	/* frame is good */
	perf_count(stream->pc_rx_frames);  //正常帧计数
	stream->rx_callback(stream->rx_callback_arg, &stream->rx_buf[0], length);
	return 1;
}
//数据流初始化
hx_stream_t
hx_stream_init(int fd,
	       hx_stream_rx_callback callback,
	       void *arg)
{
	hx_stream_t stream;

	stream = (hx_stream_t)malloc(sizeof(struct hx_stream));  //申请内存

	if (stream != NULL) { //判断内存是否申请成功
		memset(stream, 0, sizeof(struct hx_stream));  //内存清空
		stream->fd = fd;
		stream->rx_callback = callback;
		stream->rx_callback_arg = arg;
	}

	return stream;
}
//释放数据流
void
hx_stream_free(hx_stream_t stream)
{
	/* free perf counters (OK if they are NULL) */
	perf_free(stream->pc_tx_frames);
	perf_free(stream->pc_rx_frames);
	perf_free(stream->pc_rx_errors);

	free(stream);  //释放内存
}
//设置性能计数
void
hx_stream_set_counters(hx_stream_t stream,
		       perf_counter_t tx_frames,
		       perf_counter_t rx_frames,
		       perf_counter_t rx_errors)
{
	stream->pc_tx_frames = tx_frames;
	stream->pc_rx_frames = rx_frames;
	stream->pc_rx_errors = rx_errors;
}
//steam数据复位
void
hx_stream_reset(hx_stream_t stream)
{
	stream->rx_frame_bytes = 0;  //接收帧数据
	stream->rx_escaped = false;  //接收溢出

	stream->tx_buf = NULL;  //缓冲区
	stream->tx_resid = 0;
	stream->tx_state = TX_IDLE;  //发送空闲
}
//开始数据流
int
hx_stream_start(hx_stream_t stream,
		const void *data,
		size_t count)
{
	if (count > HX_STREAM_MAX_FRAME) {  //如果大于最大字节数
		return -EINVAL;  //返回错误
	}

	stream->tx_buf = data;   //依次存入结构体
	stream->tx_resid = count;
	stream->tx_state = TX_SEND_START;
	stream->tx_crc = crc32(data, count);
	return OK;
}
//发送下一帧数据流
int
hx_stream_send_next(hx_stream_t stream)
{
	int c;

	/* sort out what we're going to send */  //整理准备发送
	switch (stream->tx_state) {

	case TX_SEND_START:
		stream->tx_state = TX_SEND_DATA;
		return FBO; //返回帧边界

	case TX_SEND_DATA:
		c = *stream->tx_buf;  //取出数据

		switch (c) {
		case FBO:
		case CEO:
			stream->tx_state = TX_SENT_ESCAPE; //发送溢出
			return CEO; //返回溢出标志
		}

		break;

	case TX_SENT_ESCAPE:  //发送溢出
		c = *stream->tx_buf ^ 0x20;
		stream->tx_state = TX_SEND_DATA;
		break;

	case TX_SEND_END:
		stream->tx_state = TX_IDLE;
		return FBO;

	case TX_IDLE:
	default:
		return -1;
	}

	/* if we are here, we have consumed a byte from the buffer */  //如果在这，将消耗缓冲区中的一个字节数据
	stream->tx_resid--;
	stream->tx_buf++;

	/* buffer exhausted */
	if (stream->tx_resid == 0) {
		uint8_t *pcrc = (uint8_t *)&stream->tx_crc;

		/* was the buffer the frame CRC? */
		if (stream->tx_buf == (pcrc + sizeof(stream->tx_crc))) {
			stream->tx_state = TX_SEND_END;

		} else {
			/* no, it was the payload - switch to sending the CRC */
			stream->tx_buf = pcrc;
			stream->tx_resid = sizeof(stream->tx_crc);
		}
	}

	return c;
}
//发送一帧数据
int
hx_stream_send(hx_stream_t stream,
	       const void *data,
	       size_t count)
{
	int result;

	result = hx_stream_start(stream, data, count);

	if (result != OK) {
		return result;
	}

	int c;

	while ((c = hx_stream_send_next(stream)) >= 0) {
		hx_tx_raw(stream, c);
	}

	/* check for transmit error */  //检查发送错误
	if (stream->tx_error) {
		stream->tx_error = false;
		return -EIO;
	}

	perf_count(stream->pc_tx_frames);
	return OK;
}
//数据流接收
void
hx_stream_rx(hx_stream_t stream, uint8_t c)
{
	/* frame end? */  //判断是否到帧底部
	if (c == FBO) {
		hx_rx_frame(stream);
		return;
	}

	/* escaped? */  //判断是否溢出
	if (stream->rx_escaped) {
		stream->rx_escaped = false;
		c ^= 0x20;

	} else if (c == CEO) {
		/* now rx_escaped, ignore the byte */
		stream->rx_escaped = true;
		return;
	}

	/* save for later */
	if (stream->rx_frame_bytes < sizeof(stream->rx_buf)) {
		stream->rx_buf[stream->rx_frame_bytes++] = c;
	}
}
