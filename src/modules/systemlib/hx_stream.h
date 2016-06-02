/**
 * @file hx_stream.h
 *
 * A simple serial line framing protocol based on HDLC  基于HDLC（高级数据链路控制）的一种简单串口框架协议
 * with 32-bit CRC protection. 基于32位crc保护
 */

#ifndef _SYSTEMLIB_HX_STREAM_H
#define _SYSTEMLIB_HX_STREAM_H

#include <sys/types.h>

#include "perf_counter.h"

struct hx_stream;
typedef struct hx_stream *hx_stream_t;

#define HX_STREAM_MAX_FRAME	64  //最大帧大小

typedef void (* hx_stream_rx_callback)(void *arg, const void *data, size_t length);  //指针函数

__BEGIN_DECLS

/**
 * Allocate a new hx_stream object. 申请一个新的hx对象
 *
 * @param fd		The file handle over which the protocol will  文件句柄
 *			communicate, or -1 if the protocol will use
 *			hx_stream_start/hx_stream_send_next.
 * @param callback	Called when a frame is received.  //回调函数
 * @param callback_arg	Passed to the callback.
 * @return		A handle to the stream, or NULL if memory could
 *			not be allocated.
 */
__EXPORT extern hx_stream_t	hx_stream_init(int fd,
		hx_stream_rx_callback callback,
		void *arg);

/**
 * Free a hx_stream object.  释放一个hx_stream对象
 *
 * @param stream	A handle returned from hx_stream_init.
 */
__EXPORT extern void		hx_stream_free(hx_stream_t stream);

/**
 * Set performance counters for the stream.  对一个数据流进行性能计数
 *
 * Any counter may be set to NULL to disable counting that datum.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param tx_frames	Counter for transmitted frames.
 * @param rx_frames	Counter for received frames.
 * @param rx_errors	Counter for short and corrupt received frames.
 */
__EXPORT extern void		hx_stream_set_counters(hx_stream_t stream,
		perf_counter_t tx_frames,
		perf_counter_t rx_frames,
		perf_counter_t rx_errors);

/**
 * Reset a stream.  复位一个数据流
 *
 * Forces the local stream state to idle. 强制本地数据流状态位空闲
 *
 * @param stream	A handle returned from hx_stream_init.
 */
__EXPORT extern void		hx_stream_reset(hx_stream_t stream);

/**
 * Prepare to send a frame.  准备发送一帧数据
 *
 * Use this in conjunction with hx_stream_send_next to
 * set the frame to be transmitted.
 *
 * Use hx_stream_send() to write to the stream fd directly.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param data		Pointer to the data to send.
 * @param count		The number of bytes to send.
 * @return		Zero on success, -errno on error.
 */
__EXPORT extern int		hx_stream_start(hx_stream_t stream,
		const void *data,
		size_t count);

/**
 * Get the next byte to send for a stream.  获取下一个字节数据准备发送到一个数据流
 *
 * This requires that the stream be prepared for sending by
 * calling hx_stream_start first.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @return		The byte to send, or -1 if there is
 *			nothing left to send.
 */
__EXPORT extern int		hx_stream_send_next(hx_stream_t stream);

/**
 * Send a frame.  发送一帧数据
 *
 * This function will block until all frame bytes are sent if
 * the descriptor passed to hx_stream_init is marked blocking,
 * otherwise it will return -1 (but may transmit a
 * runt frame at the same time).
 *
 * @todo Handling of non-blocking streams needs to be better.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param data		Pointer to the data to send.
 * @param count		The number of bytes to send.
 * @return		Zero on success, -errno on error.
 */
__EXPORT extern int		hx_stream_send(hx_stream_t stream,
		const void *data,
		size_t count);

/**
 * Handle a byte from the stream.  从一个数据流中处理一个字节
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param c		The character to process.
 */
__EXPORT extern void		hx_stream_rx(hx_stream_t stream,
		uint8_t c);

__END_DECLS

#endif
