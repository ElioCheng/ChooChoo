#include "io.h"
#include "syscall.h"
#include "name.h"
#include "string.h"

static int io_server_tid = -1;

int Getc(int tid, int channel)
{
	io_request_t request;
	io_reply_t reply;

	if (tid == -1) {
		if (io_server_tid == -1) {
			io_server_tid = WhoIs(IO_SERVER_NAME);
			if (io_server_tid < 0) {
				return -1;
			}
		}
		tid = io_server_tid;
	}

	request.type = IO_REQ_GETC;
	request.channel = channel;

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return -1;
	}

	return reply.result;
}

int TryGetc(int tid, int channel)
{
	io_request_t request;
	io_reply_t reply;

	if (tid == -1) {
		if (io_server_tid == -1) {
			io_server_tid = WhoIs(IO_SERVER_NAME);
			if (io_server_tid < 0) {
				return -1;
			}
		}
		tid = io_server_tid;
	}

	request.type = IO_REQ_TRYGETC;
	request.channel = channel;

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return -1;
	}

	return reply.result;
}

int Putc(int tid, int channel, unsigned char ch)
{
	io_request_t request;
	io_reply_t reply;

	if (tid == -1) {
		if (io_server_tid == -1) {
			io_server_tid = WhoIs(IO_SERVER_NAME);
			if (io_server_tid < 0) {
				return -1;
			}
		}
		tid = io_server_tid;
	}

	request.type = IO_REQ_PUTC;
	request.channel = channel;
	request.putc.ch = ch;

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return -1;
	}

	return reply.result;
}

// Fast path for console by reducing SRR calls. Does not work for Marklin.
int Putn(int tid, int channel, const char *str, size_t len)
{
	io_request_t request;
	io_reply_t reply;

	if (tid == -1) {
		if (io_server_tid == -1) {
			io_server_tid = WhoIs(IO_SERVER_NAME);
			if (io_server_tid < 0) {
				return -1;
			}
		}
		tid = io_server_tid;
	}

	if (len > IO_REQ_PUTN_MAX_LEN) {
		return -1;
	}

	if (channel != IO_CHANNEL_CONSOLE) {
		return -1;
	}

	request.type = IO_REQ_PUTN;
	request.channel = channel;
	request.putn.len = len;
	memcpy((void *)request.putn.str, str, len);

	int result = Send(tid, (const char *)&request, sizeof(request), (char *)&reply, sizeof(reply));

	if (result < 0) {
		return -1;
	}

	return reply.result;
}
