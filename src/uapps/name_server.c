#include "name.h"
#include "compiler.h"
#include "klog.h"
#include "syscall.h"
#include "io.h"
#include "string.h"
#include "name_server.h"

typedef struct {
	char name[NS_MAX_NAME_LENGTH];
	int tid;
	int used;
} name_entry_t;

static name_entry_t name_table[MAX_REGISTRATIONS];

static void init_name_table()
{
	for (int i = 0; i < MAX_REGISTRATIONS; i++) {
		name_table[i].used = 0;
		name_table[i].name[0] = '\0';
		name_table[i].tid = -1;
	}
}

static int find_name_entry(const char *name)
{
	for (int i = 0; i < MAX_REGISTRATIONS; i++) {
		if (name_table[i].used && strcmp(name_table[i].name, name) == 0) {
			return i;
		}
	}
	return -1;
}

static int find_free_entry()
{
	for (int i = 0; i < MAX_REGISTRATIONS; i++) {
		if (!name_table[i].used) {
			return i;
		}
	}
	return -1;
}

static int register_name(const char *name, int tid)
{
	int entry_idx = find_name_entry(name);

	if (entry_idx != -1) {
		name_table[entry_idx].tid = tid;
		return 0;
	}

	entry_idx = find_free_entry();
	if (entry_idx == -1) {
		return -1;
	}

	name_table[entry_idx].used = 1;
	strncpy(name_table[entry_idx].name, name, NS_MAX_NAME_LENGTH - 1);
	name_table[entry_idx].name[NS_MAX_NAME_LENGTH - 1] = '\0';
	name_table[entry_idx].tid = tid;

	klog_info("Name server: Registered '%s' -> TID %d", name, tid);

	return 0;
}

static int lookup_name(const char *name)
{
	int entry_idx = find_name_entry(name);
	if (entry_idx != -1) {
		return name_table[entry_idx].tid;
	}
	return -1;
}

void name_task()
{
	int my_tid = MyTid();
	klog_info("Name server started with TID %d", my_tid);

	if (my_tid != NS_TID) {
		Panic("Name server: TID %d is not %d", my_tid, NS_TID);
		UNREACHABLE();
	}

	init_name_table();
	register_name("name_server", my_tid);

	while (1) {
		int sender_tid;
		ns_request_t request;
		ns_response_t response;

		int msglen = Receive(&sender_tid, (char *)&request, sizeof(request));

		if (msglen < 0) {
			klog_error("Name server: Receive error %d", msglen);
			continue;
		}

		switch (request.type) {
		case NS_REGISTER_AS:
			response.result = register_name(request.name, sender_tid);
			if (response.result == 0) {
				// klog_debug("Name server: Registered '%s' -> TID %d", request.name, sender_tid);
			} else {
				klog_error("Name server: Failed to register '%s'", request.name);
			}
			break;

		case NS_WHO_IS:
			response.result = lookup_name(request.name);
			if (response.result >= 0) {
				// klog_debug("Name server: Lookup '%s' -> TID %d", request.name, response.result);
			} else {
				klog_error("Name server: Lookup '%s' -> Not found from tid %d", request.name,
					   sender_tid);
			}
			break;

		default:
			klog_error("Name server: Unknown request type %d, msg_len %d from tid %d", request.type, msglen,
				   sender_tid);
			response.result = -1;
			break;
		}

		Reply(sender_tid, (const char *)&response, sizeof(response));
	}
}
