#include "name.h"
#include "syscall.h"
#include "string.h"

int GetNameServerTid()
{
	return NS_TID;
}

int RegisterAs(const char *name)
{
	ns_request_t request;
	ns_response_t response;

	request.type = NS_REGISTER_AS;
	strncpy(request.name, name, NS_MAX_NAME_LENGTH - 1);
	request.name[NS_MAX_NAME_LENGTH - 1] = '\0';

	int result = Send(NS_TID, (const char *)&request, sizeof(request), (char *)&response, sizeof(response));

	if (result < 0) {
		return -1;
	}

	return response.result;
}

int WhoIs(const char *name)
{
	ns_request_t request;
	ns_response_t response;

	request.type = NS_WHO_IS;
	strncpy(request.name, name, NS_MAX_NAME_LENGTH - 1);
	request.name[NS_MAX_NAME_LENGTH - 1] = '\0';

	int result = Send(NS_TID, (const char *)&request, sizeof(request), (char *)&response, sizeof(response));

	if (result < 0) {
		return -1;
	}

	return response.result;
}
