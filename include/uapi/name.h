#ifndef __UAPI_NAME_H__
#define __UAPI_NAME_H__

#define NS_MAX_NAME_LENGTH 32

#define NS_TID 3

typedef enum {
    NS_REGISTER_AS = 1,
    NS_WHO_IS = 2
} ns_request_type_t;

typedef struct {
    ns_request_type_t type;
    char name[NS_MAX_NAME_LENGTH];
} ns_request_t;

typedef struct {
    int result;
} ns_response_t;


/**
 * Get the current name server TID
 * @return The task ID of the name server, or -1 if not set
 */
int GetNameServerTid();

/**
 * Register the calling task with the given name
 * @param name The name to register under
 * @return 0 on success, -1 on error (unable to reach name server)
 */
int RegisterAs(const char *name);

/**
 * Look up the task ID for the given name
 * @param name The name to look up
 * @return The task ID of the registered task, or -1 if not found or unable to reach name server
 */
int WhoIs(const char *name);

#endif /* __UAPI_NAME_H__ */
