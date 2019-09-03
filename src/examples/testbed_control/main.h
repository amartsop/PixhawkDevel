#pragma once
#include "structs_main.h"

/* Prototypes */

extern "C" __EXPORT int testbed_control_main(int argc, char *argv[]);

/* Initialize subscriptions */
void testbed_control_init(void);

/* Check if topics are updated */
void testbed_control_update_topics(void);

/* Mainloop of daemon. */
int testbed_control_thread_main(int argc, char *argv[]);

/* Print the correct usage. */
static void usage(const char *reason);

