#pragma once

/* Prototypes */

extern "C" __EXPORT int ex_test_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int test_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

