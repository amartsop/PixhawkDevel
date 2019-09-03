#pragma once

/* Prototypes */

extern "C" __EXPORT int tricopter_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int tricopter_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

