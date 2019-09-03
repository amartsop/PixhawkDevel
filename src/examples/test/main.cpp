#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_config.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/uORB.h>

#include "main.h"
#include "structs_main.h"
#include "signal_processing.hpp"
#include "measurements.hpp"
#include "att_control.hpp"
#include "setpoint.hpp"
#include "state.hpp"
#include "failsafe.hpp"



/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static bool close_actuators = false;
static int deamon_task;				/**< Handle of deamon task / thread */



/* Main Thread */
int test_thread_main(int argc, char *argv[])
{
	
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n,
		with variable arguments */
	warnx("[test control] started");

	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));

	struct battery_status_s bat_status;
	memset(&bat_status, 0, sizeof(bat_status));

	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	
	struct estimator_status_s estimator_stat;
	memset(&estimator_stat, 0, sizeof(estimator_stat));

	struct sensor_combined_s sensor_comb;
	memset(&sensor_comb, 0, sizeof(sensor_comb));

	struct input_rc_s input_rc_signal;
	memset(&input_rc_signal, 0, sizeof(input_rc_signal));

	struct measurements_vec_s measurements_vec;
	memset(&measurements_vec, 0, sizeof(measurements_vec));

	struct setpoint_vec_s setpoint_vec;
	memset(&setpoint_vec, 0, sizeof(setpoint_vec));

	struct state_vec_s state_vec;
	memset(&state_vec, 0, sizeof(state_vec));

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = -1.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, 
		&actuators);

	/* subscribe to topics. */
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int estimator_status_sub = orb_subscribe(ORB_ID(estimator_status));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int input_rc_sub = orb_subscribe(ORB_ID(input_rc));

	/* Setup of loop */
	struct pollfd fds[3] = {};
	fds[0].fd = param_sub;
	fds[0].events = POLLIN;
	fds[1].fd = att_sub;
	fds[1].events = POLLIN;
	fds[2].fd = sensor_combined_sub;
	fds[2].events = POLLIN;
	
	/* Read failsafe switch */
	orb_copy(ORB_ID(input_rc), input_rc_sub, 
		&input_rc_signal);	
	Failsafe::failsafe_check(&close_actuators, &input_rc_signal);

	/* Main loop */
	while (!thread_should_exit) {	

		
		int ret = poll(fds, 2, 200);

		if (ret < 0) {

			warnx("poll error");

		} else if (ret == 0) {
		
		
		} else {

			/* only update parameters if they changed */
			if (fds[0].revents & POLLIN) {

				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);
			}

			/* only run controller if attitude changed */
			if (fds[1].revents & POLLIN) {

				// Copy topic's measurements
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				//Battery voltage
				orb_copy(ORB_ID(battery_status), battery_status_sub, &bat_status);

				/* get the system status and the flight mode we're in */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				/*get the system estimation status */
				orb_copy(ORB_ID(estimator_status), estimator_status_sub, 
					&estimator_stat);

				/*get the system sensor combined */
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub,
					&sensor_comb);

				/* Subscribe to rc input */
				orb_copy(ORB_ID(input_rc), input_rc_sub, 
					&input_rc_signal);
				
				// Failsafe check
				Failsafe::failsafe_check(&close_actuators, &input_rc_signal);

				// Read measurements
				Measurements::measurements_est(&measurements_vec, &att,
					&estimator_stat);

				// Read reference
				Setpoint::set_reference(&setpoint_vec, &input_rc_signal);
				
				//CHANGE STATE VECTOR FOR ZDOT!!!!!!!!!!!!!!!!
				// Construct state vector
				State::state_est(&state_vec, &measurements_vec, &estimator_stat); 
				
 				// Control law
				AttControl::control_law(&setpoint_vec, &state_vec, 
					&actuators, &bat_status, close_actuators);
				
				printf("%8.4f\n", (double) actuators.control[0]);

				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {

					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub,
						&actuators);

					if (verbose) {
						warnx("published");
					}
				}
			}
		}
		
	}

	printf("[ex_test] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = -1.0f;
	}

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
	
	fflush(stdout);
	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: ex_test {start|stop|status}\n\n");
}


int ex_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_test already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_test",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 20,
			2048,
			test_thread_main,
			(argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_test is running\n");

		} else {
			printf("\tex_test not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}



// Example code debugging

/*
	//print float32
	printf("%8.4f\n", (double) att.yawspeed);

	//Print boolean			
	printf("%d\n", global_sp_updated);

 */