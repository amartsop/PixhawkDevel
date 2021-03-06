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

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/battery_status.h>
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


struct testbed_control_subscription_data_s *subscription_data = NULL;

/* Main Thread */
int testbed_control_thread_main(int argc, char *argv[])
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
	warnx("[testbed control] started");

	struct measurements_vec_s measurements_vec;
	memset(&measurements_vec, 0, sizeof(measurements_vec));

	struct setpoint_vec_s setpoint_vec;
	memset(&setpoint_vec, 0, sizeof(setpoint_vec));

	struct state_vec_s state_vec;
	memset(&state_vec, 0, sizeof(state_vec));

	struct control_vec_s control_vec;
	memset(&control_vec, 0, sizeof(control_vec));

	struct covariance_mat_s cov_mat;
	memset(&cov_mat, 0, sizeof(cov_mat));

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = -1.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, 
		&actuators);

	/* Subscribe to uORB topics */
	testbed_control_init();

		/* Setup of loop */
	struct pollfd fds[1] = {};
	fds[0].fd = subscription_data->att_sub;
	fds[0].events = POLLIN;

	/******************* System initialization *******************/

	// Failsafe check
	Failsafe::failsafe_check(&close_actuators, 
		&subscription_data->input_rc_signal);

		int ret = poll(fds, 1, 1000);

		if (ret < 0) {

			warnx("poll error");

		} else if (ret == 0) {
			
		} else {

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {

				testbed_control_update_topics();

				Failsafe::failsafe_check(&close_actuators,
					&subscription_data->input_rc_signal);
					
				Measurements::measurements_init(&measurements_vec, 
					&subscription_data->att, &subscription_data->sen_bias);

				Measurements::measurements_est(&measurements_vec, 
					&subscription_data->att, &subscription_data->sen_bias);	

				// Construct state vector
				State::state_init(&state_vec, &measurements_vec, &cov_mat); 

				// Read reference
				Setpoint::set_reference(&setpoint_vec, &subscription_data->input_rc_signal);

				// Control law
				AttControl::control_law(&setpoint_vec, &state_vec, &actuators, 
					&control_vec, &subscription_data->bat_status, close_actuators);
		}
	}


	/* Main loop */
	while (!thread_should_exit) {	

		ret = poll(fds, 1, 1000);

		if (ret < 0) {

			warnx("poll error");

		} else if (ret == 0) {
			
		} else {

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {


				testbed_control_update_topics();
				
				// Failsafe check
				Failsafe::failsafe_check(&close_actuators,
					&subscription_data->input_rc_signal);

				// Read measurements
				Measurements::measurements_est(&measurements_vec, &subscription_data->att,
					&subscription_data->sen_bias);
				
				// Read reference
				Setpoint::set_reference(&setpoint_vec, &subscription_data->
					input_rc_signal);

				// Construct state vector
				State::state_est(&state_vec, &measurements_vec, &control_vec, 
					&cov_mat); 

 				// Control law
				AttControl::control_law(&setpoint_vec, &state_vec, 
					&actuators, &control_vec, &subscription_data->bat_status, close_actuators);
				
				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1])) {

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

	fprintf(stderr, "usage: testbed_control {start|stop|status}\n\n");
}


int testbed_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("testbed_control already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("testbed_control",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 20,
			2048,
			testbed_control_thread_main,
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
			printf("\t testbed_control is running\n");

		} else {
			printf("\t testbed_control not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}


/* Initializes the uORB subscriptions. */
void testbed_control_init(void)
{
	subscription_data = (struct testbed_control_subscription_data_s *)
		calloc(1, sizeof(struct testbed_control_subscription_data_s));

	/* subscribe to topics. */
	subscription_data->battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	subscription_data->sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));
	subscription_data->att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	subscription_data->input_rc_sub = orb_subscribe(ORB_ID(input_rc));

}

/* Check if topics are updated */
void testbed_control_update_topics(void){
	struct testbed_control_subscription_data_s *subs = subscription_data;
	bool updated;

	/* Check if attitude is updated */
	orb_check(subs->att_sub, &updated);

	if (updated){
		orb_copy(ORB_ID(vehicle_attitude), subs->att_sub, &subs->att);
	}

	/* Check if battery voltage is updated */
	orb_check(subs->battery_status_sub, &updated);

	if (updated){
		orb_copy(ORB_ID(battery_status), subs->battery_status_sub, &subs->bat_status);
	}

	/* Check if accelerometer is updated */
	orb_check(subs->sensor_bias_sub, &updated);

	if (updated){
		orb_copy(ORB_ID(sensor_bias), subs->sensor_bias_sub, &subs->sen_bias);
	}
	
	/* Chack if rc input is updated */
	orb_check(subs->input_rc_sub, &updated);

	if (updated){
		orb_copy(ORB_ID(input_rc), subs->input_rc_sub, &subs->input_rc_signal);
	}

}

// Example code debugging

/*
	//print float32
	printf("%8.4f\n", (double) att.yawspeed);

	//Print boolean			
	printf("%d\n", global_sp_updated);

 */