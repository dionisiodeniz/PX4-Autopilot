/**
ProVer-Cert Source Code

Copyright 2026 Carnegie Mellon University.

NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.

Licensed under a BSD (SEI)-style license, please see license.txt or contact permission@sei.cmu.edu for full terms.

[DISTRIBUTION STATEMENT A] This material has been approved for public release and unlimited distribution.  Please see Copyright notice for non-US Government use and distribution.

This Software includes and/or makes use of Third-Party Software each subject to its own license.

DM26-0076
*/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

//#include <px4_platform_common/module.h>
//#include <px4_platform_common/module_params.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>



/**
 * transition counter. This is an array indexed by transition index.
 * It is the resposibility of the user to assign transition indices to each of the
 * transitions being recorded. Since this is an array the index needs to be consecutive.
*/
#define MAX_TRANSITIONS 100
#define MAX_LOGGED_VALUES 50000

static int daemon_task;             /* Handle of deamon task / thread */

unsigned long long transition_counters[MAX_TRANSITIONS];

struct cfg_debug_value {
	int valueid;
	double value;
};

int cfg_debug_value_next_idx =0;
struct cfg_debug_value cfg_debug_values[MAX_LOGGED_VALUES];

__EXPORT int cfg_logger_main(int argc, char *argv[]);
__EXPORT int cfg_logger_increment_transition_counter(int transition_idx);
__EXPORT int cfg_logger_add_debug_value(int valueid, double value);
__EXPORT int cfg_logger_get_euler_from_attitude(float *q, double *yaw, double *roll, double *pitch);


static bool cfg_logger_active=false;

//Dio.TODO: verify the quaternion order for w,x,y,z
int cfg_logger_get_euler_from_attitude(float *q, double *yaw, double *roll, double *pitch){
	// Quaternion encoding
	// q[0] = w, q[1] = x, q[2] = y, q[3] = z
	int w=0,x=1,y=2,z=3;
	//double sqw = q[w] * q[w];
	double sqx = q[x] * q[x];
	double sqy = q[y] * q[y];
	double sqz = q[z] * q[z];

	// Roll (x-axis rotation)
	double sinr_cosp = 2 * (q[w] * q[x] + q[y] * q[z]);
	double cosr_cosp = 1 - 2 * (sqx + sqy);
	*roll = atan2(sinr_cosp, cosr_cosp);

	// to degrees
	* roll = *roll * (180.0 / M_PI);

	// Pitch (y-axis rotation)
	double sinp = 2 * (q[w] * q[y] - q[z] * q[x]);
	if (fabs(sinp) >= 1) {
		*pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
	} else {
		*pitch = asin(sinp);
	}

	// to degrees
	*pitch = *pitch * (180.0 / M_PI);

	// Yaw (z-axis rotation)
	double siny_cosp = 2 * (q[w] * q[z] + q[x] * q[y]);
	double cosy_cosp = 1 - 2 * (sqy + sqz);
	*yaw = atan2(siny_cosp, cosy_cosp);

	// to degrees
	*yaw = *yaw * (180.0 / M_PI);

	return 0;
}

int listener_request_exit =0;

int listener(int argc, char **argv) {
	int error_counter=0;
	FILE *pipe=NULL;

	unsigned long long logcnt = 0L;

	//px4::init(argc, argv, "cfg_logger_listener");

	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
	};

	if ((pipe=fopen("/tmp/drone-attitude.pipe","w")) == NULL){
		PX4_ERR("Error: could not open named pipe");
	}

	while (!listener_request_exit){
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_attitude_s att;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), sensor_sub_fd, &att);
				double yaw,roll,pitch;

				cfg_logger_get_euler_from_attitude(att.q,&yaw,&roll,&pitch);
				cfg_logger_add_debug_value(10, yaw);
				cfg_logger_add_debug_value(11, roll);
				cfg_logger_add_debug_value(12, pitch);

				if (pipe != NULL){
					fprintf(pipe, "%llu,%f,%f\n",logcnt++, roll,pitch);
					fflush(pipe);
				}
			}
		}
	}

	printf("cfg_logger_listener\n");
	return 0;
}


int cfg_logger_main(int argc, char *argv[])
{
	PX4_INFO("CFG_Logger");
	if (argc ==2){
		if (!strcmp(argv[1], "reset")){
			memset(transition_counters,0,sizeof(unsigned long long)*MAX_TRANSITIONS);
			PX4_INFO("counters reset");
		} else if (!strcmp(argv[1], "dump")){
			FILE *fptr = fopen("transitions.json", "w");
			fprintf(fptr, "[\n");
			printf("Transition counters > 0:\n");
			int firsttime=1;
			for (int i=0;i<MAX_TRANSITIONS;i++){
				if (transition_counters[i] >0){
				  if (!firsttime){
				    fprintf(fptr,",");
				  } else {
				    firsttime=0;
				  }
					fprintf(fptr,"{\"transition_number\": %d,\n",i);
					fprintf(fptr,"\"count\": %llu}\n",transition_counters[i]);
					printf("transition[%d].counter = %llu,\n",i,transition_counters[i]);
				}
			}
			fprintf(fptr, "]\n");

			printf("--- debug values ---\n");
			FILE *fptrvalues = fopen("debug_values.csv","w");
			for (int i = 0;i<cfg_debug_value_next_idx;i++){
				//printf(" %d, %d, %f\n",i,cfg_debug_values[i].valueid,cfg_debug_values[i].value);
				fprintf(fptrvalues,"%d,%d,%f\n",i,cfg_debug_values[i].valueid,cfg_debug_values[i].value);
			}
			fclose(fptrvalues);
			fclose(fptr);
		}else if (!strcmp(argv[1], "start")){
			memset(transition_counters,0,sizeof(unsigned long long)*MAX_TRANSITIONS);
			for (int i=0;i<MAX_LOGGED_VALUES;i++){
				cfg_debug_values[i].value=0.0;
				cfg_debug_values[i].valueid=0;
			}

			PX4_INFO("counters reset");
			cfg_logger_active = true;
		}else if (!strcmp(argv[1], "stop")){
			cfg_logger_active = false;
			listener_request_exit = 1;
		}else if (!strcmp(argv[1], "listen")){
			daemon_task = px4_task_spawn_cmd("cfg_logger_listener",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX - 5,
					2000,
					listener,
					(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		}
		else {
			//return ModuleBase::main(Logger::desc, argc, argv);
			PX4_INFO("unrecognized command. Valid commands: reset, dump, start, stop");
		}
	}
	return 0;//ModuleBase::main(Logger::desc, argc, argv);
}

int cfg_logger_increment_transition_counter(int transition_idx){
	if (!cfg_logger_active){
		return -1;
	}

	if (transition_idx >= MAX_TRANSITIONS || transition_idx<0){
		PX4_INFO("CFG_logger.cfg_logger_increment_transition_counter(): transition index out of bounds");
		return -1;
	}

	transition_counters[transition_idx] += 1L;

	return 0;
}

int cfg_logger_add_debug_value(int valueid, double value){

	if (!cfg_logger_active){
		return -1;
	}

	if (cfg_debug_value_next_idx >= MAX_LOGGED_VALUES){
		return -1;
	}

	cfg_debug_values[cfg_debug_value_next_idx].valueid = valueid;
	cfg_debug_values[cfg_debug_value_next_idx].value = value;

	cfg_debug_value_next_idx++;

	return 0;
}
