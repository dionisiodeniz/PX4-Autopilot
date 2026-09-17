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

// #include <px4_platform_common/module.h>
// #include <px4_platform_common/module_params.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>

#include "cfg_logger.h"



/**
 * transition counter. This is an array indexed by transition index.
 * It is the resposibility of the user to assign transition indices to each of the
 * transitions being recorded. Since this is an array the index needs to be consecutive.
*/
static int daemon_task=-1;             /* Handle of deamon task / thread */

unsigned long long transition_counters[MAX_TRANSITIONS];

int transition2path_index[MAX_TRANSITIONS][MAX_PATH_OBSERVED];

int cfg_debug_value_next_idx =0;
struct cfg_debug_value cfg_debug_values[MAX_LOGGED_VALUES];

struct cfg_const_value cfg_constants[MAX_CONSTANTS];

int nextpath=0;
struct cfg_path_obs cfg_path_observations[MAX_PATH_OBSERVED];

__EXPORT int cfg_logger_main(int argc, char *argv[]);

// __EXPORT int cfg_logger_main(int argc, char *argv[]);
// __EXPORT int cfg_logger_increment_transition_counter(int transition_idx);
// __EXPORT int cfg_logger_add_debug_value(int valueid, double value);
// __EXPORT int cfg_logger_get_euler_from_attitude(float *q, double *yaw, double *roll, double *pitch);

static bool cfg_logger_active=false;

void init_paths_observed(void){
	for (int i=0;i<MAX_TRANSITIONS;i++){
		for (int j=0;j<MAX_PATH_OBSERVED;j++){
			transition2path_index[i][j]=-1;
		}
	}

	for (int i=0;i<MAX_PATH_OBSERVED;i++){
		cfg_path_observations[i].nxt_transition=0;
	}
}

int cfg_get_constant_value(const char *name, double *val){
	for (int i=0;i<MAX_CONSTANTS; i++){
		if (!strcmp(cfg_constants[i].name,name)){
			*val = cfg_constants[i].value;
			return 0;
		}
	}
	return -1;
}

int cfg_set_constant_value(char *name, double val){
	int foundid = -1;
	int freeid = -1;
	for (int i=0;i<MAX_CONSTANTS; i++){
		if (!strcmp(cfg_constants[i].name,name)){
			foundid = i;
			// if I find the name I do not look for an empty space
			break;
		} else if (!strcmp(cfg_constants[i].name,"")){
			if (freeid == -1){
				freeid = i;
				// All empty records are at the end so
				// this also means that I could not fine the const name
				break;
			}
		}
	}

	// check first for foundid
	if (foundid != -1){
		// replace the value if another one was there
		cfg_constants[foundid].value = val;
		return 0;
	} else if (freeid != -1){
		// put in new record if no previous one
		strncpy(cfg_constants[freeid].name,name,CONST_NAME_LEN);
		cfg_constants[freeid].value = val;
		return 0;
	} else {
		// error I could not find tha name or an empty space
		return -1;
	}
}

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

int cfg_logger_add_path(int cnt, int argv[]){
	int pathidx=0;
	// Initialize args to store the variable arguments after 'count'

	if (nextpath >= MAX_PATH_OBSERVED)
		return -1;

	 pathidx = nextpath++;


	for (int i = 0; i < cnt; i++) {
		// Retrieve the next argument as an integer
		int trans_idx =  argv[i];

		cfg_path_observations[pathidx].trans_indices[i]=trans_idx;
		cfg_path_observations[pathidx].nxt_transition = 0;

		for (int j=0;j<MAX_PATH_OBSERVED;j++){
			if (transition2path_index[trans_idx][j] == -1){
				transition2path_index[trans_idx][j] = pathidx;
				break;
			}
		}

		if (i == (cnt-1)){
			cfg_path_observations[pathidx].last_transition_index = i;
		}
	}

	return pathidx;
}

int cfg_logger_get_path(int pathidx, char *str, int strlen){
	int offset = 0;
	if (pathidx >= MAX_PATH_OBSERVED)
		return -1;

	offset += snprintf(str,strlen,"%d:{",pathidx);
	for (int i=0;i<MAX_PATH_TRANSITIONS;i++){
		offset += snprintf(str+offset, strlen-offset, "%d", cfg_path_observations[pathidx].trans_indices[i]);
		if (cfg_path_observations[pathidx].last_transition_index != i){
			offset += snprintf(str+offset,strlen-offset,",");
		} else {
			break;
		}
	}
	offset += snprintf(str+offset,strlen-offset,"}");
	return 0;
}

int cfg_logger_save_transition_observations(char *filename){
	FILE *fptr = fopen(filename, "w");
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
	fclose(fptr);
	return 0;
}

int cfg_logger_save_debug_values(char *filename){
	FILE *fptrvalues = fopen(filename,"w");
	for (int i = 0;i<cfg_debug_value_next_idx;i++){
		fprintf(fptrvalues,"%d,%d,%f\n",i,cfg_debug_values[i].valueid,cfg_debug_values[i].value);
	}
	fclose(fptrvalues);
	return 0;
}

int cfg_logger_save_path_observations(char *filename){
	char str[50];
	FILE *fptr = fopen(filename, "w");
	fprintf(fptr, "[\n");

	for (int i=0;i<nextpath;i++){
		cfg_logger_get_path(i,str,50);
		fprintf(fptr,"{\"pathid\": %d,\n\"count\":%llu,\n\"path\":\"%s\"},",i,cfg_path_observations[i].full_path_traversal,str);
	}
	fprintf(fptr, "]\n");
	fclose(fptr);
	return 0;
}

int listener_request_exit =0;

int listener(int argc, char **argv) {
	double val;
	static uint64_t lastlandingtime;
	static int landing_detected = 0;
	static int was_landed = 1; // start as landed
	int error_counter=0;
	FILE *pipe=NULL;

	unsigned long long logcnt = 0L;

	//px4::init(argc, argv, "cfg_logger_listener");

	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_land_fd = orb_subscribe(ORB_ID(vehicle_land_detected));

	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = vehicle_land_fd,   .events = POLLIN },
	};

	if (cfg_get_constant_value("plot_attitude", &val)==0){
		if ((pipe=fopen("/tmp/drone-attitude.pipe","w")) == NULL){
			PX4_ERR("Error: could not open named pipe");
		}
	}

	while (!listener_request_exit){
		int poll_ret = px4_poll(fds, 2, 1000);

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
			if (fds[1].revents & POLLIN) {
				struct vehicle_land_detected_s land_event;
				orb_copy(ORB_ID(vehicle_land_detected),vehicle_land_fd, &land_event);

				if (was_landed && !land_event.landed) {
					// only when it was landed and took off
					was_landed = land_event.landed;
				} else if (!was_landed && land_event.landed) {
					// only when was in the air and switch to landed
					was_landed = land_event.landed;
					if (lastlandingtime != land_event.timestamp){
						lastlandingtime = land_event.timestamp;
						if (!landing_detected){
							landing_detected = 1;
							if (cfg_get_constant_value("save_on_landing", &val)<0){
								printf("Landed but no \"save_on_landing\" constant set so not saving\n");
							} else {
								cfg_logger_save_transition_observations("transitions.json");
								cfg_logger_save_debug_values("debug_values.csv");
								cfg_logger_save_path_observations("paths.json");
							}
						}
					}
				}
			}
		}
	}

	printf("cfg_logger_listener exiting\n");
	return 0;
}

int cfg_logger_main(int argc, char *argv[])
{
	PX4_INFO("CFG_Logger");
	char str[50];

	if (argc >1){
		if (!strcmp(argv[1], "reset")){
			init_paths_observed();
			memset(transition_counters,0,sizeof(unsigned long long)*MAX_TRANSITIONS);
			PX4_INFO("counters reset");
		} else if (!strcmp(argv[1], "dump")){
			cfg_logger_save_transition_observations("transitions.json");
			cfg_logger_save_debug_values("debug_values.csv");
			cfg_logger_save_path_observations("paths.json");
		}else if (!strcmp(argv[1], "start")){
			init_paths_observed();
			memset(transition_counters,0,sizeof(unsigned long long)*MAX_TRANSITIONS);
			for (int i=0;i<MAX_LOGGED_VALUES;i++){
				cfg_debug_values[i].value=0.0;
				cfg_debug_values[i].valueid=0;
			}

			for (int i=0;i<MAX_CONSTANTS;i++){
				strncpy(cfg_constants[i].name,"",CONST_NAME_LEN);
			}

			PX4_INFO("counters reset");
			cfg_logger_active = true;
		}else if (!strcmp(argv[1], "stop")){
			cfg_logger_active = false;
			listener_request_exit = 1;
			// if (daemon_task != -1){
			// 	 px4_task_delete(daemon_task);
			// }
		}else if (!strcmp(argv[1], "listen")){
			daemon_task = px4_task_spawn_cmd("cfg_logger_listener",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX - 5,
					2000,
					listener,
					(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		} else if (!strcmp(argv[1], "setconstant")){
			PX4_INFO("got setconstant\n");
			if (argc == 4){
				char *constantname = argv[2];
				double constantvalue = atof(argv[3]);
				cfg_set_constant_value(constantname,constantvalue);
				PX4_INFO_RAW("constant %s set to %f\n",constantname,constantvalue);
			} else {
				PX4_INFO_RAW("argc == %d\n",argc);
			}
		} else if (!strcmp(argv[1], "getconstant")){
			PX4_INFO("got getconstant\n");
			if (argc == 3){
				char *constantname = argv[2];
				double val;
				if (cfg_get_constant_value(constantname, &val)<0){

					PX4_INFO("constant does not extist");
				} else {
					PX4_INFO_RAW("value : %f\n",val);
				}
			} else {
				PX4_INFO_RAW("argc == %d\n",argc);
			}
		} else if (!strcmp(argv[1], "setpath")){
			int pathidx;

			PX4_INFO("got setpath\n");
			if (argc == 3){
				char *translist = argv[2];
				int cnt=0;
				int trans[MAX_TRANSITIONS];

				char *token = strtok(translist, ",");

				// Walk through other tokens
				while (token != NULL) {
					trans[cnt] = atoi(token); // Convert token to integer
					cnt++;
					token = strtok(NULL, ","); // Get next token
				}

				pathidx = cfg_logger_add_path(cnt, trans);
				cfg_logger_get_path(pathidx,str,50);
				PX4_INFO_RAW("path(%d):%s\n",pathidx,str);
			}
		} else {
			//return ModuleBase::main(Logger::desc, argc, argv);
			PX4_INFO("unrecognized command. Valid commands: reset, dump, start, stop");
		}
	}
	return 0;//ModuleBase::main(Logger::desc, argc, argv);
}

int cfg_logger_track_transition_path(int transition_idx){
	for (int i=0;i<MAX_PATH_OBSERVED;i++){
		int path_idx = transition2path_index[transition_idx][i];

		if (path_idx >= 0){
			// if the next transition idx is the one in the parameter it means that
			// we checked all the previous ones and we advance to the next transaction
			// on the path, otherwise this means that a previous transition in the
			// path we were not able to fullfil.
			if (cfg_path_observations[path_idx].trans_indices[cfg_path_observations[path_idx].nxt_transition] == transition_idx){
				if (cfg_path_observations[path_idx].nxt_transition == cfg_path_observations[path_idx].last_transition_index){
					cfg_path_observations[path_idx].full_path_traversal++;
					cfg_path_observations[path_idx].nxt_transition = 0;
					transition_counters[80] += 1L;

				} else {
					transition_counters[81] += 1L;
					cfg_path_observations[path_idx].nxt_transition++;
				}
			} else if (cfg_path_observations[path_idx].trans_indices[0] == transition_idx) {
				// start from zero again without incrementing the path count
				cfg_path_observations[path_idx].nxt_transition = 1;
			}
		} else {
			transition_counters[82] += 1L;
			// stop the traversal of paths on the first -1;
			break;
		}
	}
	return 0;
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

	cfg_logger_track_transition_path(transition_idx);

	return 0;
}

// This is not thread safe
// ToDo: Need to add mutex lock/unlock to prevent race conditions when called from
// multiple threads.
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
