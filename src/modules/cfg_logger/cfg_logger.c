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

/**
 * transition counter. This is an array indexed by transition index.
 * It is the resposibility of the user to assign transition indices to each of the
 * transitions being recorded. Since this is an array the index needs to be consecutive.
*/
#define MAX_TRANSITIONS 100

unsigned long long transition_counters[MAX_TRANSITIONS];

__EXPORT int cfg_logger_main(int argc, char *argv[]);

__EXPORT int cfp_logger_increment_transition_counter(int transition_idx);

int cfg_logger_main(int argc, char *argv[])
{
	PX4_INFO("CFG_Logger");
	if (argc ==2){
		if (!strcmp(argv[1], "reset")){
			memset(transition_counters,0,sizeof(unsigned long long)*MAX_TRANSITIONS);
			PX4_INFO("counters reset");
		} else if (!strcmp(argv[1], "dump")){
			printf("Transition counters > 0:\n");
			for (int i=0;i<MAX_TRANSITIONS;i++){
				if (transition_counters[i] >0){
					printf("transition[%d].counter = %llu\n",i,transition_counters[i]);
				}
			}
		}
	}
	return 0;
}

int cfp_logger_increment_transition_counter(int transition_idx){
	if (transition_idx >= MAX_TRANSITIONS || transition_idx<0){
		PX4_INFO("CFG_logger.cfp_logger_increment_transition_counter(): transition index out of bounds");
		return -1;
	}
	transition_counters[transition_idx] += 1L;
	return 0;
}
