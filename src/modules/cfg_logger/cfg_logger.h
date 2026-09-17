#define MAX_TRANSITIONS 100
#define MAX_LOGGED_VALUES 50000
#define MAX_CONSTANTS 20
#define CONST_NAME_LEN 50
#define MAX_PATH_TRANSITIONS 50
#define MAX_PATH_OBSERVED 50

struct cfg_debug_value {
	int valueid;
	double value;
};

struct cfg_const_value {
	char name[CONST_NAME_LEN];
	double value;
};

struct cfg_path_obs {
	int nxt_transition;
	int last_transition_index;
	int trans_indices[MAX_PATH_TRANSITIONS];
	unsigned long long full_path_traversal;
};

__EXPORT int cfg_logger_main(int argc, char *argv[]);
__EXPORT int cfg_logger_increment_transition_counter(int transition_idx);
__EXPORT int cfg_logger_add_debug_value(int valueid, double value);
__EXPORT int cfg_logger_get_euler_from_attitude(float *q, double *yaw, double *roll, double *pitch);
__EXPORT int cfg_get_constant_value(const char *name, double *value);


