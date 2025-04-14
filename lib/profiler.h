#pragma once

/* MC Function Performance Monitoring */

// TODO Rewrite this so that the sum doesn't overflow - use formula for adding incremental measurements into an average
#define MC_PROFILE(name, fcall) static FLFunctionProfile name;\
							uint32_t name##_pre = micros();\
							{ fcall }\
							uint32_t name##_diff = micros() - name##_pre;\
							(name).t_sum += name##_diff;\
							(name).n_measure +=1;

#define MC_RESET(name) (name).n_measure=0; (name).t_sum=0;

typedef struct {
	uint32_t n_measure=0;
	uint32_t t_sum=0;
} FLFunctionProfile;
