/*
 * log.c
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#include "log.h"
#include <time.h>

// Matlab log helper functions
void log_matlab_fc(float complex* cpx, int num_samps, char* filename)
{
	FILE * fd = fopen(filename, "w");
	if (!fd) {
		return;
	}

	fprintf(fd,"samps = zeros(1,%d);\n",num_samps);
	for (int i=0; i<num_samps; i++) {
		fprintf(fd,"samps(%d) = %f + i*%f;\n",i+1,creal(cpx[i]),cimag(cpx[i]));
	}
	fclose(fd);
}

void log_matlab_f(float* floats, int num_samps, char* filename)
{
	FILE * fd = fopen(filename, "w");
	if (!fd) {
		return;
	}

	fprintf(fd,"samps = zeros(1,%d);\n",num_samps);
	for (int i=0; i<num_samps; i++) {
		fprintf(fd,"samps(%d) = %f;\n",i+1,floats[i]);
	}
	fclose(fd);
}

void log_matlab_i(int* buf, int num_samps, char* filename)
{
	FILE * fd = fopen(filename, "w");
	if (!fd) {
		return;
	}

	fprintf(fd,"samps = zeros(1,%d);\n",num_samps);
	for (int i=0; i<num_samps; i++) {
		fprintf(fd,"samps(%d) = %d;\n",i+1,buf[i]);
	}
	fclose(fd);
}

void log_bin(uint8_t* buf, uint buf_len, char* filename, char* mode)
{
	FILE * fd = fopen(filename, mode);
	if (!fd) {
		return;
	}
	fwrite(buf,buf_len,1, fd);
	fclose(fd);
}

void timecheck_stop(struct timecheck_s* time)
{
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC,&end);
    float elapsed = (end.tv_sec-time->start.tv_sec)*1000000.0 +(end.tv_nsec-time->start.tv_nsec)/1000.0;
    time->avg += elapsed;
    time->count++;
    if (elapsed>time->max)
        time->max = elapsed;
}

void timecheck_info(struct timecheck_s* time)
{
    if (time->count%time->avg_len==0) {
        LOG(WARN, "TimingMon '%20s:' avg:%4.3fus max:%4.3f after %6d iter\n",time->name,time->avg/time->avg_len,
                time->max,time->count);
        time->avg = 0;
        time->max = 0;
    }
}
