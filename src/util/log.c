/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA
 */

#include "log.h"
#include <time.h>

int global_log_level = INFO;

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

// stops an ongoing timing measurement. and updates data
// if crit_delay_us is set greater than zero, it is checked if the current iteration took longer than
// the allowed crit_delay
void timecheck_stop(struct timecheck_s* time, int crit_delay_us)
{
    struct timespec end;
    clock_gettime(CLOCK_MONOTONIC,&end);
    float elapsed = (end.tv_sec-time->start.tv_sec)*1000000.0 +(end.tv_nsec-time->start.tv_nsec)/1000.0;
    time->avg += elapsed;
    time->count++;
    if (elapsed>time->max)
        time->max = elapsed;
    if (crit_delay_us>0 && crit_delay_us<elapsed)
        LOG(WARN,"[TimeMonitor] delay warning: '%s' took %4.1fus\n",time->name,elapsed);
}

void timecheck_info(struct timecheck_s* time)
{
    if (time->count%time->avg_len==0) {
        LOG(INFO, "[TimingMonitor] '%10s': avg:%4.1fus max:%4.1f after %5d iter\n",time->name,time->avg/time->avg_len,
                time->max,time->count);
        time->avg = 0;
        time->max = 0;
    }
}
