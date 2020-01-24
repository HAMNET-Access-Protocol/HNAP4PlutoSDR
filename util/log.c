/*
 * log.c
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#include "log.h"

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
