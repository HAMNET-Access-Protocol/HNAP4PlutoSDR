/*
 * pluto_test.c
 *
 *  Created on: 27.12.2019
 *      Author: lukas
 */
#include "pluto.h"
#include "../log.h"
#include "../phy/phy_config.h"

#include <string.h>
#include <signal.h>
#include <time.h>
#include <liquid/liquid.h>
#include <stdbool.h>

static bool stop;

static void handle_sig(int sig)
{
	printf("Waiting for process to finish...\n");
	stop = true;
}
int main()
{
	clock_t start;
	// Listen to ctrl+c and ASSERT
	signal(SIGINT, handle_sig);

    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];
    memset(buf_in,0,buf_len*sizeof(float complex));
    memset(buf_out,0,buf_len*sizeof(float complex));

    modem qammod = modem_create(LIQUID_MODEM_QAM4);

    init_pluto_platform(buf_len);

    start = clock();
    while (!stop) {

    	printf("tx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
    	pluto_transmit(buf_len);

    	printf("rx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		pluto_receive(buf_out,buf_len);

    	printf("preptx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
    	for (int i=0; i<buf_len; i+=1) {
    		// generate random QAM symbol
    		uint sym = modem_gen_rand_sym(qammod);
    		modem_modulate(qammod,sym,&buf_in[i]);
    		modem_modulate(qammod,sym,&buf_in[i+1]);
    		//buf_in[i]= (rand() >= RAND_MAX/2) ? 1:-1;
    		//buf_in[i]+= (rand() >= RAND_MAX/2) ? I:-I;
    	}
    	pluto_prep_tx(buf_in,buf_len);

    	printf("log: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		log_bin((uint8_t*)buf_out,buf_len*sizeof(float complex),"test.bin","a");
    }
    shutdown();
	return 0;
}
