#include "phy/phy_bs.h"
#include "phy/phy_ue.h"
#include <time.h>


#include "log.h"

// biterror test variable definitions
unsigned char payload[1024];
int curr_subframe = 0;
PhyUE phy_ue;


void mac_callback(LogicalChannel chan)
{
	//Calc Biterrors TODO
	//static uint total_error = 0;
	//static uint total_bits = 0;
	//uint biterr = count_bit_errors_array(chan->data,payload+chan->payload_len*chan->writepos,chan->payload_len);
	//total_error+=biterr;
	//total_bits +=8*(chan->payload_len);
	//biterr_buf[4*curr_subframe+chan->writepos] = (int)biterr;
	//printf("Biterrors: %d rate: %f total bits: %d\n", biterr, total_error*1.0/total_bits, total_bits);

	// Calc CFO estimation error
	float cfo_err = ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI;
	//cfo_buf[4*curr_subframe+chan->writepos] = cfo_err;
	printf("Iter %d CFO: %f\n",curr_subframe,ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI);

	free(chan->data);
}

// Performs Biterror and CFO estimation error test
int main()
{
	clock_t before;
	float elapsed_rx=0, elapsed_tx=0;
    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex  buf[buf_len];
    memset(buf,0,buf_len*sizeof(float complex));

    uint mcs = 0;

    phy_ue = phy_init_ue();
    phy_ue->common->userid = 0x01;
    phy_ue_set_mac_cb(phy_ue, mac_callback);
    phy_ue_set_mcs_dl(phy_ue, mcs);

    LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

    ofdmframesync_debug_enable(phy_ue->fs);

    FILE* fd = fopen("/home/lukas/MATLAB/test.bin","r");
    uint read_samps = 1;
    uint iterations = 0;

    while(read_samps > 0) {
    	// read from file
    	read_samps = fread(buf,buf_len,sizeof(float complex),fd);

        // Phy channel demod
    	if (read_samps > 0) {
    		before = clock();
        	phy_ue_do_rx(phy_ue, buf, buf_len);
        	elapsed_rx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
        	curr_subframe++;
    	}
    }

    free(chan);

    printf("RX: %fms avg RX: %fms\n",elapsed_rx, elapsed_rx/curr_subframe);

    //char filename[80] = {0};
    //sprintf(filename,"cfo_test_%.0fHz_gsmTU_chan.m",cfo_test);
    //log_matlab_i(biterr_buf,4*iterations,"biterr.m");
    //log_matlab_f(cfo_buf,4*iterations,filename);
    return 0;
}
