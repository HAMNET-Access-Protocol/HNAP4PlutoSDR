#include "phy/phy_bs.h"
#include "phy/phy_ue.h"
#include <time.h>


#include "log.h"

// biterror test variable definitions
unsigned char payload[1024];
#define iterations 1000
int curr_subframe = 0;
int biterr_buf[iterations*4] = {0};
PhyUE phy_ue;

void mac_callback(LogicalChannel chan)
{
	static uint total_error = 0;
	static uint total_bits = 0;
	uint biterr = count_bit_errors_array(chan->data,payload+chan->payload_len*chan->writepos,chan->payload_len);
	total_error+=biterr;
	total_bits +=8*(chan->payload_len);
	biterr_buf[4*curr_subframe+chan->writepos] = (int)biterr;
	printf("Biterrors: %d rate: %f total bits: %d\n", biterr, total_error*1.0/total_bits, total_bits);
	printf("CFO: %f\n",ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI);

	free(chan->data);

}

// cfo error test variable definitions
float cfo_test = 1000;
float cfo_buf[iterations*4] = {0};

void mac_callback_cfo_test(LogicalChannel chan)
{
	//Calc Biterrors
	static uint total_error = 0;
	static uint total_bits = 0;
	uint biterr = count_bit_errors_array(chan->data,payload+chan->payload_len*chan->writepos,chan->payload_len);
	total_error+=biterr;
	total_bits +=8*(chan->payload_len);
	biterr_buf[4*curr_subframe+chan->writepos] = (int)biterr;
	printf("Biterrors: %d rate: %f total bits: %d\n", biterr, total_error*1.0/total_bits, total_bits);

	// Calc CFO estimation error
	float cfo_err = cfo_test-ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI;
	cfo_buf[4*curr_subframe+chan->writepos] = cfo_err;
	printf("Iter %d CFO: %f\n",curr_subframe,ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/3.14);

	free(chan->data);
}

// Performs a test of bit error rate using the defined channel
int main_biterr_test() {
	clock_t before;
	float elapsed_rx=0, elapsed_tx=0;
    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];
    memset(buf_in,0,buf_len*sizeof(float complex));
    memset(buf_out,0,buf_len*sizeof(float complex));


    // create channel
    channel_cccf channel = channel_cccf_create();

    // AWGN
    float noise_floor = -60.0;
    float SNR = 19;
    channel_cccf_add_awgn(channel,noise_floor, SNR);
    
    // Multipath
    // GSM typical urban 12 tap scenario 1
    float complex gsmTUx12c1[] = {   0.0010 + 0.0013i,
    		   0.0020 + 0.0079i,
    		  -0.0092 - 0.0218i,
    		   0.0111 + 0.0325i,
    		  -0.0154 - 0.0500i,
    		   0.0226 + 0.0807i,
    		  -0.0373 - 0.1543i,
    		   0.1526 + 0.9226i,
    		   0.8225 + 0.4973i,
    		  -0.0054 - 0.0771i,
    		   0.0166 + 0.0542i,
    		  -0.0130 - 0.0359i,
    		   0.0098 + 0.0238i,
    		  -0.0072 - 0.0154i,
    		   0.0052 + 0.0095i,
    		  -0.0035 - 0.0054i,
    		  -0.0012 - 0.0006i,
    		   0.0000 + 0.0000i
    };
    uint gsmTUx12c1_len = 18;
    float complex hc[] = {1, 0, 0, 0};
    uint hc_len = 4;		// number of channel filter taps
    //channel_cccf_add_multipath(channel, gsmTUx12c1, gsmTUx12c1_len);

    // frequency offset
    float cfo = 0;	// frequency offset in Hertz
    float dphi = (2*M_PI*cfo)/256000.0;	//frequency offset in radians/sample
    float phi = 2;		// phase offset in radians
    channel_cccf_add_carrier_offset(channel, dphi, phi);

    // slow-flat fading
    float sigma = 1;	// standard deviation for log-normal shadowing
    float fd = 20.0/256000.0;		// relative Doppler frequency
    //channel_cccf_add_shadowing(channel, sigma, fd);

    channel_cccf_print(channel);

    uint mcs = 3;

    PhyBS phy_bs = phy_init_bs();
    phy_ue = phy_init_ue();
    phy_ue->common->userid = 0x01;
    phy_ue_set_mac_cb(phy_ue, mac_callback);
    phy_ue_set_mcs_dl(phy_ue, mcs);

    LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

    ofdmframesync_debug_enable(phy_ue->fs);

    for (curr_subframe=0; curr_subframe<iterations;curr_subframe++) {
        // generate random payload
    	uint tbs = get_tbs_size(phy_bs->common,mcs);
        for (int j=0; j<4*tbs/8; j++) {
            payload[j] = rand() & 0xff;
        }

        // Phy channel mapping
        chan->userid = 0x01;
        chan->payload_len = tbs/8;
        chan->data = payload;

        before = clock();
        phy_map_dlslot(phy_bs, chan, 0, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 1, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 2, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 3, mcs);

        phy_write_subframe(phy_bs, buf_in);
        elapsed_tx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
        //log_matlab(buf_in,buf_len,"tx_time.m");
        // simulate channel
        channel_cccf_execute_block(channel, buf_in,buf_len,buf_out);

        // Phy channel demod
        before = clock();
        phy_ue_do_rx(phy_ue, buf_out, buf_len);
        elapsed_rx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
    }

    free(chan);

    channel_cccf_destroy(channel);
    printf("RX: %fms TX:%fms avg RX: %fms avg TX: %fms\n",elapsed_rx,elapsed_tx,elapsed_rx/iterations,elapsed_tx/iterations);
    log_matlab_i(biterr_buf,4*iterations,"biterr.m");
    return 0;
}


// Performs Biterror and CFO estimation error test
int main_cfo_estimation_test()
{
	clock_t before;
	float elapsed_rx=0, elapsed_tx=0;
    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];
    memset(buf_in,0,buf_len*sizeof(float complex));
    memset(buf_out,0,buf_len*sizeof(float complex));


    // create channel
    channel_cccf channel = channel_cccf_create();

    // AWGN
    float noise_floor = -0.0;
    float SNR = 20;
    channel_cccf_add_awgn(channel,noise_floor, SNR);

    // Multipath
    // GSM typical urban 12 tap scenario 1
    float complex gsmTUx12c1[] = {   0.0010 + 0.0013i,
    		   0.0020 + 0.0079i,
    		  -0.0092 - 0.0218i,
    		   0.0111 + 0.0325i,
    		  -0.0154 - 0.0500i,
    		   0.0226 + 0.0807i,
    		  -0.0373 - 0.1543i,
    		   0.1526 + 0.9226i,
    		   0.8225 + 0.4973i,
    		  -0.0054 - 0.0771i,
    		   0.0166 + 0.0542i,
    		  -0.0130 - 0.0359i,
    		   0.0098 + 0.0238i,
    		  -0.0072 - 0.0154i,
    		   0.0052 + 0.0095i,
    		  -0.0035 - 0.0054i,
    		  -0.0012 - 0.0006i,
    		   0.0000 + 0.0000i
    };
    uint gsmTUx12c1_len = 18;
    float complex hc[] = {1, 0, 0, 0};
    uint hc_len = 4;		// number of channel filter taps
    channel_cccf_add_multipath(channel, gsmTUx12c1, gsmTUx12c1_len);

    // frequency offset
    float dphi = (2*M_PI*cfo_test)/256000.0;	//frequency offset in radians/sample
    float phi = 2;		// phase offset in radians
    channel_cccf_add_carrier_offset(channel, dphi, phi);

    // slow-flat fading
    float sigma = 1.0;	// standard deviation for log-normal shadowing
    float fd = 0.002;		// relative Doppler frequency
    //channel_cccf_add_shadowing(channel, sigma, fd);

    channel_cccf_print(channel);

    uint mcs = 3;

    PhyBS phy_bs = phy_init_bs();
    phy_ue = phy_init_ue();
    phy_ue->common->userid = 0x01;
    phy_ue_set_mac_cb(phy_ue, mac_callback_cfo_test);
    phy_ue_set_mcs_dl(phy_ue, mcs);

    LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

    ofdmframesync_debug_enable(phy_ue->fs);

    for (curr_subframe=0; curr_subframe<iterations;curr_subframe++) {
        // generate random payload
    	uint tbs = get_tbs_size(phy_bs->common,mcs);
        for (int j=0; j<4*tbs/8; j++) {
            payload[j] = rand() & 0xff;
        }

        // Phy channel mapping
        chan->userid = 0x01;
        chan->payload_len = tbs/8;
        chan->data = payload;

        before = clock();
        phy_map_dlslot(phy_bs, chan, 0, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 1, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 2, mcs);
        chan->data += tbs/8;
        phy_map_dlslot(phy_bs, chan, 3, mcs);

        phy_write_subframe(phy_bs, buf_in);
        elapsed_tx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
        //log_matlab(buf_in,buf_len,"tx_time.m");
        // simulate channel
        channel_cccf_execute_block(channel, buf_in,buf_len,buf_out);
        log_matlab_fc(buf_in,buf_len,"txdata_t.m");
        // Phy channel demod
        before = clock();
        phy_ue_do_rx(phy_ue, buf_out, buf_len);
        elapsed_rx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
    }

    free(chan);

    channel_cccf_destroy(channel);
    printf("RX: %fms TX:%fms avg RX: %fms avg TX: %fms\n",elapsed_rx,elapsed_tx,elapsed_rx/iterations,elapsed_tx/iterations);

    char filename[80] = {0};
    sprintf(filename,"cfo_test_%.0fHz_snr%.0f_gsmTU_chan.m",cfo_test,SNR);
    log_matlab_i(biterr_buf,4*iterations,"biterr.m");
    log_matlab_f(cfo_buf,4*iterations,filename);
    return 0;
}

int main_sync_cfo_estimation()
{
	clock_t before;
	float elapsed_rx=0, elapsed_tx=0;
    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];

    float cfos[iterations];
    int num_syncs = 0;

    // create channel
    channel_cccf channel = channel_cccf_create();

    // AWGN
    float noise_floor = -60.0;
    float SNR = 10;
    channel_cccf_add_awgn(channel,noise_floor, SNR);

    // Multipath
    // GSM typical urban 12 tap scenario 1
    float complex gsmTUx12c1[] = {   0.0010 + 0.0013i,
    		   0.0020 + 0.0079i,
    		  -0.0092 - 0.0218i,
    		   0.0111 + 0.0325i,
    		  -0.0154 - 0.0500i,
    		   0.0226 + 0.0807i,
    		  -0.0373 - 0.1543i,
    		   0.1526 + 0.9226i,
    		   0.8225 + 0.4973i,
    		  -0.0054 - 0.0771i,
    		   0.0166 + 0.0542i,
    		  -0.0130 - 0.0359i,
    		   0.0098 + 0.0238i,
    		  -0.0072 - 0.0154i,
    		   0.0052 + 0.0095i,
    		  -0.0035 - 0.0054i,
    		  -0.0012 - 0.0006i,
    		   0.0000 + 0.0000i
    };
    uint gsmTUx12c1_len = 18;
    float complex hc[] = {1, 0, 0, 0};
    uint hc_len = 4;		// number of channel filter taps
    channel_cccf_add_multipath(channel, gsmTUx12c1, gsmTUx12c1_len);

    // frequency offset
    float dphi = (2*M_PI*cfo_test)/256000.0;	//frequency offset in radians/sample
    float phi = 2;		// phase offset in radians
    channel_cccf_add_carrier_offset(channel, dphi, phi);

    // slow-flat fading
    float sigma = 10.0;	// standard deviation for log-normal shadowing
    float fd = 40.0/256000;		// relative Doppler frequency
    channel_cccf_add_shadowing(channel, sigma, fd);

    channel_cccf_print(channel);

    uint mcs = 3;

    PhyBS phy_bs = phy_init_bs();
    phy_ue = phy_init_ue();
    phy_ue->common->userid = 0x01;
    phy_ue_set_mac_cb(phy_ue, mac_callback_cfo_test);
    phy_ue_set_mcs_dl(phy_ue, mcs);

    LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

    ofdmframesync_debug_enable(phy_ue->fs);

    for (curr_subframe=0; curr_subframe<iterations;curr_subframe++) {
        // generate random payload
     	uint tbs = get_tbs_size(phy_bs->common,mcs);
         for (int j=0; j<4*tbs/8; j++) {
             payload[j] = rand() & 0xff;
         }

         // Phy channel mapping
         chan->userid = 0x01;
         chan->payload_len = tbs/8;
         chan->data = payload;

         before = clock();
         phy_map_dlslot(phy_bs, chan, 0, mcs);
         chan->data += tbs/8;
         phy_map_dlslot(phy_bs, chan, 1, mcs);
         chan->data += tbs/8;
         phy_map_dlslot(phy_bs, chan, 2, mcs);
         chan->data += tbs/8;
         phy_map_dlslot(phy_bs, chan, 3, mcs);

         phy_write_subframe(phy_bs, buf_in);
         elapsed_tx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
         //log_matlab(buf_in,buf_len,"tx_time.m");
         // simulate channel
         channel_cccf_execute_block(channel, buf_in,buf_len,buf_out);

         // Phy channel demod
         before = clock();
         int offset = phy_ue_initial_sync(phy_ue,buf_out, buf_len);
         if (offset>0) {
        	 cfos[num_syncs++] = ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI;
         }
         elapsed_rx += (clock() - before)*1000.0/CLOCKS_PER_SEC;
    }
    free(chan);

    channel_cccf_destroy(channel);
    printf("RX: %fms TX:%fms avg RX: %fms avg TX: %fms\n",elapsed_rx,elapsed_tx,elapsed_rx/iterations,elapsed_tx/iterations);

    char filename[40] = {0};
    sprintf(filename,"cfo_sync_test_%.0fHz_snr%.0f_gsmTU_chan.m",cfo_test,SNR);
    log_matlab_f(cfos,num_syncs,filename);
    return 0;
}

int main()
{
	//return main_biterr_test();
	return main_cfo_estimation_test();
	//return main_sync_cfo_estimation();


    // create channel
    channel_cccf channel = channel_cccf_create();

    // slow-flat fading
    float sigma = 1.0;	// standard deviation for log-normal shadowing
    float fd = 20.0/256000.0;	// relative Doppler frequency
    channel_cccf_add_shadowing(channel, sigma, fd);

    channel_cccf_print(channel);

    modem qam4 = modem_create(LIQUID_MODEM_QAM4);

    uint buf_len = 100000;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];

    for(int i=0; i<buf_len; i++) {
    	modem_modulate(qam4,modem_gen_rand_sym(qam4),buf_in+i);
    }
    channel_cccf_execute_block(channel, buf_in,buf_len,buf_out);

    log_matlab_fc(buf_out,buf_len,"test_slow_fading.m");

}
