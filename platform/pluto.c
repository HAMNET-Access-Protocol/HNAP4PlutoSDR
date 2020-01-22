/*
 * pluto.c
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */


#include "pluto.h"
#include <errno.h>
#include "../phy/phy_config.h"
#include <stdio.h>
#include <iio.h>

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define KHZ(x) ((long long)(x*1000.0 + .5))

#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

// Streaming devices
struct iio_device *tx;
struct iio_device *rx;

// Stream configurations
struct stream_cfg rxcfg;
struct stream_cfg txcfg;

/* cleanup and exit */
void shutdown()
{
	printf("* Destroying buffers\n");
	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	printf("* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}


/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	ASSERT(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(ctx, type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rfport);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(ctx, type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}

platform init_pluto_platform(uint buf_len)
{
	// RX stream config
	rxcfg.bw_hz = 1703632;   // Analog filter corner freq. Calculated by matlab filter tool
	rxcfg.fs_hz = KHZ(256*8);   // 8*256khz, decimation is done after this stage
	rxcfg.lo_hz = KHZ(430001); // 430Mhz rf frequency
	rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

	// TX stream config
	txcfg.bw_hz = 1701126; // Analog filter corner freq: 1.6*fs
	txcfg.fs_hz = KHZ(256*8);   // 2.5 MS/s tx sample rate
	txcfg.lo_hz = MHZ(430); // 430Mhz rf frequency
	txcfg.rfport = "A"; // port A (select for rf freq.)

	printf("* Acquiring IIO context\n");
	ASSERT((ctx = iio_create_local_context()) && "No context");
	ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

	printf("* Acquiring AD9361 streaming devices\n");
	ASSERT(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
	ASSERT(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

	//enable fir filters on both channel
	printf("* Enabling FIR filter on phy channels");
	struct iio_channel* chn;
	get_phy_chan(ctx, RX, 0, &chn);
	wr_ch_lli(chn, "filter_fir_en", 1);
	get_phy_chan(ctx, TX, 0, &chn);
	wr_ch_lli(chn, "filter_fir_en", 1);

	printf("* Configuring AD9361 for streaming\n");
	ASSERT(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");
	ASSERT(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");
	ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
	ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");
	ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found");
	ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found");

	// Enable dec/int stage of cf-ad9361-lpc / cf-ad9361-dds-core-lpc
	wr_ch_lli(rx0_i,"sampling_frequency",rxcfg.fs_hz/8);
	wr_ch_lli(tx0_i,"sampling_frequency",txcfg.fs_hz/8);

	printf("* Set TX gain\n");
    // Set TX gain
	get_phy_chan(ctx, TX, 0, &chn);
    wr_ch_lli(chn, "hardwaregain", 0.0);

    // Set RX AGC to slow attack
    get_phy_chan(ctx, RX, 0, &chn);
    wr_ch_str(chn, "gain_control_mode", "slow_attack");

	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);


	printf("* Creating non-cyclic IIO buffers\n");
	rxbuf = iio_device_create_buffer(rx, buf_len, false);
	if (!rxbuf) {
		perror("Could not create RX buffer");
		shutdown();
	}
	txbuf = iio_device_create_buffer(tx, buf_len, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		shutdown();
	}

	// Generate platform interface
	platform pluto = malloc(sizeof(struct platform_s));
	pluto->platform_rx = pluto_receive;
	pluto->platform_tx_prep = pluto_prep_tx;
	pluto->platform_tx_push = pluto_receive;
	pluto->end = shutdown;
	return pluto;
}

// Push samples to TX buffer object
// returns the number of samples that have been pushed
int pluto_prep_tx(platform hw, float complex* buf_tx, uint offset, uint num_samples)
{
	char *p_dat, *p_end, p, *p_start;
	ptrdiff_t p_inc;


	// WRITE: Get pointers to TX buf and write IQ to TX buf port 0
	p_inc = iio_buffer_step(txbuf);
	p_start = iio_buffer_first(txbuf, tx0_i) + offset*p_inc;
	p_end = iio_buffer_end(txbuf);
	uint i=0;
	for (p_dat = p_start; p_dat < p_end && i<num_samples; p_dat += p_inc) {
		// 12-bit sample needs to be MSB alligned
		// https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
		((int16_t*)p_dat)[0] = (int16_t)(8196.0*creal(buf_tx[i])); // Real (I)
		((int16_t*)p_dat)[1] = (int16_t)(8196.0*cimag(buf_tx[i++])); // Imag (Q)
	}
	return i;
}

// Flushes the TX buffer and transfers data to Kernel buffer
// so that samples will be sent
void pluto_transmit(platform hw)
{
	ssize_t nbytes_tx;
	// Schedule TX buffer
	nbytes_tx = iio_buffer_push(txbuf);
	if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); }
}


// Receive samples from Kernel buffer
// returns the number of received samples
int pluto_receive(platform hw, float complex* buf_rx)
{
	ssize_t nbytes_rx;
	char *p_dat, *p_start, *p_end;
	ptrdiff_t p_inc;

	// Refill RX buffer
	nbytes_rx = iio_buffer_refill(rxbuf);
	if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); }

	// READ: Get pointers to RX buf and read IQ from RX buf port 0
	p_start = iio_buffer_first(rxbuf, rx0_i);
	p_inc = iio_buffer_step(rxbuf);
	p_end = iio_buffer_end(rxbuf);

	uint i=0;
	for (p_dat = p_start; p_dat < p_end; p_dat += p_inc) {
		buf_rx[i++] = ((int16_t*)p_dat)[0]/2048.0 + I*((int16_t*)p_dat)[1]/2048.0;
	}
	return i;
}
