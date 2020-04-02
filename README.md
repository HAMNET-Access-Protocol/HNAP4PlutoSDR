## Installation

For installation instructions, read the README in the main project.

### Build configuration

The Build configuration can be tweaked using C Makros.

Configuration regarding PHY layer can be done in `phy/phy_config.h`.
The MAC layer is configured in `mac/mac_config.h`. Some platform specific
configuration for the Pluto can be made in `platform/pluto.h`.

Some useful configuration:

- **Logs**: A global log-level can be defined in `util/log.h`.
- **PTT**:  PTT signal at port MIO0 can be generated. Set *GENERATE_PTT_SIGNAL*
            in `platform/pluto.h`. You can tune the toggle timing using *PTT_DELAY_ADJUST_US*.

## Demo / Usage

### Basestation

The basestation application will be located in the `/root/` directory of the pluto  
system. If the basestation firmware is configured to autostart the basesation,
it will start it with TX-gain 0 (~0dBm) and RX-gain set to 70.
You can manually specify the gain parameters by passing the -g and -t flag:

`./basestation -g <rxgain> -t <txgain>`

### Client

The client application and the calibration tool is located in `/root/`
directory. Below, the configuration of the client is described.

#### MCS

By default, MCS0 (QPSK, 1/2rate code) is used. You can manually set other
MCS values for uplink and downlink. Downlink MCS can be set with the `-d` or
`--dl-mcs` flag. Uplink MCS can be set with the `-u` or `--ul-mcs` flag.

The following MCS have been defined

| MCS value | modulation | conv-coding| Notes  |
|:----------|:-----------|:-----------|:-------|
|     0     |    QPSK    | k=7, r=1/2 | |
|     1     |    QPSK    | k=7, r=3/4 | |
|     2     |    QAM16   | k=7, r=1/2 | |
|     3     |    QAM16   | k=7, r=3/4 | |
|     4     |    QAM64   | k=7, r=1/2 | |
|     5     |    QAM64   | k=7, r=3/4 | |
|     6     |    QAM256  | k=7, r=1/2 | MCS5 gives higher data-rates|

#### Gain

By default the gain is automatically set during startup-phase and fixed  
during runtime of the client. However, the AGC might might set the gain  
too high so that clipping occurs. If so, the gain can be manually set with  
the parameters `-g` and `-t`.

The application `client-calib` can be used to read the current gain values  
of the application. They should not exceed 2000 to prevent clipping. The procedure
to use the manual gain mode looks as follows: first start the calib tool
with some rxgain value

`./client-calib -g <rxgain>`

Tweak the rxgain until the gain values look good.

>adapt xo by corr_factor=0  
>Real gain avg: 115, max: 2122  
>Imag gain avg: 115, max: 2146

The max value should be less than 2000.

Then, start the client. You also have to specify the txgain value. Since
the basestation operates at a txgain of 0 and rxgain of 70, you can calculate
the txgain of the client, assuming that Uplink and Downlink have the same channel
properties:

`txgain(client) = -70 + rxgain(client)`

Start the client:

`./client -g <rxgain> -t <txgain>`

#### Frequency offset calibration

The default pluto TCXO has a bad accuracy and might need some initial
calibration. The client and the client calib tool perform an initial
cfo estimation and retune the transceiver to the correct carrier frequency.

However, if the offset is too high, already the initial sync can fail. A simple method is to
recalibrate the XO in steps of 100Hz and test until the client finds sync.

The TCXO can be calibrated using the *fw_setenv* command:

`fw_setenv xo_correction <new frequency>`

The default frequency is 40Mhz, so try 39999800 39999900 etc.

### Tests

The plutos run a iperf3 server by default, in order to test the connection,
you can run `iperf3 -c 192.168.123.X` on another pluto.