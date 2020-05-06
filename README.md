## Installation

For installation instructions, read the README in the main project.

### Build configuration

Most of the configuration is now possible at runtime by editing a config file. For 
configuration that is not possible via the files, this section applies. 

The Build configuration can be tweaked using C Makros.

Configuration regarding PHY layer can be done in `phy/phy_config.h`.
The MAC layer is configured in `mac/mac_config.h`. Some platform specific
configuration for the Pluto can be made in `platform/pluto.h`.

## Demo / Usage

### Runtime configuration

Both basestation and client may be configured via a config.txt file. We use library 
[libconfig](https://hyperrealm.github.io/libconfig/) and its syntax for the config file.
The configuration is split into currently three parts: PHY layer, Platform and log configuration.
The default configuration is given in config.txt in this repo.

The configuration file can be used by passing the `-c <config-file>` parameter to the applications.


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
|     5     |    QAM64   | k=7, r=3/4 | currently unstable. sometimes not working|
|     6     |    QAM256  | k=7, r=1/2 | MCS5 gives higher data-rates|

#### Gain

By default the rxgain is automatically set during startup-phase and adapted  
during runtime of the client. If you want to manually fix the gain, use the `-g` flag.
The gain can be set in the range of [0 73].
 
The application `client-calib` can be used to read the current signal level  
of the application. It prints the absolute amplitudes of the I and Q path and the calculated
rssi. RXgain should be set to keep the RSSI at ~-15dB.

`./client-calib -g <rxgain>`

The txgain of the client is automatically set, assuming a symmetrical UL and DL link.
It is calculated from RX and TX gain broadcasts that the basestation sends and the
rxgain that the client calculated:

`txgain_client = bs_txgain + rxgain_client - bs_rxgain`

In the current implementation, neither the rxgain of the basestation nor the
txgain of a client are adaptive. This feature still has to be implemented.

#### Frequency offset calibration

The default pluto TCXO has a bad accuracy and might need some initial
calibration. The client and the client calib tool perform an initial
cfo estimation and retune the transceiver to the correct carrier frequency.

However, the offset estimation only works in a range of +-2Khz, the default TCXO
 might give larger offsets. A simple method is to
recalibrate the XO in steps of 100Hz and test until the client finds sync.

The TCXO can be calibrated using the *fw_setenv* command:

`fw_setenv xo_correction <new frequency>`

The default frequency is 40Mhz, so try 39999800 39999900 etc.

Instead of tweaking the TCXO, you can also specify the frequency of the client with the
`-f` flag. The default DL carrier is located at 439.7 MHz, try frequencies nearby if your client
cannot get sync.


**NOTE:** in the first minutes after the pluto started, the frequency offset varies by somtimes
multiple Khz. Let the Pluto run and heat up for some minutes, if you cannot find a constant
offset. The offset will settle after a while.

### Tests

The plutos run a iperf3 server by default. In order to test the connection,
you can run `iperf3 -c 192.168.123.X` on another pluto.

Note that networking currently only works between the Plutos TAP devices, i.e.
within Network 192.168.123.0/24.
No routes have been configured.