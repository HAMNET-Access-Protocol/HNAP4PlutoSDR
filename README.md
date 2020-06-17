## HNAP4PlutoSDR

This is the implementation of the HAMNET Access Protocol for 
Adalm Pluto SDRs. It uses a 200 kHz duplex channel in the 70cm band to 
transmit IP traffic.

Key Facts
- OFDM based system with 40 subcarriers (4kHz subcarrier spacing)
- Basestation - Client topology, supports up to 14 connected clients
- up to 400kbps data-rate and <150ms RTT at the application layer

## Quickstart

### Installation
Download the HNAP4PlutoSDR firmware. Connect a PlutoSDR to your PC and copy the 
image to the Pluto's mass storage device. Eject the Pluto and wait until
the flashing process is completed (LED1 stops blinking).

Now, connect to the Pluto via SSH to initially configure the system.
Set the following variables:

```shell script
# Activate 2nd CPU, isolate core 1
fw_setenv maxcpus 2 isolcpus=1
# IP address of this Pluto within the HNAP Network 
# (you can define any IP address as long as they are within one subnet)
fw_setenv hnap_tap_ip 192.168.123.1

# If this Pluto will act as a basestation, set this variable
# To enable autostart of the basesation
fw_setenv hnap_bs_autostart 1
```
Reboot the Adalm Pluto.

### First tests

You are going to need 2 Adalm Plutos to do first transmission tests.
When connecting both Plutos to the same Host PC, make sure that the ethernet
connection to both Plutos uses different IP addresses. See [here](https://wiki.analog.com/university/tools/pluto/users/customizing)
for more info. We suggest that you configure your first Pluto with the
IP addresses 192.168.2.1 (pluto) 192.168.2.10 (host PC) and the second with 
IP address 192.168.**3**.1 (pluto) and 192.168.**3**.10 (host PC). Also make sure
that you have configured different *hnap_tap_ip*s in the previous step.

Once this is done, SSH to the client Pluto and start the client application
```shell script
./client
```

The client should start up and connect to the basestation. Once this is done,
open another ssh connection and test the HNAP connection with *ping* or *iperf*.

Every Pluto starts an iperf3 server by default, so simply run:
```shell script
iperf3 -c 192.168.123.1 -R
```

### Troubleshooting

If the client is not able to connect to the basestation, the following might help:

**Make sure the second CPU core is active**

Verify it with `fw_printenv maxcpus`. Without the second core, no realtime operation
is possible.

**Use correct antennas for the used band**

The Pluto does not come with any analog filters after/before the mixing stage.
We therefore produce strong intermodulation products (e.g. output with -10dBc @1.31GHz).
These products will be mixed down to the baseband at the receiver and interfere with our signal. 
Use UHF antennas in order to suppress the intermodulation products. DO NOT use the original 2.4GHz antennas! They just
recept the intermodulation products, but not the original signal at 440MHz.

**Adjust the carrier frequency**

The default TCXO on the Adalm Pluto has an accuracy of 25ppm. At 440MHz, this can result
in a frequency offset of up to 10kHz. The application can only detect and compensate +-2kHz, we therefore
strongly suggest to switch to a more accurate TCXO (more details here TODO!).

If you do not want to swap the TCXO immediately, try the following:
1. Let the Pluto heat up a bit. The frequency drift is very high in the first minutes of operation. Wait some minutes for the offset to settle.
2. Adjust the frequency offset manually. Run `./client -f 439702000` to tune to higher and lower carrier frequencies. Do this in 2kHz steps 
around the default frequency of 439700000. There is a calibration tool *client-calib* that only syncs to the Downlink and 
estimates the carrier offset that might help: `./client-calib -f <DL-FREQ>`

