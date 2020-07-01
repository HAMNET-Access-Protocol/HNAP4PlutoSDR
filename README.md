## HNAP4PlutoSDR

This is the implementation of the HAMNET Access Protocol for 
Adalm Pluto SDRs. It uses a 200 kHz duplex channel in the 70cm band to 
transmit IP traffic.

Main Features:
- OFDM based system with 40 subcarriers (4kHz subcarrier spacing)
- DL @ 439.7MHz / UL @ 434.9MHz (can be reconfigured)
- Basestation - Client topology, supports up to 14 connected clients
- up to 400kbps data-rate and <150ms RTT at the application layer

## Quickstart

Visit [manual.hnap.de](https://manual.hnap.de/) for an installation and usage guide

## Notes

This repository uses [pre-commit](https://pre-commit.com/) hooks.
Please make sure you have installed pre-commit (`pip install pre-commit`) and
set up the git hooks with `pre-commit install` after cloning the repository.
The linting hook uses clang-format for linting, install it with `apt install clang-format`.
