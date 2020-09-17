# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - Unreleased

### Added

* Version info added to executables
* Transmit firmware and HNAP version with LLDP (#5) (#30)

### Changed

* Enhanced Idle mode (#20) *Necessary for operation with RX/TX switch!*
* Fix memory leaks in PHY layer when creating sync_info slot (#36)

### Removed

* none

## 1.0.0 - 2002-06-18

### Added

* Callsigns are now automatically broadcasted every 60sec using LLDP

[Unreleased]: https://github.com/HAMNET-Access-Protocol/HNAP4PlutoSDR/compare/v2.0.0...develop
[2.0.0]: https://github.com/HAMNET-Access-Protocol/HNAP4PlutoSDR/compare/v1.0.0...v2.0.0
