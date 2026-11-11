This kernel module enables access to FPGA devices on IMAGO's hardware. It is used by the userspace library *libAGEXHAL.so*.

The driver implements PCI, PCIe, SPI and USB interface for FPGA communication. The following platforms are supported:
- VisionBox AGE-X, AGE-X2, AGE-X4, AGE-X5, AGE-X6
- VisionBox Daytona
- VisionBox Le Mans
- VisionBox AI
- VisionCam XM, XM2
- VisionSensor PV3, PV4
- Machine Vision Controller (PCIe card)
- Machine Vision Manager 2 (USB device)

The module code is automatically analyzed by using *sparse* and *smatch*.

Current status:
[![Analyze kernel module](https://github.com/IMAGO-Technologies/imago_fpga/actions/workflows/analyze.yml/badge.svg)](https://github.com/IMAGO-Technologies/imago_fpga/actions/workflows/analyze.yml)
