# SturmFront

SturmFront is a DIY fan-controller for indoor sports, which adjusts fan speed according to your current heart rate.

Commercially available is the Wahoo® KICKR Headwind, which has an integrated fan and regulates fan speed according to your power output. Keith Wakeham developed an open-source DIY fan controller named [Maelstom](https://github.com/kwakeham/Maelstrom). Maelstrom also adjusts fan speed according to your power output.

I developed SturmFront independently. A project description is also available: [https://hannesweisbach.github.io/SturmFront/](https://hannesweisbach.github.io/SturmFront/)

## Software used

I used KiCAD as EDA-tool. The controller is based on the Nordic Semiconductor [NRF52832 SoC](https://www.nordicsemi.com/products/nrf52832). I used the [NRF5 SDK 17.1.0](https://www.nordicsemi.com/Products/Development-software/nrf5-sdk/download) in conjuction with the [S332 softdevice](https://www.nordicsemi.com/Products/Development-software/s332-ant) to provide ANT+ and BLE connectivity.
