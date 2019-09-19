# nrf52811-radio-long-range-demo

Example how to use the nRF52811 for long range demo

In this version of example,

## The FW size of peripheral and central are around 18KB (optimization with size, without debugging information, without RTT log).
It means that the remainder size for the application is around 192KB - 156KB - 20KB = ~16KB.

## Requirement

* nRF52840 DK x 2 (acts nRF52811 feature PCA10056e)
* SDK 15.3 Example (NUS)
* Softdevice S140v7.0.1 -- qualified with nRF52811 (156KB flash)
* Segger Embedded Studio (4.22)

** This example shows the RSSI with Packet Success Rate at the Central UART terminal after connection.

All details can be found at [URL: https://jimmywongbluetooth.wordpress.com/2019/09/18/nrf52811-with-ble-long-range-example/](https://jimmywongbluetooth.wordpress.com/2019/09/18/nrf52811-with-ble-long-range-example/).

