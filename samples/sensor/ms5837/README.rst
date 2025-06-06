.. zephyr:code-sample:: ms5837
   :name: MS5837 Digital Pressure Sensor
   :relevant-api: sensor_interface

   Get pressure and temperature data from an MS5837 sensor (polling mode).

Overview
********

This sample application retrieves the pressure and temperature from a MS5837
sensor every 10 seconds, and prints this information to the UART console.

Requirements
************

- `nRF52840 Development kit`_
- MS5837 sensor

Wiring
******

The nrf52840 Development kit should be connected as follows to the
MS5837 sensor.

+-------------+----------+
| | nrf52840  | | MS5837 |
| | Pin       | | Pin    |
+=============+==========+
| P0.3        | SCL      |
+-------------+----------+
| P0.31       | SDA      |
+-------------+----------+

Building and Running
********************

Build this sample using the following commands:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ms5837
   :board: nrf52840dk/nrf52840
   :goals: build
   :compact:

See :zephyr:board:`nrf52840dk` on how to flash the build.

References
**********

.. target-notes::

.. _MS5837 Sensor: http://www.te.com/usa-en/product-CAT-BLPS0017.html?q=&type=products&samples=N&q2=ms5837
.. _nRF52840 Development kit: https://www.nordicsemi.com/Products/Development-hardware/nRF52840-DK
