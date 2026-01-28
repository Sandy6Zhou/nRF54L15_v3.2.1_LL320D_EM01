
After programming the sample to your development kit, complete the following steps to test the basic functionality:

.. tabs::

   .. group-tab:: nRF21, nRF52 and nRF53 DKs

      1. Connect the device to the computer to access UART 0.
         If you use a development kit, UART 0 is forwarded as a serial port.
         |serial_port_number_list|
         If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter to it.
      #. |connect_terminal|
      #. Reset the kit.
      #. Observe that **LED 1** is blinking and the device is advertising under the default name **Nordic_UART_Service**.
         You can configure this name using the :kconfig:option:`CONFIG_BT_DEVICE_NAME` Kconfig option.
      #. Observe that the text "Starting Nordic UART service sample" is printed on the COM listener running on the computer.

   .. group-tab:: nRF54 DKs

      .. note::
          |nrf54_buttons_leds_numbering|

      1. Connect the device to the computer to access UART 0.
         If you use a development kit, UART 0 is forwarded as a serial port.
         |serial_port_number_list|
         If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
      #. |connect_terminal|
      #. Reset the kit.
      #. Observe that **LED 0** is blinking and the device is advertising under the default name **Nordic_UART_Service**.
         You can configure this name using the :kconfig:option:`CONFIG_BT_DEVICE_NAME` Kconfig option.
      #. Observe that the text "Starting Nordic UART service sample" is printed on the COM listener running on the computer.

.. _peripheral_uart_testing_mobile:

Testing with nRF Connect for Mobile
-----------------------------------

You can test the sample pairing with a mobile device.
For this purpose, use `nRF Connect for Mobile`_ (or other similar applications, such as `nRF Blinky`_ or `nRF Toolbox`_).

To perform the test, complete the following steps:

.. tabs::

   .. group-tab:: nRF21, nRF52 and nRF53 DKs

      .. tabs::

         .. group-tab:: Android

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your Android device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 2** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 1** on the device and accept the passkey value on the smartphone.
            #. In the application, observe that the services are shown in the connected device.
            #. Select **Nordic UART Service** and tap the up arrow button for the **UART RX characteristic**.
               A pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789" and tap :guilabel:`SEND`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the upper right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your Android device, tap the three-dot menu next to **Disconnect** and select **Show log**.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 2** turns off.

         .. group-tab:: iOS

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your iOS device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 2** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 1** on the device and accept the passkey value on the smartphone.
            #. In the application, change to **Client** tab and observe that the services are shown in the connected device.
            #. In **Nordic UART Service**, tap the up arrow button for the **UART RX characteristic**.
               A **Write Value** pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789", select "UTF8" input type and tap :guilabel:`Write`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the lower right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your iOS device, select the **Log** tab.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 2** turns off.

   .. group-tab:: nRF54 DKs

      .. note::
          |nrf54_buttons_leds_numbering|

      .. tabs::

         .. group-tab:: Android

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your Android device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 1** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 0** on the device and accept the passkey value on the smartphone.
            #. In the application, observe that the services are shown in the connected device.
            #. Select **Nordic UART Service** and tap the up arrow button for the **UART RX characteristic**.
               A pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789" and tap :guilabel:`SEND`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the upper right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your Android device, tap the three-dot menu next to **Disconnect** and select **Show log**.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 1** turns off.

         .. group-tab:: iOS

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your iOS device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 1** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 0** on the device and accept the passkey value on the smartphone.
            #. In the application, change to **Client** tab and observe that the services are shown in the connected device.
            #. In **Nordic UART Service**, tap the up arrow button for the **UART RX characteristic**.
               A **Write Value** pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789", select "UTF8" input type and tap :guilabel:`Write`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the lower right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your iOS device, select the **Log** tab.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 1** turns off.

.. _nrf52_computer_testing:
.. _peripheral_uart_testing_ble:

Testing with Bluetooth Low Energy app
-------------------------------------

If you have an nRF52 Series DK with the Peripheral UART sample and either a dongle or second Nordic Semiconductor development kit that supports the `Bluetooth Low Energy app`_, you can test the sample on your computer.
Use the `Bluetooth Low Energy app`_ in `nRF Connect for Desktop`_ for testing.

To perform the test, complete the following steps:

1. Install the `Bluetooth Low Energy app`_ in `nRF Connect for Desktop`_.
#. Connect to your nRF52 Series DK.
#. Connect the dongle or second development kit to a USB port of your computer.
#. Open the app.
#. Select the serial port that corresponds to the dongle or the second development kit.
   Do not select the kit you want to test just yet.

   .. note::
      If the dongle or the second development kit has not been used with the Bluetooth Low Energy app before, you may be asked to update the J-Link firmware and connectivity firmware on the nRF SoC to continue.
      When the nRF SoC has been updated with the correct firmware, the app finishes connecting to your device over USB.
      When the connection is established, the device appears in the main view.

#. Click :guilabel:`Start scan`.
#. Find the development kit you want to test and click the corresponding :guilabel:`Connect` button.

   The default name for the Peripheral UART sample is *Nordic_UART_Service*.

#. Select the **Universal Asynchronous Receiver/Transmitter (UART)** RX characteristic value.
#. Write ``30 31 32 33 34 35 36 37 38 39`` (the hexadecimal value for the string "0123456789") and click :guilabel:`Write`.

   The data is transmitted over Bluetooth LE from the app to the DK that runs the Peripheral UART sample.
   The terminal emulator connected to the development kit then displays ``"0123456789"``.

#. In the terminal emulator, enter any text, for example ``Hello``.

   The data is transmitted to the development kit that runs the Peripheral UART sample.
   The **UART TX** characteristic displayed in the app changes to the corresponding ASCII value.
   For example, the value for ``Hello`` is ``48 65 6C 6C 6F``.

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`lib_uart_async_adapter`
* :ref:`nus_service_readme`
* :ref:`dk_buttons_and_leds_readme`

In addition, it uses the following Zephyr libraries:

* :file:`include/zephyr/types.h`
* :file:`boards/arm/nrf*/board.h`
* :ref:`zephyr:kernel_api`:

  * :file:`include/kernel.h`

* :ref:`zephyr:api_peripherals`:

   * :file:`include/gpio.h`
   * :file:`include/uart.h`

* :ref:`zephyr:bluetooth_api`:

  * :file:`include/bluetooth/bluetooth.h`
  * :file:`include/bluetooth/gatt.h`
  * :file:`include/bluetooth/hci.h`
  * :file:`include/bluetooth/uuid.h`

The sample also uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`

LL311_BLE: nRF54L15 蓝牙透传与多功能模块
#########################################

项目概述
********

LL311_BLE 是基于 Nordic nRF54L15 SoC 开发的多功能蓝牙应用项目。
该项目在原生 Peripheral UART (NUS) 示例的基础上进行了深度定制，采用了模块化架构，支持以下核心功能：

1. **蓝牙透传 (NUS)**：实现手机端（nRF Connect）与硬件串口（UART20）之间的双向透传。
2. **模块化架构**：包含 BLE、Shell、LTE、G-Sensor、NFC 和系统控制模块。
3. **消息分发系统**：各模块通过自定义消息队列（MSG_S）进行异步通信。
4. **低功耗设计**：针对 nRF54L 系列优化的电源管理逻辑。

硬件环境
********

* **开发板**：nRF54L15 DK (PCA10156)
* **调试接口**：RTT (默认日志输出)
* **外部串口**：UART20 (引脚: TX:P1.04, RX:P1.05)
  * *注：由于引脚冲突，已禁用 I2C21 和 I2C22。*

模块职责
********

* **my_main**: 系统入口，负责各模块的初始化顺序管理及主消息循环。
* **my_ble_core**: 蓝牙协议栈管理、NUS 服务实现及多广播（Beacon）管理。
* **my_shell**: 串口命令行交互逻辑，及 UART 与 BLE 之间的数据 FIFO 转发。
* **my_ctrl**: 硬件外设控制（LED、按键、蜂鸣器）。
* **my_lte**: LTE 模块驱动与通信接口。
* **my_gsensor**: 加速度传感器（LIS2DH12）数据采集。
* **my_nfc**: NFC 读写器（ST25R3911）管理（当前保留）。

用户界面
********

* **LED 1 (DK_LED1)**：系统心跳灯，程序运行时闪烁。
* **LED 2 (DK_LED2)**：蓝牙连接状态灯，连接时常亮。
* **蜂鸣器**：系统启动或关键事件时提供声音反馈。
* **按键 1/2**：用于蓝牙配对过程中的数值比较确认（需开启安全配置）。

构建与运行
**********

使用 nRF Connect SDK (NCS) 环境进行构建：

.. code-block:: console

   west build -b nrf54l15dk/nrf54l15/cpuapp

烧录固件：

.. code-block:: console

   west flash

调试日志
********

本工程默认通过 **RTT** 输出调试日志。请使用 Segger RTT Viewer 或 `nrfutil device terminal` 观察输出。
日志模块名为 `peripheral_uart` (Main), `my_ble_core`, `my_shell` 等。

注意事项
********

* **引脚冲突**：本项目使用的 UART20 引脚与 DK 默认的 I2C 引脚存在冲突，修改 `nrf54l15dk_nrf54l15_cpuapp.overlay` 时需格外注意。
* **消息长度**：`MSG_S` 结构体的大小需在各模块中保持同步。

=======

After programming the sample to your development kit, complete the following steps to test the basic functionality:

.. tabs::

   .. group-tab:: nRF21, nRF52 and nRF53 DKs

      1. Connect the device to the computer to access UART 0.
         If you use a development kit, UART 0 is forwarded as a serial port.
         |serial_port_number_list|
         If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter to it.
      #. |connect_terminal|
      #. Reset the kit.
      #. Observe that **LED 1** is blinking and the device is advertising under the default name **Nordic_UART_Service**.
         You can configure this name using the :kconfig:option:`CONFIG_BT_DEVICE_NAME` Kconfig option.
      #. Observe that the text "Starting Nordic UART service sample" is printed on the COM listener running on the computer.

   .. group-tab:: nRF54 DKs

      .. note::
          |nrf54_buttons_leds_numbering|

      1. Connect the device to the computer to access UART 0.
         If you use a development kit, UART 0 is forwarded as a serial port.
         |serial_port_number_list|
         If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
      #. |connect_terminal|
      #. Reset the kit.
      #. Observe that **LED 0** is blinking and the device is advertising under the default name **Nordic_UART_Service**.
         You can configure this name using the :kconfig:option:`CONFIG_BT_DEVICE_NAME` Kconfig option.
      #. Observe that the text "Starting Nordic UART service sample" is printed on the COM listener running on the computer.

.. _peripheral_uart_testing_mobile:

Testing with nRF Connect for Mobile
-----------------------------------

You can test the sample pairing with a mobile device.
For this purpose, use `nRF Connect for Mobile`_ (or other similar applications, such as `nRF Blinky`_ or `nRF Toolbox`_).

To perform the test, complete the following steps:

.. tabs::

   .. group-tab:: nRF21, nRF52 and nRF53 DKs

      .. tabs::

         .. group-tab:: Android

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your Android device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 2** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 1** on the device and accept the passkey value on the smartphone.
            #. In the application, observe that the services are shown in the connected device.
            #. Select **Nordic UART Service** and tap the up arrow button for the **UART RX characteristic**.
               A pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789" and tap :guilabel:`SEND`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the upper right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your Android device, tap the three-dot menu next to **Disconnect** and select **Show log**.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 2** turns off.

         .. group-tab:: iOS

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your iOS device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 2** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 1** on the device and accept the passkey value on the smartphone.
            #. In the application, change to **Client** tab and observe that the services are shown in the connected device.
            #. In **Nordic UART Service**, tap the up arrow button for the **UART RX characteristic**.
               A **Write Value** pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789", select "UTF8" input type and tap :guilabel:`Write`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the lower right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your iOS device, select the **Log** tab.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 2** turns off.

   .. group-tab:: nRF54 DKs

      .. note::
          |nrf54_buttons_leds_numbering|

      .. tabs::

         .. group-tab:: Android

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your Android device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 1** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 0** on the device and accept the passkey value on the smartphone.
            #. In the application, observe that the services are shown in the connected device.
            #. Select **Nordic UART Service** and tap the up arrow button for the **UART RX characteristic**.
               A pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789" and tap :guilabel:`SEND`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the upper right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_android.png
                  :scale: 50 %
                  :alt: Screenshot of Android nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your Android device, tap the three-dot menu next to **Disconnect** and select **Show log**.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 1** turns off.

         .. group-tab:: iOS

            1. Connect the device to the computer to access UART 0.
               If you use a development kit, UART 0 is forwarded as a serial port.
               |serial_port_number_list|
               If you use Thingy:53, you must attach the debug board and connect an external USB to UART converter.
            #. |connect_terminal|
            #. Optionally, you can display debug messages.
               See :ref:`peripheral_uart_debug` for details.
            #. Install and start the `nRF Connect for Mobile`_ application on your iOS device.
            #. If the application does not automatically start scanning, tap the Play icon in the upper right corner.
            #. Connect to the device using nRF Connect for Mobile.

               Observe that **LED 1** is lit.
            #. Optionally, pair or bond with the device with MITM protection.
               This requires using the passkey value displayed in debug messages.

               See :ref:`peripheral_uart_configuration_options` for details on how to enable the MITM protection.

               See :ref:`peripheral_uart_debug` for details on how to access debug messages containing passkey.

               To confirm pairing or bonding, press **Button 0** on the device and accept the passkey value on the smartphone.
            #. In the application, change to **Client** tab and observe that the services are shown in the connected device.
            #. In **Nordic UART Service**, tap the up arrow button for the **UART RX characteristic**.
               A **Write Value** pop-up window with a text input field appears.
               You can write to the UART RX and get the text displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_button_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic up arrow button
            #. Type "0123456789", select "UTF8" input type and tap :guilabel:`Write`.

               Verify that the text "0123456789" is displayed on the COM listener.

               .. figure:: /images/bt_peripheral_uart_rx_popup_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART RX characteristic write value popup window
            #. To send data from the device to your phone or tablet, in the terminal emulator connected to the sample, enter any text, for example, "Hello", and press Enter to see it on the COM listener.

               The text is sent through the development kit to your mobile device over a Bluetooth LE link.
               It appears in the **Value** field of **UART TX characteristic**.

               If the text does not appear, check if notifications are enabled for this characteristic.
               You can toggle the notification settings with a button in the lower right corner of **UART TX Characteristic**.

               .. figure:: /images/bt_peripheral_uart_tx_ios.png
                  :scale: 50 %
                  :alt: Screenshot of iOS nRF Connect, showing UART TX characteristic with notification toggle and received text
            #. On your iOS device, select the **Log** tab.

               The device displays the text in the nRF Connect for Mobile log.
            #. Disconnect the device in nRF Connect for Mobile.

               Observe that **LED 1** turns off.

.. _nrf52_computer_testing:
.. _peripheral_uart_testing_ble:

Testing with Bluetooth Low Energy app
-------------------------------------

If you have an nRF52 Series DK with the Peripheral UART sample and either a dongle or second Nordic Semiconductor development kit that supports the `Bluetooth Low Energy app`_, you can test the sample on your computer.
Use the `Bluetooth Low Energy app`_ in `nRF Connect for Desktop`_ for testing.

To perform the test, complete the following steps:

1. Install the `Bluetooth Low Energy app`_ in `nRF Connect for Desktop`_.
#. Connect to your nRF52 Series DK.
#. Connect the dongle or second development kit to a USB port of your computer.
#. Open the app.
#. Select the serial port that corresponds to the dongle or the second development kit.
   Do not select the kit you want to test just yet.

   .. note::
      If the dongle or the second development kit has not been used with the Bluetooth Low Energy app before, you may be asked to update the J-Link firmware and connectivity firmware on the nRF SoC to continue.
      When the nRF SoC has been updated with the correct firmware, the app finishes connecting to your device over USB.
      When the connection is established, the device appears in the main view.

#. Click :guilabel:`Start scan`.
#. Find the development kit you want to test and click the corresponding :guilabel:`Connect` button.

   The default name for the Peripheral UART sample is *Nordic_UART_Service*.

#. Select the **Universal Asynchronous Receiver/Transmitter (UART)** RX characteristic value.
#. Write ``30 31 32 33 34 35 36 37 38 39`` (the hexadecimal value for the string "0123456789") and click :guilabel:`Write`.

   The data is transmitted over Bluetooth LE from the app to the DK that runs the Peripheral UART sample.
   The terminal emulator connected to the development kit then displays ``"0123456789"``.

#. In the terminal emulator, enter any text, for example ``Hello``.

   The data is transmitted to the development kit that runs the Peripheral UART sample.
   The **UART TX** characteristic displayed in the app changes to the corresponding ASCII value.
   For example, the value for ``Hello`` is ``48 65 6C 6C 6F``.

Dependencies
************

This sample uses the following |NCS| libraries:

* :ref:`lib_uart_async_adapter`
* :ref:`nus_service_readme`
* :ref:`dk_buttons_and_leds_readme`

In addition, it uses the following Zephyr libraries:

* :file:`include/zephyr/types.h`
* :file:`boards/arm/nrf*/board.h`
* :ref:`zephyr:kernel_api`:

  * :file:`include/kernel.h`

* :ref:`zephyr:api_peripherals`:

   * :file:`include/gpio.h`
   * :file:`include/uart.h`

* :ref:`zephyr:bluetooth_api`:

  * :file:`include/bluetooth/bluetooth.h`
  * :file:`include/bluetooth/gatt.h`
  * :file:`include/bluetooth/hci.h`
  * :file:`include/bluetooth/uuid.h`

The sample also uses the following secure firmware component:

* :ref:`Trusted Firmware-M <ug_tfm>`
