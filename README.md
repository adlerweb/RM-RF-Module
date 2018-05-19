@@ -0,0 +1,41 @@
# RM-RF-Module

This system can be used to add RF functionality to ordinary photoelectric smoke detectors based on the widespread MC145010 IC or its derivatives.

The module uses RF24 2.4GHz transceivers and communicates with a central receiver. In addition to relaying the smoke alarm it also features a DS18B20 temperature sensor and will report high temperature conditions. Lastly I'll - even if no alarm is active - report battery voltage and temperature back to the central receiver in regular intervals to allow for statistics and detect broken detectors.

Power can be supplied using the original 9V battery or a protected 7.4V lithium battery pack.

Real time communication or mesh functionality is not implemented to keep power consumption down. It should be possible to use nodes as relays using this concept, but nothing has been implemented so far.

Please consider this project as WIP - if you're looking for a system that "just works" you're probably better off with a commercial system.
More details in German can be found in the design and build vlog over at HTTPS://www.youtube.com/user/adlerweb/search?query=rauchmelder

# Warning

Modifying detectors will void their approval. If you're required to install smoke detectors modified modules are not sufficient to pass the requirements. Use at your own risk.

# Repo

## Hardware

The module is based on a cheap ATTiny88 as central processor and an NRF24L01+ for radio communication. The 3.3V system power uses a NCP551 featuring a rather low quiescent current for it's price to keep consumption down. Peripherals like the radio module or an optional DS18B20 are only powered up when accessed.

The original smoke alarm uses the IO-functionality of the MC145010 detector IC. Additionally the onboard LED and (optionally) the switch are interfaced. Since the board is based on an older design with full separation the interface uses optocouplers between detector and this PCB. There is however no galvanic isolation. It should be possible to switch to a transistor based approach and shrink the PCB if desired.

## Software

### Basic function

All system timing is based on the regular LED flash of the original smoke detector IC. In my case it blinks every 20 seconds.

When booting for the first time all used GPIOs will be prepared and a boot message transmitted to the master.

Now the IO pin is checked for an alarm condition, if there is one an alarm message with type 1 will be transmitted.
Next - if the configured number of LED flashes for temperature is reached - the temperature sensor will be powered up and checked. If the measured temperature is above a defined alarm level an alarm message with type 2 will be transmitted
Now - if the configured number of LED flashes for the alive-message is reached - it will power up the temperature sensor and ADC, measure both and send the results to the master
Lastly it ensures the detectors LED is turned off, rearms the LED-flash-interrupt and enters power-down-mode.

### RF-Notes

NRF24L01+ only supports listening to 6 FIFOs at a time. While it is possible to target a single FIFO from multiple senders there are reports this would mess up the auto-ACK-feature in some cases and as such provide only unreliable communication. The code uses an additional checksum and acknowledgement layer to ensure proper transmission. Inside the ACK the master can piggyback a 2-byte-value to send commands to the smoke detector.