
Rev 0.2 Changes

Schematic
- fixed Q2 (swapped source & drain)
- added P_JUMP to bypass pushbutton SW2
- fixed voltage regulator U1 (swapped in and out)
- added USR_SW
- added BOOT0 connection to PB8

PCB
- corresponding changes from schematic changes
- moved BATT_BALANCE connector for 5 volt regulator screw clearance
- increased size of 5 volt regulator pin holes
- changed C3 to be a slightly larger cap (old one wasn't rated for 12 volts)
- fixed the silkscreen for the Bioloid AX-12 connector

Pins
- GATE_PIN (A14) - Bring GATE_PIN high to keep power going to the MCU, bring GATE_PIN
    low to turn the MCU off. Pressing the PowerSwitch acts as a bootstrap
    to turn power on, and it must remain pressed until the MCU brings
    GATE_PIN high
- SWITCH_PIN (A13) - Allows the MCU to detect when SW2 has been pressed. This is
    typically used as a way of indicating that a shudown should be
    initiated. However, you could also discern between long press and
    short press to perform different things.
- GPIO_A4, GPIO_A5, GPIO_A6, GPIO_A7 - These are 3 pins headers with power,
    ground and signal available. The power signal can be switched between
    3.3v and 5v by using a solder ball. By default power is connected to 3.3v
    but the trace can be cut using an X-Acto knife.
    The signal lines can be configured as digital lines or as ADC lines. As
    digital lines they are 3.3v but 5v tolerant. As ADC, they are limited to
    3.3v
- STAT_PIN (A15) - low when wall adapter is present.
- LED_R (B0/TIM3_CH3), LED_G (B1/TIM3_CH4), LEB_B (B5/TIM3_CH2) - used to
    control the RGB status LED. All 3 LEDs are also connected to TIM3 so you
    can use PWM to control the brightness.
- CELL1_VOLTAGE (A0), CELL2_VOLTAGE (A1), CELL3_VOLTAGE (A3) - connected to
    the battery balance connector. Each signal will read 25% of the real voltage.
    The voltage of each cell can be inferred by subtracting the voltage of the
    earlier cell. This can be used to detect batteries which are not properly
    balanced.
- BIOLOID_TX (A2) - Connects to the bioloid bus.
- VBUS (PA9) - Can be used to detect when the USB cable is plugged in.
- UART1_TX (B6), UART1_RX (B7) - UART port 1. Can be used for debugging or
    connecting a serial based sensor. Can also be used to bootload code.
- I2C2_SDA (B3), I2C_SCL (B10) - I2C bus 2. Can be used to connect additional
    peripherals/sensors.
- SPI2_CS1 (B12), SPI_CS2 (A8), SPI2_MOSI (B15), SPI2_MISO (B14), SPI2_SCK (B13)
    SPI port available for external peripherals/sensors.
- USR_SW (C13) - can be used as a digital I/O pin.
- P_JUMP - can be used to force power on until such time as the MCU does this itself.
- ALT_3.3 - can be used to power the MCU if the 5V regulator is unplugged.
- RPI - 5V power used to provide power to the host processor (RPi)
- BUS_SWITCH - can be connected to a switch to control Bioloid Bus Power.

