# ESP32_TTN_LoRaWan

OLED I2C 

```
// the OLED used
U8X8_SSD1306_128X64_NONAME_HW_I2C display(/*rst*/ 16, /*scl*/ 15, /*sda*/ 4);
```

and RFM95 connect to ESP32

```
// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ LMIC_UNUSED_PIN}
};

```

and start SPI  

```
SPI.begin(5,19,27); 
```

