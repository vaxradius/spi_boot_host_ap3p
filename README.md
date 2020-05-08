# spi_boot_host

python create_cust_wireupdate_blob.py --load-address 0xC000 --bin binary_counter.bin -i 6 -o straight_up --options 0x0
put straight_up.bin in \spi_boot_host\src

//! PIN fly lead connections assumed:
//!     HOST (this board)                       SLAVE (Apollo3 SBL target)
//!     --------                                --------
//! Apollo3 SPI or I2C common connections:
//!     GPIO[40]  GPIO Interrupt (slave to host) GPIO[4]  GPIO interrupt
//!     GPIO[25]  OVERRIDE pin   (host to slave) GPIO[16] Override pin or n/c
//!     GPIO[27] Slave reset (host to slave)    Reset Pin or n/c
//!     GND                                     GND
//!
//! Apollo3 SPI additional connections:
//!     GPIO[5]  IOM0 SPI CLK                   GPIO[0]  IOS SPI SCK
//!     GPIO[6]  IOM0 SPI MISO                  GPIO[2]  IOS SPI MISO
//!     GPIO[7]  IOM0 SPI MOSI                  GPIO[1]  IOS SPI MOSI
//!     GPIO[11] IOM0 SPI nCE                   GPIO[3]  IOS SPI nCE
