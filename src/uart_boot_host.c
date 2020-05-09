//*****************************************************************************
//
//! @file uart_boot_host.c
//!
//! @brief Converts UART Wired transfer commands to SPI for use with SBL SPI testing.
//!
//! Purpose: This example running on an intermediate board, along with the standard
//! uart_wired_update script running on host PC, can be used as a way to
//! communicate to Apollo3 SBL using SPI mode.
//!
//! Printing takes place over the ITM at 1M Baud.
//!
//! Additional Information:
//! @verbatim
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
//!
//! @endverbatim
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Configuration options
//
//*****************************************************************************

#define IOM_MODULE                      0
#define MAX_SPI_SIZE                    1023
#define MAX_IOS_LRAM_SIZE               120     // LRAM can only accept 120 bytes at a time.

//
// This definition assumes Host is running on same type of board as target.
// If that is not case, this definition needs to be adjusted to match the
// desired pin on target board
//
#if 1
#define TARGET_BOARD_OVERRIDE_PIN       AM_BSP_GPIO_BUTTON0
#else
#define TARGET_BOARD_OVERRIDE_PIN       16
#endif


//
// Slave interrupt pin is connected here
//
#define BOOTLOADER_HANDSHAKE_PIN        40

//
// This pin is connected to RESET pin of slave
//
#define DRIVE_SLAVE_RESET_PIN           27

//
// This pin is connected to the 'Override' pin of slave
//
#define DRIVE_SLAVE_OVERRIDE_PIN        25

#define IOSOFFSET_WRITE_CMD             0x80
#define IOSOFFSET_READ_FIFO             0x7F
#define IOSOFFSET_READ_FIFOCTR          0x7C

#define PRT_INFO        am_util_stdio_printf
//#define PRT_INFO        no_print

//
// Define PRT_DATA if additional pkt data and other information is desired.
// PRT_INFO must also be defined.
//
#if PRT_INFO == am_util_stdio_printf
#define PRT_DATA        no_print
//#define PRT_DATA        am_util_stdio_printf
#endif

#define PARTIAL_IMG_SIZE 8180


typedef struct
{
    uint32_t                     crc32; // First word
    uint16_t                     msgType; // am_secboot_wired_msgtype_e
    uint16_t                     length;
} am_secboot_wired_msghdr_t;

typedef struct
{
    uint32_t                      length  : 16;
    uint32_t                      resv    : 14;
    uint32_t                      bEnd    : 1;
    uint32_t                      bStart  : 1;
} am_secboot_ios_pkthdr_t;

typedef struct
{
    am_secboot_wired_msghdr_t msg;
	uint32_t Sequence;
	uint8_t Image_Blob[PARTIAL_IMG_SIZE];
}am_Data_Message;

typedef struct
{
    am_secboot_wired_msghdr_t msg;
	uint32_t SrcMsg;
	uint32_t Status;
	uint32_t Sequence;
}am_ACK_Message;

typedef enum
{
    AM_SECBOOT_WIRED_MSGTYPE_HELLO,
    AM_SECBOOT_WIRED_MSGTYPE_STATUS,
    AM_SECBOOT_WIRED_MSGTYPE_OTADESC,
    AM_SECBOOT_WIRED_MSGTYPE_UPDATE,
    AM_SECBOOT_WIRED_MSGTYPE_ABORT,
    AM_SECBOOT_WIRED_MSGTYPE_RECOVER,
    AM_SECBOOT_WIRED_MSGTYPE_RESET,
    AM_SECBOOT_WIRED_MSGTYPE_ACK,
    AM_SECBOOT_WIRED_MSGTYPE_DATA,
} am_secboot_wired_msgtype_e;

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
am_hal_iom_buffer(128) g_psReadData;
extern uint8_t IMGDataBegin;
extern uint8_t IMGDataEnd;

am_Data_Message g_Data_Message;

struct
{
    am_secboot_ios_pkthdr_t       header;
    uint8_t                       data[MAX_IOS_LRAM_SIZE - sizeof(am_secboot_ios_pkthdr_t)];
} g_IosPktData;

volatile bool bIosInt = false;

void *g_IOMHandle;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void iom_slave_write(uint32_t offset, uint32_t *pBuf, uint32_t size);

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
static am_hal_gpio_pincfg_t g_AM_BSP_GPIO_BOOT_HANDSHAKE =
{
    .uFuncSel       = AM_HAL_PIN_2_GPIO,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
};

static am_hal_iom_config_t g_sIOMSpiConfig =
{
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq = AM_HAL_IOM_8MHZ,
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0,    // Default
};

//*****************************************************************************
//
// no_print
//
//*****************************************************************************
int no_print(char*pFmtStr, ...)
{
    return 0;
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_isr(void)
{
    //
    // Read and clear the GPIO interrupt status.
    //
#if defined(AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);

    am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
    am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
    am_hal_gpio_interrupt_service(pGpioIntStatusMask);
#elif defined(AM_PART_APOLLO3)
    uint64_t ui64Status;

    am_hal_gpio_interrupt_status_get(false, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);
    am_hal_gpio_interrupt_service(ui64Status);
#else
    #error Unknown device.
#endif
}

// ISR callback for the host IOINT
static void hostint_handler(void)
{
    bIosInt = true;
}

void handshake_set_up(void)
{
    //
    // Set up the host IO interrupt
    //
    am_hal_gpio_pinconfig(BOOTLOADER_HANDSHAKE_PIN, g_AM_BSP_GPIO_BOOT_HANDSHAKE);

    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, BOOTLOADER_HANDSHAKE_PIN));

    //
    // Register handler for IOS => IOM interrupt
    //
    am_hal_gpio_interrupt_register(BOOTLOADER_HANDSHAKE_PIN, hostint_handler);
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, BOOTLOADER_HANDSHAKE_PIN));
    NVIC_EnableIRQ(GPIO_IRQn);
}

//*****************************************************************************
//
// Initialize the IOM.
//
//*****************************************************************************
static void iom_set_up(uint32_t iomModule)
{
    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(iomModule, &g_IOMHandle);

    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);


    //
    // Configure the IOM for SPI.
    //
    am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);


    //
    // Enable the IOM.
    //
    am_hal_iom_enable(g_IOMHandle);

	//handshake_set_up();
}

//*****************************************************************************
//
// Reset the slave device and force it into boot mode.
//
//*****************************************************************************
void start_boot_mode(bool bReset)
{
	const am_hal_gpio_pincfg_t pincfg_RESET_OUTPUT =
	{
	    .uFuncSel       = 3,
	    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_4MA, //with Higher drive strength 
	    .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL
	};

	if ( !bReset )
    {
        //
        // Drive RESET high and configure the pin.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_pinconfig(DRIVE_SLAVE_RESET_PIN, pincfg_RESET_OUTPUT);

        //
        // Drive the override pin high and configure the pin.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_OVERRIDE_PIN, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_pinconfig(DRIVE_SLAVE_OVERRIDE_PIN, g_AM_HAL_GPIO_OUTPUT);
    }
    else
    {
        //
        // Drive RESET low.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

        //
        // Drive the override pin low to force the slave into boot mode.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_OVERRIDE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

        //
        // Short delay.
        //
        am_util_delay_ms(1);

        //
        // Release RESET.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_SET);

        //
        // Short delay.
        //
        //am_util_delay_ms(1);
    }
}

//*****************************************************************************
//
// Read a packet from the SBL IOS.
//
//*****************************************************************************
void iom_slave_read(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = offset;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;


    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;


    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

//*****************************************************************************
//
// Write a packet to the SBL IOS.
//
//*****************************************************************************
void iom_slave_write(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = offset;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;


    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;


    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
void send_hello(void)
{
    struct
    {
        am_secboot_ios_pkthdr_t   hdr;
        am_secboot_wired_msghdr_t msg;
    } pkt;

    pkt.hdr.bStart = 1;
    pkt.hdr.bEnd = 1;
    pkt.hdr.length = 12;
    pkt.msg.msgType = AM_SECBOOT_WIRED_MSGTYPE_HELLO;
    pkt.msg.length = sizeof(am_secboot_wired_msghdr_t);

    //
    // Compute CRC
    //
    PRT_INFO("Send a HELLO packet\n");
    am_hal_crc32((uint32_t)&pkt.msg.msgType, pkt.msg.length - sizeof(uint32_t), &pkt.msg.crc32);
    iom_slave_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&pkt, sizeof(pkt));
}

//*****************************************************************************
//
// Send a "UPDATE" packet.
//
//*****************************************************************************
void send_update(uint8_t *Blob, uint32_t size)
{
    struct
    {
        am_secboot_ios_pkthdr_t   hdr;
        am_secboot_wired_msghdr_t msg;
		uint32_t Total_Size;
		uint32_t Blob_CRC32;
		uint32_t Valid_Size;
    } pkt;

    pkt.hdr.bStart = 1;
    pkt.hdr.bEnd = 1;
    pkt.hdr.length = (4*5);
    pkt.msg.msgType = AM_SECBOOT_WIRED_MSGTYPE_UPDATE;
    pkt.msg.length = sizeof(pkt)-sizeof(am_secboot_ios_pkthdr_t);
	pkt.Total_Size = size;
	am_hal_crc32((uint32_t)Blob, size, &pkt.Blob_CRC32);
	pkt.Valid_Size = 0;
	am_hal_crc32((uint32_t)&pkt.msg.msgType, pkt.msg.length - sizeof(uint32_t), &pkt.msg.crc32);
	PRT_INFO("Send UPDATE packet	\n");
    
    iom_slave_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&pkt, sizeof(pkt));
}


void send_SPI_Data_Fragment(uint8_t * data_message)
{
    uint32_t ui32DotCnt = 0;
	am_secboot_wired_msghdr_t *pHdr = (am_secboot_wired_msghdr_t *)data_message;
    uint32_t ui32PktLength = pHdr->length;
	
	for (uint32_t index = 0; index < ui32PktLength; index += sizeof(g_IosPktData.data))
    {
        PRT_DATA("index             : %d\n", index);

        g_IosPktData.header.bStart = 0;
        g_IosPktData.header.bEnd = 0;
        g_IosPktData.header.length = 0;

        //
        // If this is the first packet, then set the Start flag.
        //
        if ( 0 == index )
        {
            g_IosPktData.header.bStart = 1;
        }

        //
        // If this this the last packet, then set the End flag.
        //
        if ((index + sizeof(g_IosPktData.data)) >= ui32PktLength)
        {
            g_IosPktData.header.bEnd = 1;
        }

        //
        // Build and Send the next packet.
        //
        g_IosPktData.header.length = ((ui32PktLength - index) < sizeof(g_IosPktData.data)) ? (ui32PktLength - index) : sizeof(g_IosPktData.data);
        memcpy(&g_IosPktData.data[0], data_message+index, g_IosPktData.header.length);
        g_IosPktData.header.length += sizeof(am_secboot_ios_pkthdr_t);
        bIosInt = false;

        PRT_DATA("IOS Length        : %d\n", g_IosPktData.header.length);
        PRT_DATA("IOS Start Bit     : %d\n", g_IosPktData.header.bStart);
        PRT_DATA("IOS End Bit       : %d\n", g_IosPktData.header.bEnd);

        if ( ui32DotCnt >= 20 )
        {
            PRT_INFO("\n");
            ui32DotCnt = 0;
        }
        PRT_INFO("*");
        ui32DotCnt++;

        iom_slave_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&g_IosPktData, g_IosPktData.header.length);

        //
        // Wait for the GPIO Interrupt before sending the next packet.
        //
        for (uint32_t timeout = 0; timeout < 100000; timeout++)
        {
            if ( bIosInt )
            {
                PRT_DATA("Received Handshake for next packet after %d us\n", timeout);
                break;
            }
            else
            {
                am_util_delay_us(1);
            }
        }

        if ( !bIosInt )
        {
            PRT_INFO("Timed out waiting for Handshake signal\n");
        }
    }
}

//*****************************************************************************
//
// Send a "DATA" packet.
//
//*****************************************************************************
void send_data(uint8_t *Blob, uint32_t size, uint32_t seq)
{
	g_Data_Message.msg.msgType = AM_SECBOOT_WIRED_MSGTYPE_DATA;
	g_Data_Message.msg.length = size+12;   
	g_Data_Message.Sequence = seq;
	memcpy(g_Data_Message.Image_Blob, Blob,size);
	am_hal_crc32((uint32_t)&g_Data_Message.msg.msgType, g_Data_Message.msg.length - sizeof(uint32_t), &g_Data_Message.msg.crc32);
	PRT_INFO("Send DATA packet\n");	
	send_SPI_Data_Fragment((uint8_t *)&g_Data_Message);
}

void wait_4_ACK(void)
{
	uint32_t maxSize = MAX_SPI_SIZE;
	//
	// Wait for the GPIO Interrupt before sending the next packet.
	//
	for (uint32_t timeout = 0; timeout < 100000; timeout++)
	{
	    if ( bIosInt )
	    {
	        PRT_DATA("Received Handshake for next packet after %d us\n", timeout);
	        break;
	    }
	    else
	    {
	        am_util_delay_us(1);
	    }
	}

	if ( !bIosInt )
	{
	    PRT_INFO("Timed out waiting for Handshake signal\n");
	}

	if ( bIosInt == true )
    {
        bIosInt = false;
        uint32_t iosSize = 0;

        //
        // Read the Data Size from the IOS.
        //
        iom_slave_read(IOSOFFSET_READ_FIFOCTR, &iosSize, 2);
        iosSize = (iosSize > maxSize) ? maxSize : iosSize;

        if ( iosSize > 0 )
        {
            //
            // Read the Data from the IOS.
            //
            iom_slave_read(IOSOFFSET_READ_FIFO, (uint32_t*)&g_psReadData, iosSize);
			am_ACK_Message *pAckMsg = (am_ACK_Message *)&g_psReadData;

			PRT_INFO("pAckMsg->msg.crc32=%x\n", pAckMsg->msg.crc32);
			PRT_INFO("pAckMsg->msg.msgType=%d\n", pAckMsg->msg.msgType);
			PRT_INFO("pAckMsg->msg.length=%d\n", pAckMsg->msg.length);
			PRT_INFO("pAckMsg->SrcMsg=%d\n", pAckMsg->SrcMsg);
			PRT_INFO("pAckMsg->Status=%d\n", pAckMsg->Status);
			PRT_INFO("pAckMsg->Sequence=%d\n", pAckMsg->Sequence);
        }
    }
}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
	uint32_t u32TotalSize = ((uint8_t *)(&IMGDataEnd) - (uint8_t *)(&IMGDataBegin));
	uint32_t sub_seq_size = 0;
	//
    // Default setup.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
    am_bsp_low_power_init();

    //
    // Enable the ITM
    //
    am_bsp_itm_printf_enable();
    am_util_stdio_printf("\nApollo3 SPI boot Host( IMG size: %d, %x, %x)\n",((uint8_t *)(&IMGDataEnd) - (uint8_t *)(&IMGDataBegin)), (&IMGDataEnd), (&IMGDataBegin));

    am_util_stdio_printf("SPI clock = %d.%d MHz\n",
                         g_sIOMSpiConfig.ui32ClockFreq / 1000000,
                         g_sIOMSpiConfig.ui32ClockFreq % 1000000);


    //
    // Set and configure the reset/bootmode pins high, but don't reset slave.
    //
    start_boot_mode(false);

    //
    // Start the IOM interface.
    //
    iom_set_up(IOM_MODULE);

    //
    // Force the slave into boot mode.
    //
    start_boot_mode(true);

    //
    // Wait for initial handshake signal to know that IOS interface is alive
    //
    //while( !bIosInt );

	am_util_delay_ms(100);


	//
	// Drive the override pin High after the slave into boot mode.
	//
	am_hal_gpio_state_write(DRIVE_SLAVE_OVERRIDE_PIN, AM_HAL_GPIO_OUTPUT_SET);


	handshake_set_up();
	
	
    bIosInt = false;

    //
    // Short delay.
    //
    am_util_delay_ms(1);

    //
    // Send the "HELLO" message to connect to the interface.
    //
    send_hello();

    while( !bIosInt );
    bIosInt = false;

    //
    // Read the "STATUS" response from the IOS.
    //
    iom_slave_read(IOSOFFSET_READ_FIFO, (uint32_t*)&g_psReadData, 88);

	send_update((uint8_t *)(&IMGDataBegin), u32TotalSize);
	wait_4_ACK();

	for(uint32_t i = 0; i < u32TotalSize; )
	{
		sub_seq_size = PARTIAL_IMG_SIZE;
		if(i+PARTIAL_IMG_SIZE > u32TotalSize)
			sub_seq_size =  (u32TotalSize - i);
		
		send_data((uint8_t *)(&IMGDataBegin)+i, sub_seq_size, i);
		wait_4_ACK();

		i += sub_seq_size;
	}

    while (1)
    {
        //
        // Go to Deep Sleep and stay there.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

