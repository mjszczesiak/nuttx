/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz-config.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* UARTs ****************************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if CHIP_NUARTS < 1
#  undef CONFIG_PIC32MZ_UART1
#  undef CONFIG_PIC32MZ_UART2
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 2
#  undef CONFIG_PIC32MZ_UART2
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 3
#  undef CONFIG_PIC32MZ_UART3
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 4
#  undef CONFIG_PIC32MZ_UART4
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 5
#  undef CONFIG_PIC32MZ_UART5
#  undef CONFIG_PIC32MZ_UART6
#elif CHIP_NUARTS < 6
#  undef CONFIG_PIC32MZ_UART6
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_PIC32MZ_UART1) || defined(CONFIG_PIC32MZ_UART2) || \
    defined(CONFIG_PIC32MZ_UART4) || defined(CONFIG_PIC32MZ_UART4) || \
    defined(CONFIG_PIC32MZ_UART5) || defined(CONFIG_PIC32MZ_UART6)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART3)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART4)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART5)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_PIC32MZ_UART6)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Device Configuration *************************************************************/
/* DEVCFG3 */
/* Configurable settings */

#ifndef CONFIG_PIC32MZ_USERID               /* User ID */
#  define CONFIG_PIC32MZ_USERID   0x584e    /* "NX" */
#endif

#ifndef CONFIG_PIC32MZ_FMIIEN               /* Ethernet MII enable: 0=RMII 1=MII */
#  define CONFIG_PIC32MZ_FMIIEN   1         /* MII enabled */
#endif

#ifndef CONFIG_PIC32MZ_PGL1WAY              /* Permission group lock one way configuration */
#  define CONFIG_PIC32MZ_PGL1WAY  0         /* Allow multiple configurations */
#endif

#ifndef CONFIG_PIC32MZ_PMDL1WAY             /* Peripheral module disable configuration */
#  define CONFIG_PIC32MZ_PMDL1WAY 0         /* Allow multiple reconfigurations */
#endif

#ifndef CONFIG_PIC32MZ_IOL1WAY              /* Peripheral pin select configuration */
#  define CONFIG_PIC32MZ_IOL1WAY  0         /* Allow multiple reconfigurations */
#endif

#ifndef CONFIG_PIC32MZ_FETHIO               /* Ethernet I/O Pins 0=alternate 1=default */
#  define CONFIG_PIC32MZ_FETHIO   1         /* Default Ethernet I/O Pins */
#endif

#ifndef CONFIG_PIC32MZ_FUSBIDIO              /* USB USBID selection: 0=GPIO 1=USB */
#  ifdef CONFIG_PIC32MZ_USB
#    define CONFIG_PIC32MZ_FUSBIDIO 0        /* USBID pin is controlled by the IOPORT configuration */
#  else
#    define CONFIG_PIC32MZ_FUSBIDIO 1        /* USBID pin is controlled by the USB module */
#  endif
#endif

/* DEVCFG2 */
/* PLL Input Divider bits */

#undef CONFIG_PIC32MZ_PLLIDIV
#if BOARD_PLL_IDIV == 1
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_1
#elif BOARD_PLL_IDIV == 2
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_2
#elif BOARD_PLL_IDIV == 3
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_3
#elif BOARD_PLL_IDIV == 4
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_4
#elif BOARD_PLL_IDIV == 5
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_5
#elif BOARD_PLL_IDIV == 6
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_6
#elif BOARD_PLL_IDIV == 7
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_7
#elif BOARD_PLL_IDIV == 8
#  define CONFIG_PIC32MZ_PLLIDIV  DEVCFG2_FPLLIDIV_8
#else
#  error "Unsupported BOARD_PLL_IDIV"
#endif

/* System PLL Divided Input Clock Frequency Range bits */

#if (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 5000000
#  error BOARD_PLL_INPUT / BOARD_PLL_IDIV too low
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_BYPASS /* < 5 MHz */
#elif (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 9000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_5_10MHZ /* 5-10 MHz */
#elif (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 14500000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_8_16MHZ /* 8-16 MHz */
#elif (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 23500000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_13_26MHZ /* 13-26 MHz */
#elif (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 39000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_21_42MHZ /* 21-42 MHz */
#elif (BOARD_PLL_INPUT / BOARD_PLL_IDIV) < 64000000
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_34_64MHZ /* 36-64 MHz */
#else
#  error BOARD_PLL_INPUT / BOARD_PLL_IDIV too high
#  define CONFIG_PIC32MZ_FPLLRNG  DEVCFG2_FPLLRNG_34_64MHZ /* 36-64 MHz */
#endif

/* PLL multiplier */

#undef CONFIG_PIC32MZ_PLLMULT
#if BOARD_PLL_MULT >= 1 && BOARD_PLL_MULT <= 128
#  define CONFIG_PIC32MZ_PLLMULT  ((BOARD_PLL_MULT-1) << DEVCFG2_FPLLIDIV_SHIFT)
#else
#  error "Unsupported BOARD_PLL_MULT"
#endif

/* PLL output divider */

#undef CONFIG_PIC32MZ_PLLODIV
#if BOARD_PLL_ODIV == 2
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_2
#elif BOARD_PLL_ODIV == 4
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_4
#elif BOARD_PLL_ODIV == 8
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_8
#elif BOARD_PLL_ODIV == 16
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_16
#elif BOARD_PLL_ODIV == 32
#  define CONFIG_PIC32MZ_PLLODIV  DEVCFG2_FPLLODIV_32
#else
#  error "Unsupported BOARD_PLL_ODIV"
#endif

/* Not yet configurable settings (REVISIT) */

                                           /* System PLL Input Clock Select bit */
#define CONFIG_PIC32MZ_FPLLICLK   0        /* POSC is selected as input to the System PLL */
                                           /* USB PLL Input Frequency Select bit */
#define CONFIG_PIC32MZ_UPLLFSEL   DEVCFG2_UPLLFSEL

/* DEVCFG1 */
/* Configurable settings */

#undef CONFIG_PIC32MZ_FNOSC
#if defined(BOARD_FNOSC_FRC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_FRC
#elif defined(BOARD_FNOSC_SPLL)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_SPLL
#elif defined(BOARD_FNOSC_POSC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_POSC
#elif defined(BOARD_FNOSC_SOSC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_SOSC
#elif defined(BOARD_FNOSC_LPRC)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_LPRC
#elif defined(BOARD_FNOSC_FRCDIV)
#  define CONFIG_PIC32MZ_FNOSC    DEVCFG1_FNOSC_FRCDIV
#else
#  error "Unknown board FNOSC selection"
#endif

#undef CONFIG_PIC32MZ_FSOSCEN
#ifdef BOARD_SOSC_ENABLE
#  define CONFIG_PIC32MZ_FSOSCEN DEVCFG1_FSOSCEN
#else
#  define CONFIG_PIC32MZ_FSOSCEN 0
#endif

#undef CONFIG_PIC32MZ_IESO
#ifdef BOARD_SOSC_IESO
#  define CONFIG_PIC32MZ_IESO    DEVCFG1_IESO
#else
#  define CONFIG_PIC32MZ_IESO    0
#endif

#undef CONFIG_PIC32MZ_POSCMOD
#if defined(BOARD_POSC_ECMODE)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_EC
#elif defined(BOARD_POSC_HSMODE)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_HS
#elif defined(BOARD_POSC_DISABLED)
#  define CONFIG_PIC32MZ_POSCMOD DEVCFG1_POSCMOD_DIS
#else
#  error "Unknown board POSC mode"
#endif

#ifdef CONFIG_PIC32MZ_OSCIOFNC
#  undef CONFIG_PIC32MZ_OSCIOFNC
#  define CONFIG_PIC32MZ_OSCIOFNC DEVCFG1_OSCIOFNC
#else
#  undef CONFIG_PIC32MZ_OSCIOFNC
#  define CONFIG_PIC32MZ_OSCIOFNC 0
#endif

#undef CONFIG_PIC32MZ_FCKSM
#if defined(BOARD_POSC_SWITCH)
#  if defined(BOARD_POSC_FSCM)
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_BOTH
#  else
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_SWITCH
#  endif
#else
#  if defined(BOARD_POSC_FSCM)
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_MONITOR
#  else
#    define CONFIG_PIC32MZ_FCKSM DEVCFG1_FCKSM_NONE
#  endif
#endif

#undef CONFIG_PIC32MZ_WDTPS
#if BOARD_WD_PRESCALER == 1
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1
#elif BOARD_WD_PRESCALER == 2
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_2
#elif BOARD_WD_PRESCALER == 4
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_4
#elif BOARD_WD_PRESCALER == 8
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_8
#elif BOARD_WD_PRESCALER == 16
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_16
#elif BOARD_WD_PRESCALER == 32
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_32
#elif BOARD_WD_PRESCALER == 64
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_64
#elif BOARD_WD_PRESCALER == 128
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_128
#elif BOARD_WD_PRESCALER == 256
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_256
#elif BOARD_WD_PRESCALER == 512
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_512
#elif BOARD_WD_PRESCALER == 1024
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1024
#elif BOARD_WD_PRESCALER == 2048
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_2048
#elif BOARD_WD_PRESCALER == 4096
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_4096
#elif BOARD_WD_PRESCALER == 8192
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_8192
#elif BOARD_WD_PRESCALER == 16384
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_16384
#elif BOARD_WD_PRESCALER == 32768
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_32768
#elif BOARD_WD_PRESCALER == 65536
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_65536
#elif BOARD_WD_PRESCALER == 131072
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_131072
#elif BOARD_WD_PRESCALER == 262144
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_262144
#elif BOARD_WD_PRESCALER == 524288
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_524288
#elif BOARD_WD_PRESCALER == 1048576
#  define CONFIG_PIC32MZ_WDTPS   DEVCFG1_WDTPS_1048576
#else
#  error "Unsupported BOARD_WD_PRESCALER"
#endif

#undef CONFIG_PIC32MZ_FWDTEN
#if CONFIG_PIC32MZ_WDTENABLE
#  define CONFIG_PIC32MZ_FWDTEN  DEVCFG1_FWDTEN
#else
#  define CONFIG_PIC32MZ_FWDTEN  0
#endif

/* Not yet configurable settings */

#define CONFIG_PIC32MZ_DMTINV    DEVCFG1_FNOSC_FRCDIV
#define CONFIG_PIC32MZ_WDTSPGM   DEVCFG1_WDTSPGM
#define CONFIG_PIC32MZ_WINDIS    DEVCFG1_WINDIS
#define CONFIG_PIC32MZ_FWDTWINSZ DEVCFG1_FWDTWINSZ_25
#define CONFIG_PIC32MZ_DMTCNT    DEVCFG1_DMTCNT_MASK
#define CONFIG_PIC32MZ_FDMTEN    0

/* DEVCFG0 */
/* Configurable settings */

#undef CONFIG_PIC32MZ_DEBUGGER
#ifdef CONFIG_PIC32MZ_DEBUGGER_ENABLE
#  define CONFIG_PIC32MZ_DEBUGGER DEVCFG0_DEBUG_ENABLED
#else
#  define CONFIG_PIC32MZ_DEBUGGER DEVCFG0_DEBUG_DISABLED
#endif

#undef CONFIG_PIC32MZ_JTAGEN
#ifdef CONFIG_PIC32MZ_JTAG_ENABLE
#  define CONFIG_PIC32MZ_JTAGEN DEVCFG0_JTAGEN
#else
#  define CONFIG_PIC32MZ_JTAGEN 0
#endif

#undef CONFIG_PIC32MZ_ICESEL
#ifdef CONFIG_PIC32MZ_ICESEL_CH2
#  define CONFIG_PIC32MZ_ICESEL DEVCFG0_ICESEL_2
#else
#  define CONFIG_PIC32MZ_ICESEL DEVCFG0_ICESEL_1
#endif

#undef CONFIG_PIC32MZ_TRCEN
#ifdef CONFIG_PIC32MZ_TRACE_ENABLE
#  define CONFIG_PIC32MZ_TRCEN DEVCFG0_TRCEN
#else
#  define CONFIG_PIC32MZ_TRCEN 0
#endif

/* Not yet configurable settings */

#define CONFIG_PIC32MZ_BOOTISA  0 /* microMIPS always */
#define CONFIG_PIC32MZ_FECCCON  DEVCFG0_FECCCON_DISWR
#define CONFIG_PIC32MZ_FSLEEP   DEVCFG0_FSLEEP
#define CONFIG_PIC32MZ_DBGPER   DEVCFG0_DBGPER_MASK
#define CONFIG_PIC32MZ_EJTAGBEN DEVCFG0_EJTAGBEN

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_CONFIG_H */
