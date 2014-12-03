/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc111x_iocon.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Rommel Marcelo
 *           Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC111X_IOCON_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC111X_IOCON_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

/* Note: The IOCON offset is not linear. See User manual UM10398 Page 74 */
/* Note: The IOCON base is not linear. See User manual UM10398 Page 74 */

/* Register addresses ***************************************************************/

#define LPC11_IOCON_P0_0            (LPC11_IOCON_BASE + 0x00C)
#define LPC11_IOCON_P0_1            (LPC11_IOCON_BASE + 0x010)
#define LPC11_IOCON_P0_2            (LPC11_IOCON_BASE + 0x01C)
#define LPC11_IOCON_P0_3            (LPC11_IOCON_BASE + 0x02C)
#define LPC11_IOCON_P0_4            (LPC11_IOCON_BASE + 0x030)
#define LPC11_IOCON_P0_5            (LPC11_IOCON_BASE + 0x034)
#define LPC11_IOCON_P0_6            (LPC11_IOCON_BASE + 0x04C)
#define LPC11_IOCON_P0_7            (LPC11_IOCON_BASE + 0x050)
#define LPC11_IOCON_P0_8            (LPC11_IOCON_BASE + 0x060)
#define LPC11_IOCON_P0_9            (LPC11_IOCON_BASE + 0x064)
#define LPC11_IOCON_P0_10           (LPC11_IOCON_BASE + 0x068)
#define LPC11_IOCON_P0_11           (LPC11_IOCON_BASE + 0x074)

#define LPC11_IOCON_P1_0            (LPC11_IOCON_BASE + 0x078)
#define LPC11_IOCON_P1_1            (LPC11_IOCON_BASE + 0x07c)
#define LPC11_IOCON_P1_2            (LPC11_IOCON_BASE + 0x080)
#define LPC11_IOCON_P1_3            (LPC11_IOCON_BASE + 0x090)
#define LPC11_IOCON_P1_4            (LPC11_IOCON_BASE + 0x094)
#define LPC11_IOCON_P1_5            (LPC11_IOCON_BASE + 0x0a0)
#define LPC11_IOCON_P1_6            (LPC11_IOCON_BASE + 0x0a4)
#define LPC11_IOCON_P1_7            (LPC11_IOCON_BASE + 0x0a8)
#define LPC11_IOCON_P1_8            (LPC11_IOCON_BASE + 0x014)
#define LPC11_IOCON_P1_9            (LPC11_IOCON_BASE + 0x038)
#define LPC11_IOCON_P1_10           (LPC11_IOCON_BASE + 0x06c)
#define LPC11_IOCON_P1_11           (LPC11_IOCON_BASE + 0x098)

#define LPC11_IOCON_P2_0            (LPC11_IOCON_BASE + 0x008)
#define LPC11_IOCON_P2_1            (LPC11_IOCON_BASE + 0x028)
#define LPC11_IOCON_P2_2            (LPC11_IOCON_BASE + 0x05c)
#define LPC11_IOCON_P2_3            (LPC11_IOCON_BASE + 0x08c)
#define LPC11_IOCON_P2_4            (LPC11_IOCON_BASE + 0x040)
#define LPC11_IOCON_P2_5            (LPC11_IOCON_BASE + 0x044)
#define LPC11_IOCON_P2_6            (LPC11_IOCON_BASE + 0x000)
#define LPC11_IOCON_P2_7            (LPC11_IOCON_BASE + 0x020)
#define LPC11_IOCON_P2_8            (LPC11_IOCON_BASE + 0x024)
#define LPC11_IOCON_P2_9            (LPC11_IOCON_BASE + 0x054)
#define LPC11_IOCON_P2_10           (LPC11_IOCON_BASE + 0x058)
#define LPC11_IOCON_P2_11           (LPC11_IOCON_BASE + 0x070)

#define LPC11_IOCON_P3_0            (LPC11_IOCON_BASE + 0x084)
#define LPC11_IOCON_P3_1            (LPC11_IOCON_BASE + 0x088)
#define LPC11_IOCON_P3_2            (LPC11_IOCON_BASE + 0x09C)
#define LPC11_IOCON_P3_3            (LPC11_IOCON_BASE + 0x0ac)
#define LPC11_IOCON_P3_4            (LPC11_IOCON_BASE + 0x03c)
#define LPC11_IOCON_P3_5            (LPC11_IOCON_BASE + 0x048)

#define LPC11_IOCON_SCK_LOC         (LPC11_IOCON_BASE + 0x0b0)
#define LPC11_IOCON_DSR_LOC         (LPC11_IOCON_BASE + 0x0b4)
#define LPC11_IOCON_DCD_LOC         (LPC11_IOCON_BASE + 0x0b8)
#define LPC11_IOCON_RI_LOC          (LPC11_IOCON_BASE + 0x0bc)

/* Register bit definitions *********************************************************/
/* IOCON pin function select */

#define IOCON_FUNC_GPIO             (0)
#define IOCON_FUNC_ALT1             (1)
#define IOCON_FUNC_ALT2             (2)
#define IOCON_FUNC_ALT3             (3)
#define IOCON_FUNC_ALT4             (4)
#define IOCON_FUNC_ALT5             (5)
#define IOCON_FUNC_ALT6             (6)
#define IOCON_FUNC_ALT7             (7)

#define IOCON_FUNC_SHIFT            (0)   /* Bits 0-2: All types */
#define IOCON_FUNC_MASK             (7 << IOCON_FUNC_SHIFT)
#define IOCON_MODE_SHIFT            (3)   /* Bits 3-4: Type D,A,W */
#define IOCON_MODE_MASK             (3 << IOCON_MODE_SHIFT )
#define IOCON_HYS_SHIFT             (5)   /* Bit 5: Type D,W */
#define IOCON_HYS_MASK              (1 << IOCON_HYS_SHIFT)
                                          /* Bit 6-9: Reserved */
#define IOCON_OD_SHIFT              (10)  /* Bit 10: Type D,A,W */
#define IOCON_OD_MASK               (1 << IOCON_OD_SHIFT)
                                          /* Bit 11-31: Reserved */

/* Pin modes */

#define IOCON_MODE_FLOAT            (0)   /* 00: pin has neither pull-up nor pull-down */
#define IOCON_MODE_PD               (1)   /* 01: pin has a pull-down resistor enabled */
#define IOCON_MODE_PU               (2)   /* 10: pin has a pull-up resistor enabled */
#define IOCON_MODE_RM               (3)   /* 11: pin has repeater mode enabled */

/* Pin types */

#define IOCON_TYPE_D_MASK (0x0000067f) /* All ports except where ADC/DAC, USB, I2C is present */
#define IOCON_TYPE_A_MASK (0x000105df) /* USB/ADC/DAC P0:12-13, P0:23-26, P1:30-31 */
#define IOCON_TYPE_U_MASK (0x00000007) /* USB P0:29 to 31 */
#define IOCON_TYPE_I_MASK (0x00000347) /* I2C/USB P0:27-28, P5:2-3  */
#define IOCON_TYPE_W_MASK (0x000007ff) /* I2S P0:7-9 */

/* Slew rate modes */

#define IOCON_SLEWMODE_NORMAL       (0 << IOCON_SLEW_SHIFT)
#define IOCON_SLEWMODE_FAST         (1 << IOCON_SLEW_SHIFT)

/* I2C modes */

#define IOCON_I2CMODE_SHIFT         (IOCON_I2CHS_SHIFT)
#define IOCON_I2CMODE_MASK          (3 << IOCON_I2CMODE_SHIFT)
#  define IOCON_I2CMODE_FAST        (0 << IOCON_I2CMODE_SHIFT)
#  define IOCON_I2CMODE_FASTPLUS    (1 << IOCON_I2CMODE_SHIFT)/*   */
#  define IOCON_I2CMODE_HIOPENDRAIN (2 << IOCON_I2CMODE_SHIFT)/*  */
#  define IOCON_I2CMODE_OPENDRAIN   (3 << IOCON_I2CMODE_SHIFT)/*  */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC118X_IOCON_H */
