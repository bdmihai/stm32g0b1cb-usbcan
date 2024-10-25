/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2023 Mihai Baneu                                             |
 │                                                                            |
 | Permission is hereby  granted,  free of charge,  to any person obtaining a |
 | copy of this software and associated documentation files (the "Software"), |
 | to deal in the Software without restriction,  including without limitation |
 | the rights to  use, copy, modify, merge, publish, distribute,  sublicense, |
 | and/or sell copies  of  the Software, and to permit  persons to  whom  the |
 | Software is furnished to do so, subject to the following conditions:       |
 |                                                                            |
 | The above  copyright notice  and this permission notice  shall be included |
 | in all copies or substantial portions of the Software.                     |
 |                                                                            |
 | THE SOFTWARE IS PROVIDED  "AS IS",  WITHOUT WARRANTY OF ANY KIND,  EXPRESS |
 | OR   IMPLIED,   INCLUDING   BUT   NOT   LIMITED   TO   THE  WARRANTIES  OF |
 | MERCHANTABILITY,  FITNESS FOR  A  PARTICULAR  PURPOSE AND NONINFRINGEMENT. |
 | IN NO  EVENT SHALL  THE AUTHORS  OR  COPYRIGHT  HOLDERS  BE LIABLE FOR ANY |
 | CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER IN AN ACTION OF CONTRACT, TORT |
 | OR OTHERWISE, ARISING FROM,  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR  |
 | THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                 |
 |____________________________________________________________________________|
 |                                                                            |
 |  Author: Mihai Baneu                           Last modified: 29.Dec.2023  |
 |                                                                            |
 |___________________________________________________________________________*/

#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"
#include "gpio.h"
#include "system.h"

uint32_t SystemCoreClock = 64000000;
const uint32_t AHBPrescTable[16UL] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL, 6UL, 7UL, 8UL, 9UL};
const uint32_t APBPrescTable[8UL] =  {0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL};

extern void panic(const char *fmt, ...);

void system_init()
{
    HAL_Init();

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV5;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        panic("");
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        panic("");
    }
//    /* configure Flash prefetch, Instruction cache, Data cache */
//    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
//
//    /* configure the main internal regulator output voltage */
//    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
//    MODIFY_REG(PWR->CR, PWR_CR_VOS_Msk, PWR_CR_VOS_1);
//
//    /* enable the external High Speed Clock @25MHz*/
//    SET_BIT(RCC->CR, RCC_CR_HSEON);
//    do {
//    } while ((RCC->CR & RCC_CR_HSERDY_Msk) != RCC_CR_HSERDY);
//
//    /* configure the PLL */
//    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk, RCC_PLLCFGR_PLLSRC_HSE);
//    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, 25 << RCC_PLLCFGR_PLLM_Pos);
//    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, 192 << RCC_PLLCFGR_PLLN_Pos);
//    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk, 0);
//    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk, 4 << RCC_PLLCFGR_PLLQ_Pos);
//
//    /* enable PLL clock generation */
//    SET_BIT(RCC->CR, RCC_CR_PLLON);
//    do {
//    } while ((RCC->CR & RCC_CR_PLLRDY_Msk) != RCC_CR_PLLRDY);
//
//    /* to correctly read data from FLASH memory, the number of wait states (LATENCY)
//       must be correctly programmed according to the frequency of the CPU clock
//       (HCLK) of the device. */
//    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_3WS);
//
//    /* set the highest APBx dividers in order to ensure that we do not go through
//        a non-spec phase whatever we decrease or increase HCLK. */
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV16);
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV16);
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);
//
//    /* set the SYSCLK to PLL */
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
//    do {
//    } while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
//
//    /* disable HSI clock */
//    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
//    do {
//    } while ((RCC->CR & RCC_CR_HSIRDY_Msk) == RCC_CR_HSIRDY);
//
//    /* confgure the APB clocks */
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV2);
//    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV2);
//
//    /* enable AHB1 ports clock */
//    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
//    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
//    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
//
//    /* one us timer for delay */
//    TIM10->PSC = (system_cpu_f() / 1000000) - 1;
//    MODIFY_REG(TIM10->CR1, TIM_CR1_CEN_Msk, TIM_CR1_CEN);
//
//    /* stop timer when debuggng */
//    SET_BIT(DBGMCU->APB2FZ, DBGMCU_APB2_FZ_DBG_TIM10_STOP);
//
//    /* set-up the vector table in ram */
//    extern char __isr_vector_start;
//    SCB->VTOR = (uintptr_t) &__isr_vector_start;
}

unsigned int system_cpu_f()
{
    return SystemCoreClock;
}
