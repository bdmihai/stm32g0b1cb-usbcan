/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2021 Mihai Baneu                                             |
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
 |  Author: Mihai Baneu                           Last modified: 02.Jan.2021  |
 |                                                                            |
 |___________________________________________________________________________*/

#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"
#include "gpio.h"

#define LED_RXD_Pin GPIO_PIN_0
#define LED_RXD_GPIO_Port GPIOA
#define LED_TXD_Pin GPIO_PIN_1
#define LED_TXD_GPIO_Port GPIOA
#define LED_RDY_Pin GPIO_PIN_2
#define LED_RDY_GPIO_Port GPIOA

void gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_RXD_Pin|LED_TXD_Pin|LED_RDY_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : LED_RXD_Pin LED_TXD_Pin LED_RDY_Pin */
    GPIO_InitStruct.Pin = LED_RXD_Pin|LED_TXD_Pin|LED_RDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    /* disable JTAG */
//    MODIFY_REG(GPIOA->MODER,    GPIO_MODER_MODER15_Msk,     0);                         /* JTDI disabled */
//    MODIFY_REG(GPIOB->MODER,    GPIO_MODER_MODER4_Msk,      0);                         /* NJTRST disabled */
//    MODIFY_REG(GPIOB->MODER,    GPIO_MODER_MODER3_Msk,      0);                         /* JTDO disabled */
//    
//    /* configure LED pin */
//    MODIFY_REG(GPIOC->MODER,    GPIO_MODER_MODER13_Msk,     GPIO_MODER_MODER13_0);      /* set the pin as output */
//    MODIFY_REG(GPIOC->OTYPER,   GPIO_OTYPER_OT13_Msk,       0);                         /* push pull */
//    MODIFY_REG(GPIOC->OSPEEDR,  GPIO_OSPEEDR_OSPEED13_Msk,  0);                         /* low speed */
//    MODIFY_REG(GPIOC->PUPDR,    GPIO_PUPDR_PUPD13_Msk,      0);                         /* no pull up, no pull down */
}

void gpio_set_led()
{
    HAL_GPIO_WritePin(GPIOA, LED_RDY_Pin, GPIO_PIN_RESET);
//    GPIOC->BSRR = GPIO_BSRR_BR13;
}

void gpio_reset_led()
{
    HAL_GPIO_WritePin(GPIOA, LED_RDY_Pin, GPIO_PIN_SET);
//    GPIOC->BSRR = GPIO_BSRR_BS13;
}
