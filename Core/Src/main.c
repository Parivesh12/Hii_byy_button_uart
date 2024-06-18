#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

void config_evry(void){
    RCC->AHB1ENR |= (1 << 0);      // clk config for GPIOA
    RCC->AHB1ENR |= (1 << 3);      // CLK CONFIG FOR GPIOD
    RCC->APB1ENR |= (1 << 17);     // clk config for usart2

    GPIOA->MODER = 0x00000000;   // set pa0 as input mode
    GPIOA->MODER |= (1 << 5);    // set pa2 as in AF mode
    GPIOD->MODER &= ~(0x3 << (15 * 2)); // Clear mode bits for pin 15
    GPIOD->MODER |= (0x1 << (15 * 2));  // Set mode to output for pin 15
    GPIOD->MODER &= ~(0x3 << (14 * 2)); // Clear mode bits for pin 14
    GPIOD->MODER |= (0x1 << (14 * 2));  // Set mode to output for pin 14

    GPIOA->AFR[0] |= (7 << 8);   // for usart2 AF

    USART2->BRR = 0x0683;        // set baud rate of 9600
    USART2->CR1 |= (1 << 3);     // enable tx pin for sending data
    USART2->CR1 |= (1 << 13);    // enable usart2
    USART2->CR1 &= ~(1 << 12);   // word length of 1 start bit and 8 data bit
    USART2->CR1 &= ~(1 << 15);   // over sampling by 16
    USART2->CR2 &= ~(3 << 12);   // 1 stop bit
}

void config_interrupt(void){
    // Enable SYSCFG clock
    RCC->APB2ENR |= (1 << 14);   // clk config for interrupt

    // Connect EXTI line to PA0
    SYSCFG->EXTICR[0] = 0x00000000; // EXTI0 is connected to PA0

    // Configure EXTI line 0
    EXTI->IMR |= (1 << 0);  // Unmask interrupt
    EXTI->RTSR |= (1 << 0); // Trigger on rising edge

    // Enable and set EXTI0 Interrupt to priority 1
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void timer2_config(void){
    RCC->APB1ENR |= (1 << 0);      // clk config for timer
    TIM2->PSC = 1599;              // Prescaler value (assuming 84 MHz clock -> 10 kHz)
    TIM2->ARR = 19999;             // Auto-reload value (10 kHz / 50000 = 0.2 Hz -> 5 seconds)
    TIM2->DIER |= (1 << 0);        // Enable update interrupt
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void USART_send(char msg){
    USART2->DR = msg;
    while(!(USART2->SR & (1 << 7)));
}

void USART_sendt(char* msg){
    while(*msg){
        USART2->DR = *msg++;
        while(!(USART2->SR & (1 << 7)));
    }
}


//void SysTick_Init(void) {
//    SystemCoreClockUpdate();
//    if (SysTick_Config(SystemCoreClock / 1000)) { // 1 ms interrupt
//        while (1); // Capture error
//    }
//}

volatile uint32_t button_press_count = 0;
volatile uint8_t button_pressed = 0;
volatile uint32_t debounce_counter = 0;
volatile uint8_t button_stable = 0;
volatile uint32_t off = 0;
volatile uint32_t pressed_once = 0;
volatile uint32_t toggle = 0;

volatile uint32_t ms2=0;

void EXTI0_IRQHandler(void){
    if(EXTI->PR & (1 << 0)){
//       button_press_count++;
//    	delay(10);
        EXTI->PR |= (1 << 0);     // Clear interrupt flag
        button_pressed = 1;       // Set the button pressed flag
//        debounce_counter = 0;     // Reset debounce counter
        pressed_once = 1;   //
        off = 0;
    }

}


//void SysTick_Handler(void){
//	ms2++;

//}

void delay(uint32_t ms) {
    uint32_t start = ms2;
    while ((ms2 - start) < ms);
}

void SysTick_Handler(void){
ms2++;
if(pressed_once){
   if((GPIOA->IDR & (1<<0))!=0){
	   debounce_counter++;
	   toggle = 1;
	   off=0;
   }else{

	   toggle = 0;
	   if(!toggle){
	  off++;

   }

   if ((debounce_counter >= 20) && (off>=50)) {
	   button_press_count++;
	   pressed_once=0;
	   GPIOD->ODR ^= (1 << 14);
//	   char buffer[10];
//	                  sprintf(buffer, "%lu", button_press_count);
//	                  USART_sendt(buffer);
//	                  USART_send(' ');
   }
//   button_pressed = 0;  // Reset the button pressed flag
//               debounce_counter = 0;  // Reset debounce counter
   }
}
}

//   if(off>10)

//    if (button_pressed) {
//        debounce_counter++;

//        if (debounce_counter >= 10) {  // Check for 50 ms debounce time
//            if ((GPIOA->IDR & (1 << 0)) != 0) {  // If button is still pressed
//                button_stable = 1;
//                button_press_count++;
//                button_stable =0;
//                GPIOD->ODR ^= (1 << 14);  // Toggle LED for visual feedback
//
//                // Send button press count over UART
//                char buffer[10];
//                sprintf(buffer, "%lu", button_press_count);
//                USART_sendt(buffer);
//                USART_send(' ');
////                USART_sendt("\r\n");  // Send a newline for better readability
//            }
//            button_pressed = 0;  // Reset the button pressed flag
//            debounce_counter = 0;  // Reset debounce counter
//        }



void TIM2_IRQHandler(void){
    char buffer[10];
    if(TIM2->SR & (1 << 0)){
        TIM2->SR &= ~(1 << 0); // Clear interrupt flag

        if(button_press_count == 1){
            USART_sendt("Hii ");
        } else if(button_press_count == 2){
            USART_sendt("BYY ");
        } else {
            sprintf(buffer, "%lu", button_press_count);
            USART_sendt(buffer);
            USART_send(' ');
        }
        button_press_count = 0; // Reset the button press count
    }
}

int main(void){
    config_evry();
    config_interrupt();
    timer2_config();
    SysTick_Config(SystemCoreClock / 1000);  // Configure SysTick for 1ms interrupts
    TIM2->CR1 |= (1 << 0);  // Enable Timer 2

    while(1){
        // Main loop can perform other tasks if needed
    }
}
