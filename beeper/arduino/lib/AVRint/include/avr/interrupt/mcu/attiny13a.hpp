#pragma once

#define AVRINT_RESET(...) AVRINT_ISR(__vector_0, __VA_ARGS__)
#define AVRINT_INT0(...) AVRINT_ISR(__vector_1, __VA_ARGS__)
#define AVRINT_PCINT0(...) AVRINT_ISR(__vector_2, __VA_ARGS__)
#define AVRINT_TIM0_OVF(...) AVRINT_ISR(__vector_3, __VA_ARGS__)
#define AVRINT_TIMER_OVERFLOW(...) AVRINT_TIM0_OVF(__VA_ARGS__)
#define AVRINT_EE_RDY(...) AVRINT_ISR(__vector_5, __VA_ARGS__)
#define AVRINT_ANA_COMP(...) AVRINT_ISR(__vector_5, __VA_ARGS__)
#define AVRINT_ANALOG_COMPARATOR(...) AVRINT_ANA_COMP(__VA_ARGS__)
#define AVRINT_TIM0_COMPA(...) AVRINT_ISR(__vector_6, __VA_ARGS__)
#define AVRINT_TIMER_COMPARE_A(...) AVRINT_TIM0_COMPA(__VA_ARGS__)
#define AVRINT_TIM0_COMPB(...) AVRINT_ISR(__vector_7, __VA_ARGS__)
#define AVRINT_TIMER_COMPARE_B(...) AVRINT_TIM0_COMPB(__VA_ARGS__)
#define AVRINT_WDT(...) AVRINT_ISR(__vector_8, __VA_ARGS__)
#define AVRINT_ADC(...) AVRINT_ISR(__vector_9, __VA_ARGS__)
