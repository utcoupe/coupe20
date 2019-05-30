#include "compat.h"

long timeMicros() {
    return (DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000L) );
}
