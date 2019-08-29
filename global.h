#ifndef _GLOBAL_H_
#define _GLOBAL_H_
#ifdef __ASSEMBLER__

#define sreg_irq_save r2
#define irq_r16       r16

#else

#include <stdint.h>
register uint8_t sreg_irq_save asm("r2");
register uint8_t irq_r16 asm("r16");

#endif
#endif
