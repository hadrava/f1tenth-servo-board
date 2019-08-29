#define __SFR_OFFSET 0
#include "global.h"
#include <avr/io.h>

; Register usage readme: http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_reg_usage

.global TIMER0_OVF_vect
TIMER0_OVF_vect:
	in sreg_irq_save, SREG
	in irq_r16, GPIOR1
	inc irq_r16
	out GPIOR1, irq_r16
	out SREG, sreg_irq_save
	reti

.global TIMER2_OVF_vect
TIMER2_OVF_vect:
	in sreg_irq_save, SREG
	in irq_r16, GPIOR2
	inc irq_r16
	out GPIOR2, irq_r16
	out SREG, sreg_irq_save
	reti

.global INT0_vect
INT0_vect:
	in sreg_irq_save, SREG
	sbis GPIOR0, 0 ; skip next if bit 0 is set (we are detecting falling edge)
	rjmp int0_rising
	in irq_r16, TCNT0 ; Read current counter value
	sts counter_0, irq_r16 ; save it to RAM
	cpi irq_r16, 0x80 ; compare r16 with 0x80 (sets C flag iff r16 < 0x80)
	in irq_r16, GPIOR1 ; Read upper part
	brsh skip_increment_0 ; skip increment if C is cleared (TCNT0 >= 0x80)
	sbic TIFR0, TOV0 ; skip increment if interrupt flag is not set
	inc irq_r16 ; increment one
skip_increment_0:
	sts counter_0+1, irq_r16 ; save it to RAM

	cbi EIMSK, INT0 ; Disable any interrupt on INT0

	;cbi TIMSK0, TOIE0 ; Overflow Interrupt Disable (register out of range)
	lds irq_r16, TIMSK0
	cbr irq_r16, _BV(TOIE0)
	sts TIMSK0, irq_r16

	out SREG, sreg_irq_save
	reti

int0_rising:
	clr irq_r16
	out TCNT0, irq_r16 ; Reset current counter value to 0
	out GPIOR1, irq_r16 ; Clear software part of counter
	sbi GPIOR0, 0 ; Remember to detect falling edge

	;cbi EICRA, ISC00  ; Switch to falling edge irq (register out of range)
	lds irq_r16, EICRA
	cbr irq_r16, _BV(ISC00)
	sts EICRA, irq_r16

	;sbi TIMSK0, TOIE0 ; Overflow Interrupt Enable (register out of range)
	lds irq_r16, TIMSK0
	sbr irq_r16, _BV(TOIE0)
	sts TIMSK0, irq_r16

	sbi TIFR0, TOV0  ; Clear overflow interrupt flag (by writing one to it)
	out SREG, sreg_irq_save
	reti

.global INT1_vect
INT1_vect:
	in sreg_irq_save, SREG
	sbis GPIOR0, 1 ; skip next if bit 1 is set (we are detecting falling edge)
	rjmp int1_rising
	;in irq_r16, TCNT2 ; Read current counter value; (register out of range, use longer variant)
	lds irq_r16, TCNT2
	sts counter_1, irq_r16 ; save it to RAM
	cpi irq_r16, 0x80 ; compare r16 with 0x80 (sets C flag iff r16 < 0x80)
	in irq_r16, GPIOR2 ; Read upper part
	brsh skip_increment_1 ; skip increment if C is cleared (TCNT2 >= 0x80)
	sbic TIFR2, TOV2 ; skip increment if interrupt flag is not set
	inc irq_r16 ; increment one
skip_increment_1:
	sts counter_1+1, irq_r16 ; save it to RAM

	cbi EIMSK, INT1 ; Disable any interrupt on INT1

	;cbi TIMSK2, TOIE2 ; Overflow Interrupt Disable (register out of range)
	lds irq_r16, TIMSK2
	cbr irq_r16, _BV(TOIE2)
	sts TIMSK2, irq_r16

	out SREG, sreg_irq_save
	reti

int1_rising:
	clr irq_r16
	;out TCNT2, irq_r16 ; Reset current counter value to 0; (register out of range, use longer variant)
	sts TCNT2, irq_r16
	out GPIOR2, irq_r16 ; Clear software part of counter
	sbi GPIOR0, 1 ; Remember to detect falling edge

	;cbi EICRA, ISC10  ; Switch to falling edge irq (register out of range)
	lds irq_r16, EICRA
	cbr irq_r16, _BV(ISC10)
	sts EICRA, irq_r16

	;sbi TIMSK2, TOIE2 ; Overflow Interrupt Enable (register out of range)
	lds irq_r16, TIMSK2
	sbr irq_r16, _BV(TOIE2)
	sts TIMSK2, irq_r16

	sbi TIFR2, TOV2  ; Clear overflow interrupt flag (by writing one to it)
	out SREG, sreg_irq_save
	reti


.DATA
.global counter_0
counter_0:
	.BYTE 0
	.BYTE 0

.global counter_1
counter_1:
	.BYTE 0
	.BYTE 0

; vim: ft=avr8bit
