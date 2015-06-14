;---------------------------------------------------------------------------;
; Extended atoi                                                (C)ChaN, 2011
;---------------------------------------------------------------------------;


.nolist
#include <avr/io.h>	// Include device specific definitions.
.list

#ifdef SPM_PAGESIZE	// Recent devices have "lpm Rd,Z+" and "movw".
.macro	_LPMI	reg
	lpm	\reg, Z+
.endm
.macro	_MOVW	dh,dl, sh,sl
	movw	\dl, \sl
.endm
#else			// Earlier devices do not have "lpm Rd,Z+" nor "movw".
.macro	_LPMI	reg
	lpm
	mov	\reg, r0
	adiw	ZL, 1
.endm
.macro	_MOVW	dh,dl, sh,sl
	mov	\dl, \sl
	mov	\dh, \sh
.endm
#endif

;---------------------------------------------------------------------------
; Extended numeral string input
;
;Prototype:
; char xatoi (           /* 1: Successful, 0: Failed */
;      const char **str, /* pointer to pointer to source string */
;      long *res         /* result */
; );
;

.func xatoi
.global xatoi
xatoi:
	_MOVW	r1, r0, r23, r22
	_MOVW	XH, XL, r25, r24
	ld	ZL, X+
	ld	ZH, X+
	clr	r18		;r21:r18 = 0;
	clr	r19		;
	clr	r20		;
	clr	r21		;/
	clt			;T = 0;

	ldi	r25, 10		;r25 = 10;
	rjmp	41f		;/
40:	adiw	ZL, 1		;Z++;
41:	ld	r22, Z		;r22 = *Z;
	cpi	r22, ' '	;if(r22 == ' ') continue
	breq	40b		;/
	brcs	70f		;if(r22 < ' ') error;
	cpi	r22, '-'	;if(r22 == '-') {
	brne	42f		; T = 1;
	set			; continue;
	rjmp	40b		;}
42:	cpi	r22, '9'+1	;if(r22 > '9') error;
	brcc	70f		;/
	cpi	r22, '0'	;if(r22 < '0') error;
	brcs	70f		;/
	brne	51f		;if(r22 > '0') cv_start;
	ldi	r25, 8		;r25 = 8;
	adiw	ZL, 1		;r22 = *(++Z);
	ld	r22, Z		;/
	cpi	r22, ' '+1	;if(r22 <= ' ') exit;
	brcs	80f		;/
	cpi	r22, 'b'	;if(r22 == 'b') {
	brne	43f		; r25 = 2;
	ldi	r25, 2		; cv_start;
	rjmp	50f		;}
43:	cpi	r22, 'x'	;if(r22 != 'x') error;
	brne	51f		;/
	ldi	r25, 16		;r25 = 16;

50:	adiw	ZL, 1		;Z++;
	ld	r22, Z		;r22 = *Z;
51:	cpi	r22, ' '+1	;if(r22 <= ' ') break;
	brcs	80f		;/
	cpi	r22, 'a'	;if(r22 >= 'a') r22 =- 0x20;
	brcs	52f		;
	subi	r22, 0x20	;/
52:	subi	r22, '0'	;if((r22 -= '0') < 0) error;
	brcs	70f		;/
	cpi	r22, 10		;if(r22 >= 10) {
	brcs	53f		; r22 -= 7;
	subi	r22, 7		; if(r22 < 10)
	cpi	r22, 10		;
	brcs	70f		;}
53:	cp	r22, r25	;if(r22 >= r25) error;
	brcc	70f		;/
60:	ldi	r24, 33		;r21:r18 *= r25;
	sub	r23, r23	;
61:	brcc	62f		;
	add	r23, r25	;
62:	lsr	r23		;
	ror	r21		;
	ror	r20		;
	ror	r19		;
	ror	r18		;
	dec	r24		;
	brne	61b		;/
	add	r18, r22	;r21:r18 += r22;
	adc	r19, r24	;
	adc	r20, r24	;
	adc	r21, r24	;/
	rjmp	50b		;repeat

70:	ldi	r24, 0
	rjmp	81f
80:	ldi	r24, 1
81:	brtc	82f
	clr	r22
	com	r18
	com	r19
	com	r20
	com	r21
	adc	r18, r22
	adc	r19, r22
	adc	r20, r22
	adc	r21, r22
82:	st	-X, ZH
	st	-X, ZL
	_MOVW	XH, XL, r1, r0
	st	X+, r18
	st	X+, r19
	st	X+, r20
	st	X+, r21
	clr	r1
	ret
.endfunc
