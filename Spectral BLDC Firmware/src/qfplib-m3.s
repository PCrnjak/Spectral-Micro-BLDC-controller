@ Copyright 2016 Mark Owen
@ http://www.quinapalus.com
@ E-mail: qfp@quinapalus.com
@
@ This file is free software: you can redistribute it and/or modify
@ it under the terms of version 2 of the GNU General Public License
@ as published by the Free Software Foundation.
@
@ This file is distributed in the hope that it will be useful,
@ but WITHOUT ANY WARRANTY; without even the implied warranty of
@ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
@ GNU General Public License for more details.
@
@ You should have received a copy of the GNU General Public License
@ along with this file.  If not, see <http://www.gnu.org/licenses/> or
@ write to the Free Software Foundation, Inc., 51 Franklin Street,
@ Fifth Floor, Boston, MA  02110-1301, USA.

.syntax unified
.cpu cortex-m3
.thumb

@ exported symbols

.global qfp_fadd
.global qfp_fsub
.global qfp_fmul
.global qfp_fdiv
.global qfp_fsqrt
.global qfp_fexp
.global qfp_fsin
.global qfp_fcos
.global qfp_fln
.global qfp_ftan
.global qfp_fatan2


@ This macro loads a 32-bit constant n into register rx.

.macro movlong rx,n
 movw \rx,#(\n)&0xffff
 movt \rx,#((\n)>>16)&0xffff
.endm

@ Addition and subtraction are handled using the following two macros.
@ fadd_s0 is used for adding arguments of like sign and subtracting
@ arguments of unlike sign; fsub_s0 for adding arguments of unlike sign
@ and subtracting arguments of like sign.
@
@ The parameters to the macros are the registers containing the
@ mantissas of the two arguments, the exponents of the two arguments,
@ and a flag indicating if the result is to be negated. In the
@ case of fadd_s0 ye has already been subtracted from xe, with
@ the result in r12.
@
@ fadd_s0 is split into two parts, fadd_s0a and fadd_s0b; likewise fsub_s0.

.macro fadd_s0a xm,ym,xe,ye,neg
 rsbs \ye,r12,#31         @ get complementary mantissa shift
 bmi 105f                 @ too great? result is just x
 lsl \ye,\ym,\ye          @ save sticky bits before...
 lsr \ym,\ym,r12          @ ... aligning mantissas
 adds r0,\xm,\ym          @ the addition
 cmp r0,#0x02000000       @ need to increment exponent?
 bhs 106f
 sub r0,#0x01000000
 lsrs r0,#1
 adc r0,r0,\xe,lsl#23     @ add in exponent, round mantissa
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 lsls \ye,#1
 it ne                    @ sticky bits nonzero?
 bxne r14                 @ finished if so
 it cs                    @ if tie in rounding, make result even
 biccs r0,#1
 bx r14
.endm

.macro fadd_s0b xm,ym,xe,ye,neg
.balign 4
106:                      @ case where exponent increments
 cmp \xe,#0xfe            @ will result become infinity?
 bge 7f
 lsrs r0,#2
 adc r0,r0,\xe,lsl#23     @ add in exponent, round mantissa
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 cbz \ye,8f               @ sticky bits zero?
 bx r14                   @ otherwise finished
.balign 4
8:
 it cs                    @ if tie in rounding, make result even
 biccs r0,#1
 bx r14

.balign 4
105:                      @ x>>y so result is x
 lsrs r0,\xm,#1
 bfi r0,\xe,#23,#8        @ insert exponent
.if \neg
 orr r0,#0x80000000
.endif
 bx r14
.balign 4
7:                        @ infinite result
 mov r0,#0x7f800000
.if \neg
 orr r0,#0x80000000
.endif
 bx r14
.endm

.macro fadd_s0 xm,ym,xe,ye,neg
 fadd_s0a \xm,\ym,\xe,\ye,\neg
 fadd_s0b \xm,\ym,\xe,\ye,\neg
.endm

.macro fsub_s0a xm,ym,xe,ye,neg
 bic \xm,\xm,#0xff000000  @ clear exponent fields
 bic \ym,\ym,#0xff000000
 orrs \ym,\ym,#0x01000000 @ insert implied 1 in y only; clear carry
 sub \ye,\xe,\ye          @ difference of exponents
 lsr r12,\ym,\ye          @ align mantissas
 rsb \ye,#32              @ complementary shift
 lsl \ye,\ym,\ye          @ sticky bits
 cbz \ye,8f               @ sticky bits=0?
 sbcs \xm,r12             @ the subtraction: note carry is clear from above
 bmi 109f                 @ does subtraction cause exponent to change?
.if \neg
 add \xe,#256             @ insert negative sign bit above exponent if required
.endif
 lsrs \xm,#1              @ round mantissa...
 adc r0,\xm,\xe,lsl#23    @ ... and pack result
 bx r14
.balign 4
8:                        @ here sticky bits are 0
 subs \xm,r12             @ do subtraction with carry set
 bmi 105f                 @ does subtraction cause exponent to change?
.if \neg
 add \xe,#256             @ insert negative sign bit above exponent if required
.endif
 lsrs \xm,#1              @ round mantissa...
 adc r0,\xm,\xe,lsl#23    @ ... and pack result
 it cc                    @ was rounding tied?
 bxcc r14
 bic r0,#1                @ if so, round to even
 bx r14
.endm

.macro fsub_s0b xm,ym,xe,ye,neg
.balign 4
105:                      @ here xm is result-0x01000000, exponent in \xe, and sticky bits are zero
 cmn \xm,#0x00800000      @ (i.e., result is exact)
 blt 6f                   @ exponent decrease of more than 1?
                          @ here exponent is decreasing by exactly 1 and result is exact
 cmp \xe,#1               @ will result be denormal?
 beq 7f
 add r0,\xm,\xe,lsl#23    @ pack result
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 bx r14

.balign 4
109:                      @ here xm is result-0x01000000, exponent in \xe
 cmn \xm,#0x00800000
 blt 6f                   @ exponent decrease of >1?
                          @ here exponent is decreasing by exactly 1
 cmp \xe,#1               @ will result be denormal?
 beq 7f
 rsbs \ye,#0              @ calculate sticky bits of result
 lsls \ye,#1              @ check rounding
 adc r0,\xm,\xe,lsl#23    @ round and pack exponent
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 it cc                    @ potential rounding tie?
 bxcc r14
 it ne                    @ sticky bits<>0?
 bxne r14
 bic r0,#1                @ make even
 bx r14

.balign 4
7:                        @ here the result is denormal (but the exponent has decreased by only 1)
 add r0,\xm,#0x1000000    @ fix result
 lsrs r0,#1               @ result is denormal so it is exact
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 bx r14

.balign 4
6:                        @ here if cancellation leads to exponent decrease of >1; result must be exact
 adds \xm,#0x01000000     @ correct difference now <=0x007fffff
 beq 7f                   @ zero result?
 clz \ye,\xm              @ 9 => shift up one place for normalised result
 subs \ye,#8              @ \ye now has shift up for normal case
 subs \xe,\ye             @ correct exponent for normalising shift
 subs \xe,\xe,#1          @ correct exponent offset
 lsl r0,\xm,\ye           @ perform normalising shift
 ble 5f                   @ result is denormal?
 bfi r0,\xe,#23,#8        @ pack exponent
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 bx r14
.balign 4
7:
 movs r0,#0
 bx r14
.balign 4
5:
 rsb \xe,\xe,#1           @ result is denormal: get required down shift
 lsrs r0,\xe              @ generate denormal result
.if \neg
 orr r0,#0x80000000       @ negate result if required
.endif
 bx r14
.endm

.macro fsub_s0 xm,ym,xe,ye,neg
 fsub_s0a \xm,\ym,\xe,\ye,\neg
 fsub_s0b \xm,\ym,\xe,\ye,\neg
.endm

.section .text.qfp.faddfsub,"ax",%progbits
.balign 4
.thumb_func
qfp_fadd:
 ubfx r2,r0,#23,#8        @ extract exponents
 ubfx r3,r1,#23,#8
 cbz r3,0f                @ y 0/denormal?
10:
 cbz r2,1f                @ x 0/denormal?
20:
 cmp r3,#0xff             @ y Inf/NaN?
 beq 2f
 cmp r2,#0xff             @ x Inf/NaN?
 beq 3f

 lsls r1,#1               @ shift up y, test sign
 bcs fadd_m
 lsls r0,#1               @ shift up x, test sign
 bcs fadd_mp

fadd_pp:                  @ positive+positive
 movs r12,#1              @ insert implied 1:s
 bfi r0,r12,#24,#8
 bfi r1,r12,#24,#8
 subs r12,r2,r3           @ difference of exponents
 bmi 104f                 @ ye>xe?
 fadd_s0a r0,r1,r2,r3,0   @ fadd_pp is continued below

0:                        @ here y 0/denormal
 cmp r2,#0x80
 bge 30f                  @ x exponent large? treat y as zero
 cmp r2,#0
 beq 50f                  @ x also 0/denormal?
 lsr r12,r1,#31           @ save y sign
 lsls r1,#1               @ shift out y sign bit
 clz r3,r1                @ normalise y
 lsls r1,r3
 beq 30f                  @ y zero?
 lsrs r1,#8               @ shift back to Q23 position
 orr r1,r1,r12,lsl#31     @ restore sign
 rsb r3,#8                @ adjust exponent
 b 10b                    @ and continue, treating y as normal value

1:                        @ here x 0/denormal
 cmp r3,#0x80
 bge 40f                  @ y exponent large? treat x as zero
 lsr r12,r0,#31           @ save x sign
 lsls r0,#1               @ shift out x sign bit
 clz r2,r0                @ normalise x
 lsls r0,r2
 beq 40f                  @ x zero?
 lsrs r0,#8               @ shift back to Q23 position
 orr r0,r0,r12,lsl#31     @ restore sign
 rsb r2,#8                @ adjust exponent
 b 20b                    @ and continue, treating x as normal value

.balign 4
30:                       @ here x>>y
 cmp r2,#0xff
 it ne
 bxne r14                 @ x finite? return it
3:                        @ x Inf/NaN
 lsls r12,r0,#9           @ x NaN?
 it ne
 orrne r0,#0x00400000     @ return Inf or modified NaN
 bx r14

.balign 4
40:                       @ here y>>x
 mov r0,r1                @ by default return y
 cmp r3,#0xff             @ y finite? return it
 it ne
 bxne r14
 lsls r12,r0,#9           @ y NaN?
 it ne
 orrne r0,#0x00400000     @ return Inf or modified NaN
 bx r14

.balign 4
2:                        @ here y is Inf/NaN
 lsls r12,r1,#9           @ y NaN?
 itt ne
 orrne r0,r1,#0x00400000  @ return modified NaN
 bxne r14
                          @ here y Inf
 cmp r2,#0xff
 itt ne                   @ x finite?
 movne r0,r1
 bxne r14                 @ return y
 lsls r12,r0,#9           @ x NaN?
 itt ne
 orrne r0,#0x00400000     @ return modified NaN
 bxne r14
                          @ here x Inf, y Inf
 teq r0,r1
 it eq
 bxeq r14                 @ same signs? return it
 mov r0,#0xff000000       @ otherwise return NaN
 orr r0,#0x00c00000
 bx r14

.balign 4
50:                       @ here both x,y 0/denormal
 cmp r0,#0x80000000       @ -0+-0?
 itt eq
 cmpeq r1,#0x80000000     @ return -0
 bxeq r14
 cmp r0,#0
 it mi
 rsbmi r0,#0x80000000     @ x negative? make 2's complement
 cmp r1,#0
 it mi
 rsbmi r1,#0x80000000     @ y negative? make 2's complement
 adds r0,r1               @ direct addition
 it mi
 rsbmi r0,#0x80000000     @ convert back to sign-magnitude
 bx r14


 fadd_s0b r0,r1,r2,r3,0   @ continuation of fadd_pp
.balign 4
104:
 subs r12,r3,r2           @ here ye>xe: exchange macro paramters
 fadd_s0 r1,r0,r3,r2,0

.balign 4
fsub_m:
 lsls r1,#1               @ shift up y, test sign
 bcc fadd_mm              @ drop into addition
                          @ drop into fadd_mp

fadd_mp:                  @ negative+positive
 cmp r2,r3                @ which has larger magnitude?
 it eq
 cmpeq r0,r1
 blt fadd_mp_xlty
 fsub_s0 r0,r1,r2,r3,1    @ |x|>=|y|, subtract with negative result
.balign 4
fadd_mp_xlty:
 fsub_s0 r1,r0,r3,r2,0    @ |x|<|y|, subtract with positive result

.balign 4
fadd_m:
 lsls r0,#1               @ x negative?
 bcc fadd_pm
fadd_mm:                  @ negative+negative
 movs r12,#1              @ insert implied 1:s
 bfi r0,r12,#24,#8
 bfi r1,r12,#24,#8
 subs r12,r2,r3           @ difference of exponents
 bmi 4f                   @ ye>xe?
 fadd_s0 r0,r1,r2,r3,1
.balign 4
4:
 subs r12,r3,r2           @ here ye>xe: exchange macro paramters
 fadd_s0 r1,r0,r3,r2,1

.balign 4
.thumb_func
qfp_fsub:
 ubfx r2,r0,#23,#8        @ extract exponents
 ubfx r3,r1,#23,#8
 cbz r2,0f                @ x 0/denormal?
10:
 cbz r3,1f                @ y 0/denormal?
20:
 cmp r2,#0xff             @ x Inf/NaN?
 beq 2f
 cmp r3,#0xff             @ y Inf/NaN?
 beq 3f

 lsls r0,#1               @ shift up x, test sign
 bcs fsub_m
 lsls r1,#1               @ shift up y, test sign
 bcs fadd_pp              @ drop into addition
                          @ drop into fadd_pm

fadd_pm:                  @ positive+negative
 cmp r2,r3                @ which has larger magnitude?
 it eq
 cmpeq r0,r1
 blt fadd_pm_xlty
 fsub_s0a r0,r1,r2,r3,0   @ |x|>=|y|, subtract with positive result
                          @ fadd_pm is continued below

.balign 4
0:                        @ here x 0/denormal
 b 100f                   @ continue below

.balign 4
1:                        @ here y 0/denormal
 cmp r2,#0x80
 bge 40f                  @ x exponent large? treat y as zero
 lsr r12,r1,#31           @ save y sign
 lsls r1,#1               @ shift out y sign bit
 clz r3,r1                @ normalise y
 lsls r1,r3
 beq 40f                  @ y zero?
 lsrs r1,#8               @ shift back to Q23 position
 orr r1,r1,r12,lsl#31     @ restore sign
 rsb r3,#8                @ adjust exponent
 b 20b                    @ and continue, treating y as normal value

.balign 4
 fsub_s0b r0,r1,r2,r3,0   @ continuation of fadd_pm
.balign 4
fadd_pm_xlty:
 fsub_s0 r1,r0,r3,r2,1    @ |x|<|y|, subtract with negative result

.balign 4
100:
 cmp r3,#0x80
 bge 30f                  @ y exponent large? treat x as zero
 cmp r3,#0
 beq 50f                  @ y also 0/denormal?
 lsr r12,r0,#31           @ save x sign
 lsls r0,#1               @ shift out x sign bit
 clz r2,r0                @ normalise x
 lsls r0,r2
 beq 30f                  @ x zero?
 lsrs r0,#8               @ shift back to Q23 position
 orr r0,r0,r12,lsl#31     @ restore sign
 rsb r2,#8                @ adjust exponent
 b 10b                    @ and continue, treating x as normal value

.balign 4
30:                       @ here y>>x
 cmp r3,#0xff
 itt ne
 eorne r0,r1,#0x80000000
 bxne r14                 @ y finite? return -y
3:                        @ y Inf/NaN
 lsls r12,r1,#9           @ y NaN?
 mov r0,r1
 itt ne
 orrne r0,#0x00400000     @ return modified NaN
 bxne r14
 eor r0,#0x80000000       @ return -Inf
 bx r14

.balign 4
40:                       @ here x>>y
 cmp r2,#0xff             @ x finite? return it
 it ne
 bxne r14
 lsls r12,r0,#9           @ x NaN?
 it ne
 orrne r0,#0x00400000     @ return Inf or modified NaN
 bx r14

.balign 4
2:                        @ here x is Inf/NaN
 lsls r12,r0,#9           @ x NaN?
 itt ne
 orrne r0,#0x00400000     @ return modified NaN
 bxne r14
                          @ here x Inf
 cmp r3,#0xff
 it ne                    @ y finite?
 bxne r14                 @ return x
 lsls r12,r1,#9           @ y NaN?
 itt ne
 orrne r0,r1,#0x00400000  @ return modified NaN
 bxne r14
                          @ here x Inf, y Inf
 teq r0,r1
 it ne
 bxne r14                 @ differing signs? return x
 mov r0,#0xff000000       @ otherwise return NaN
 orr r0,#0x00c00000
 bx r14

.balign 4
50:                       @ here both x,y 0/denormal
 eor r1,#0x80000000       @ negate y
 cmp r0,#0x80000000       @ -0+-0?
 itt eq
 cmpeq r1,#0x80000000     @ return -0
 bxeq r14
 cmp r0,#0
 it mi
 rsbmi r0,#0x80000000     @ x negative? make 2's complement
 cmp r1,#0
 it mi
 rsbmi r1,#0x80000000     @ y negative? make 2's complement
 adds r0,r1               @ direct addition
 it mi
 rsbmi r0,#0x80000000     @ convert back to sign-magnitude
 bx r14

.section .text.qfp.fmul,"ax",%progbits
.balign 4
.thumb_func
qfp_fmul:
 eors r2,r0,r1
 ands r12,r2,#0x80000000  @ save sign of result
 ubfx r3,r1,#23,#8        @ y exponent
 cbz r3,fmul_y_0          @ y 0/denormal?
 cmp r3,#0xff
 beq fmul_y_inf           @ y Inf/Nan?
11:
 ubfx r2,r0,#23,#8        @ x exponent
 cbz r2,fmul_x_0          @ x 0/denormal?
 cmp r2,#0xff
 beq fmul_x_inf           @ x Inf/Nan?
10:

 add r3,r3,r2             @ result exponent+254
 lsls r0,#9               @ x Q32 with no implied 1
 lsls r1,#8               @ y Q31
 orr r1,r1,#0x80000000    @ ... with implied 1
 umull r2,r0,r0,r1        @ (x-1)y Q63
 adds r0,r1               @ xy Q63
 bcc 0f                   @ will result need to be shifted down one place?

 subs r3,#126             @ correct exponent (implied 1 present)
 ble 5f                   @ result 0/denormal?
 cmp r3,#255
 bge 4f                   @ result Inf?
 orrs r2,r2,r0,lsl#24
 beq 1f                   @ sticky bits 0?
 lsrs r0,#9               @ pack result
 adc r0,r0,r3,lsl#23      @ rounding and exponent
 orr r0,r0,r12            @ sign
 bx r14

.balign 4
fmul_y_0:                 @ here y 0/denormal
 ubfx r2,r0,#23,#8        @ x exponent
 cmp r2,#0xff
 beq 111f                 @ x Inf/NaN?
 bics r1,r1,#0x80000000   @ clear sign
 beq 8f                   @ y=0, x finite, so result is zero
 clz r3,r1                @ y denormal: make normal...
 subs r3,#8
 lsls r1,r3
 rsb r3,#1                @ ... and fix exponent
 b 11b

.balign 4
fmul_x_0:                 @ x 0/denormal
 bics r0,#0x80000000
 beq 8f                   @ x=0? 0 result
 clz r2,r0                @ x denormal: make normal...
 subs r2,#8
 lsls r0,r2
 rsb r2,#1                @ ... and fix exponent
 b 10b

.balign 4
1:                        @ sticky bits 0
 lsrs r0,#9               @ pack result
 adc r0,r0,r3,lsl#23      @ rounding and exponent
 it cs
 biccs r0,#1              @ round to even
 orr r0,r0,r12            @ sign
 bx r14

.balign 4
0:                        @ result must be shifted down one place
 subs r3,#128             @ correct exponent (implied 1 absent)
 ble 3f                   @ result 0/denormal?
 cmp r3,#254
 bge 4f                   @ result Inf?
 orrs r2,r2,r0,lsl#25
 beq 2f                   @ sticky bits 0?
 lsrs r0,#8               @ pack result
 adc r0,r0,r3,lsl#23
 orr r0,r0,r12
 bx r14

.balign 4
2:                        @ sticky bits 0
 lsrs r0,#8 @ pack result
 adc r0,r0,r3,lsl#23
 it cs
 biccs r0,#1              @ round to even
 orr r0,r0,r12
 bx r14

.balign 4
4:                        @ return +/- Inf
 mov r0,#0x7f800000
 orr r0,r0,r12
 bx r14

.balign 4
5:                        @ 0/denormal result
 subs r3,#1               @ get shift
 subs r3,#0               @ set carry
 rrxs r0,r0               @ shift in implied 1
 rrxs r2,r2
3:
 adds r3,#25              @ will become 0?
 ble 8f
 lsls r1,r0,r3            @ double-length shift
 orrs r2,r2,r1
 rsb r3,#33
 lsrs r0,r3               @ sticky bits
 bcc 9f                   @ not a potential tie?
 adds r0,#1               @ rounding...
 cmp r2,#0
 it eq
 biceq r0,#1              @ ... to even
9:
 orr r0,r0,r12
 bx r14

.balign 4
8:
 mov r0,r12               @ 0 result with sign
 bx r14

.balign 4
111:                      @ here y 0/denormal, x Inf/NaN
 lsls r2,r0,#9
 bne 2f                   @ x NaN?
 bics r1,r1,#0x80000000
 bne 3f                   @ y<>0?
4:
 mov r0,#0xff000000       @ y=0, x Inf: return NaN
 orr r0,r0,#0x00c00000
 bx r14

.balign 4
2:
 orr r0,r0,#0x00400000    @ return modified x NaN
 bx r14

.balign 4
fmul_y_inf:               @ here y Inf/NaN
 lsls r3,r1,#9
 beq 0f                   @ y Inf?
 orr r0,r1,#0x00400000    @ return modified y NaN
 bx r14
.balign 4
0:                        @ here y Inf
 ubfx r2,r0,#23,#8        @ x exponent
 cmp r2,#0xff
 beq 1f                   @ x Inf/NaN?
 bics r0,#0x80000000
 beq 4b                   @ y=Inf, x=0? return NaN
3:
 orr r0,r12,#0x7f800000   @ x<>0, y Inf: return signed Inf
 bx r14

.balign 4
1:                        @ here y Inf, x Inf/NaN
 lsls r2,r0,#9
 beq 3b                   @ x Inf? return Inf
 b 2f                     @ x NaN: return modified x NaN

.balign 4
fmul_x_inf:               @ x Inf/NaN, y normal
 lsls r2,r0,#9
 beq 3b                   @ x Inf? return Inf
2: @ x NaN
 orr r0,r0,#0x00400000    @ return modified x NaN
 bx r14

@ This macro does the mantissa division operation in two cases.
@ When ds=0, x>=y and quotient of mantissas is   1..2.
@ When ds=1, x< y and quotient of mantissas is 0.5..1.
@ In the first case the result is Q24; in the second, Q25.

.macro div_s0 ds
 lsls r0,#8               @ x Q31
 lsrs r2,r1,#7+\ds        @ approximation to y Q16|Q15
 udiv r3,r0,r2            @ first quotient approximation Q31/Q16=Q15 1..2
 lsls r0,#7+\ds           @ x Q38|Q39
 mls r0,r1,r3,r0          @ remainder Q23*Q15=Q38
 lsls r0,#2-\ds           @ remainder Q40
 sdiv r2,r0,r2            @ second part of quotient Q40/Q16|Q15=Q24|Q25
 lsls r0,#7+\ds           @ Q47|Q48
 mls r0,r1,r2,r0          @ updated remainder Q23*Q24|Q25=Q47|Q48
 add r2,r2,r3,lsl#9       @ add quotient parts for Q24|Q25 quotient
 cmp r0,r1                @ (signed) remainder out of range?
 bhs 4f
.endm

.section .text.qfp.fdiv,"ax",%progbits
.balign 4
.thumb_func
qfp_fdiv:
 eor r12,r0,r1
 lsr r12,#31              @ sign bit of result in r12b0
 ubfx r2,r0,#23,#8        @ extract x exponent
 ubfx r3,r1,#23,#8        @ extract y exponent

 cbz r2,fdiv_x_0          @ x 0/denormal?
 cmp r2,#0xff
 beq fdiv_x_inf           @ x Inf/NaN?
10:

 cbz r3,fdiv_y_0          @ y 0/denormal?
 cmp r3,#0xff
 beq fdiv_y_inf           @ y Inf/NaN?
11:

 subs r2,r3               @ difference in exponents
 movs r3,#1
 bfi r0,r3,#23,#9         @ insert implied 1:s
 bfi r1,r3,#23,#9
 cmp r0,r1                @ mantissa of x < mantissa of y?
 blo 1f
                          @ x>=y
 adds r2,#126             @ add exponent offset
 cmp r2,#0xfe             @ will result be 0/denormal/Inf?
 bhs 50f
 add r12,r2,r12,lsl#8     @ add in sign bit
 div_s0 0                 @ perform division
6:
 lsrs r0,r2,#1            @ get rounding bit
 adc r0,r0,r12,lsl#23     @ round and pack
 bx r14                   @ there can be no rounding tie if the result is normal (exercise for the reader)

.balign 4
fdiv_x_0:                 @ x 0/denormal
 bics r0,#0x80000000
 beq 30f                  @ x=0?
 clz r2,r0                @ x denormal: make normal and fix exponent
 subs r2,#8
 lsls r0,r2
 rsb r2,#1
 b 10b

.balign 4
fdiv_y_0:                 @ y 0/denormal
 bics r1,#0x80000000
 beq 70f                  @ y=0, x finite? return +/-Inf
 clz r3,r1                @ y denormal: make normal and fix exponent
 subs r3,#8
 lsls r1,r3
 rsb r3,#1
 b 11b

.balign 4
1:                        @ x<y
 adds r2,#125             @ add exponent offset
 cmp r2,#0xfe             @ will result be 0/denormal/Inf?
 bhs 60f
 add r12,r2,r12,lsl#8     @ add in sign bit
 div_s0 1                 @ perform division
6:
 lsrs r0,r2,#1            @ get rounding bit
 adc r0,r0,r12,lsl#23     @ round and pack
 bx r14

@ Here we trap the case where the remainder is out of range
@ because the (approximate) division quotient is not correct.
@ By this point the x<y and x>=y cases can be treated together.
.balign 4
4:
 blt 5f                   @ negative remainder?
7:
 adds r2,#1               @ increment quotient
 subs r0,r1               @ adjust remainder
 cmp r0,r1                @ finished?
 blo 6b
 b 7b
.balign 4
5:
 subs r2,#1               @ decrement quotient
 adds r0,r1               @ adjust remainder
 bpl 6b                   @ finished?
 b 5b

.balign 4
50:                       @ x>=y, denormal or infinte result
 bge 70f                  @ infinite result?
 rsbs r2,#1               @ shift for denormal result
 add r12,r2,r12,lsl#31    @ save in r12
 div_s0 0                 @ perform division
 b 6f

.balign 4
60:                       @ x<y, denormal or infinte result
 bge 70f                  @ infinite result?
 rsbs r2,#1               @ shift for denormal result
 add r12,r2,r12,lsl#31    @ save in r12
 div_s0 1                 @ perform division
6:
 rsb r3,r12,#33           @ complementary shift
 lsls r3,r2,r3            @ bits about to be shifted off (less one)
 orrs r0,r3               @ OR sticky bits with remainder
 beq 8f                   @ sticky bits 0?
 lsrs r0,r2,r12           @ shift down result
 bic r12,#0xff
 adc r0,r0,r12            @ round and pack
 bx r14

.balign 4
8:                        @ sticky bits 0: possibly tied rounding case
 lsrs r0,r2,r12           @ shift down result
 bic r12,#0xff
 adc r0,r0,r12
 it cs                    @ tied?
 biccs r0,#1              @ round to even
 bx r14

@ Here we trap the case where the remainder is out of range
@ when the result is denormal.
.balign 4
4:
 blt 5f                   @ negative remainder?
7:
 adds r2,#1               @ increment quotient
 subs r0,r1               @ adjust remainder
 cmp r0,r1                @ finished?
 blo 6b
 b 7b
.balign 4
5:
 subs r2,#1               @ decrement quotient
 adds r0,r1               @ adjust remainder
 bpl 6b                   @ finished?
 b 5b

.balign 4
70:                       @ return signed infinity
 lsl r0,r12,#31
 orr r0,#0x7f800000
 bx r14

.balign 4
30:                       @ x zero
 lsls r3,r1,#1            @ test y
 beq 31f                  @ 0/0? NaN
 cmp r3,#0xff000000
 bls 32f                  @ y not NaN?
33:
 orr r0,r1,#0x00400000    @ return modified y NaN
 bx r14

.balign 4
31:                       @ return NaN
 mov r0,#0xff000000
 orr r0,r0,#0x00c00000
 bx r14

.balign 4
fdiv_x_inf:               @ x Inf/NaN
 lsls r2,r0,#9
 beq 0f                   @ x Inf?
 orr r0,r0,#0x00400000    @ return modified x NaN
 bx r14
.balign 4
0:                        @ x Inf
 cmp r3,#0xff
 beq 1f                   @ y Inf/NaN?
 mov r0,#0x7f800000       @ x Inf y finite: return +/-Inf
 orr r0,r0,r12,lsl#31
 bx r14

.balign 4
1:                        @ y Inf/NaN
 lsls r3,r1,#9
 bne 33b                  @ y NaN?
 b 31b                    @ x Inf y Inf: return NaN

.balign 4
fdiv_y_inf:               @ y Inf/NaN
 lsls r3,r1,#9
 bne 33b                  @ y NaN?
32:
 lsl r0,r12,#31           @ return +/-0
 bx r14

@ Square root is calculated using an initial 8-bit approximation from a
@ look-up table which is then refined twice.  The look-up table is split
@ into two parts, depending on whether the exponent is odd or even. Seven
@ mantissa bits are used in the look-up. The final remainder is checked
@ and used to correct the approximate result if necessary.

.section .text.qfp.fsqrt,"ax",%progbits
.balign 4
rsqrt_t0:
.byte 181,182,183,183,184,185,186,186,187,188,188,189,190,190,191,192
.byte 192,193,194,194,195,196,196,197,198,198,199,200,200,201,201,202
.byte 203,203,204,205,205,206,206,207,208,208,209,210,210,211,211,212
.byte 213,213,214,214,215,216,216,217,217,218,219,219,220,220,221,221
.byte 222,223,223,224,224,225,225,226,227,227,228,228,229,229,230,230
.byte 231,232,232,233,233,234,234,235,235,236,237,237,238,238,239,239
.byte 240,240,241,241,242,242,243,243,244,244,245,246,246,247,247,248
.byte 248,249,249,250,250,251,251,252,252,253,253,254,254,255,255,255
rsqrt_t1:
.byte 128,129,129,130,130,131,131,132,132,133,133,134,134,135,135,136
.byte 136,136,137,137,138,138,139,139,140,140,141,141,142,142,142,143
.byte 143,144,144,145,145,146,146,146,147,147,148,148,149,149,149,150
.byte 150,151,151,152,152,152,153,153,154,154,155,155,155,156,156,157
.byte 157,157,158,158,159,159,159,160,160,161,161,161,162,162,163,163
.byte 163,164,164,165,165,165,166,166,166,167,167,168,168,168,169,169
.byte 170,170,170,171,171,171,172,172,173,173,173,174,174,174,175,175
.byte 175,176,176,177,177,177,178,178,178,179,179,179,180,180,180,181

.balign 4
.thumb_func
qfp_fsqrt:
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,#0x00800000       @ insert implied 1
 asrs r1,r0,#24
 bmi 1f                   @ negative argument?
 bcs 2f                   @ odd exponent?
 beq 3f                   @ zero exponent?
9:                        @ here we have even exponent (e.g. 2<=x<4 Q22)
 add r12,r1,#0x3e         @ exponent 80->7f, 82->80 etc.
 ubfx r3,r2,#16,#7        @ extract 7 mantissa bits
 adr r0,rsqrt_t0
 ldrb r0,[r0,r3]          @ first approximation to result Q7
 mul r3,r0,r0             @ approximation squared Q14
 sub r1,r2,r3,lsl#8       @ first remainder Q22
 lsls r1,#14              @ Q36
 sdiv r1,r1,r0            @ first correction*2 Q36/Q7=Q29
 lsls r0,#7               @ Q14
 add r0,r0,r1,asr#16      @ add in correction Q14
 mul r3,r0,r0             @ Q28
 rsb r1,r3,r2,lsl#6       @ second remainder Q28
 lsls r1,#9               @ Q37
 sdiv r1,r1,r0            @ second correction*2 Q37/Q14=Q23
 add r0,r1,r0,lsl#10      @ add in correction Q24
 mul r3,r0,r0             @ Q48
 rsbs r3,r3,r2,lsl#26     @ remainder Q48
 bmi 4f                   @ when approximation is not exact, remainder happens always to be negative
 lsrs r0,#1               @ Q23 result
 adc r0,r0,r12,lsl#23     @ pack and round (rounding can never tie)
 bx r14

.balign 4
4:                        @ here approximation was too high
 subs r0,#1               @ adjust result
 mul r3,r0,r0
 rsbs r3,r3,r2,lsl#26     @ recompute remainder Q48
 bmi 5f                   @ still incorrect?
 lsrs r0,#1               @ pack and round
 adc r0,r0,r12,lsl#23
 bx r14
.balign 4
5:
 subs r0,#1               @ worst-case error is 2ulps at Q24
 lsrs r0,#1               @ pack and round
 adc r0,r0,r12,lsl#23
 bx r14

.balign 4
2:                        @ here we have odd exponent (e.g. 1<=x<2 Q23)
 cmp r1,#0x7f
 beq 6f                   @ +Inf/NaN?
 add r12,r1,#0x3f         @ exponent 7f->7f, 81->80 etc.
 ubfx r3,r2,#16,#7        @ extract 7 mantissa bits
 adr r0,rsqrt_t1
 ldrb r0,[r0,r3]          @ first approximation to result Q7
 mul r3,r0,r0             @ approximation squared Q14
 sub r1,r2,r3,lsl#9       @ first remainder Q23
 lsls r1,#13              @ Q36
 sdiv r1,r1,r0            @ first correction*2 Q36/Q7=Q29
 lsls r0,#7               @ Q14
 add r0,r0,r1,asr#16      @ add in correction Q14
 mul r3,r0,r0             @ Q28
 rsb r1,r3,r2,lsl#5       @ second remainder Q28
 lsls r1,#9               @ Q37
 sdiv r1,r1,r0            @ second correction*2 Q37/Q14=Q23
 add r0,r1,r0,lsl#10      @ add in correction Q24
 mul r3,r0,r0             @ Q48
 rsbs r3,r3,r2,lsl#25     @ remainder Q48
 bmi 4f                   @ when approximation is not exact, remainder happens always to be negative
 lsrs r0,#1               @ Q23 result
 adc r0,r0,r12,lsl#23     @ pack and round (rounding can never tie)
 bx r14

.balign 4
4:                        @ here approximation was too high
 subs r0,#1               @ adjust result
 mul r3,r0,r0
 rsbs r3,r3,r2,lsl#25     @ recompute remainder Q48
 bmi 5f                   @ still incorrect?
 lsrs r0,#1               @ pack and round
 adc r0,r0,r12,lsl#23
 bx r14
.balign 4
5:
 subs r0,#1               @ worst-case error is 2ulps at Q24
 lsrs r0,#1               @ pack and round
 adc r0,r0,r12,lsl#23
 bx r14

.balign 4
6:                        @ +Inf/NaN
 lsls r1,r0,#9
 bne 7f                   @ NaN? return modified NaN
8:
 bx r14                   @ return +Inf

.balign 4
3:                        @ 0/denormal
 cmp r0,#0
 beq 8b                   @ +0? return it
 clz r1,r0
 subs r2,r1,#8
 lsl r2,r0,r2             @ normalise
 rsb r1,#9                @ adjust exponent
 lsrs r1,#1
 bcc 9b                   @ even exponent?
 b 2b                     @ odd exponent

.balign 4
1:                        @ negative argument
 cmp r0,#0x80000000       @ -0? return it
 beq 8b
 cmn r0,#0x00800000       @ NaN? return modified NaN
 bhi 7f
 mov r0,#0xff000000       @ finite negative argument: return NaN
7:
 orr r0,#0x00c00000
 bx r14




@ After range reduction, exp(x) for 0<x<ln 2 is calculated by writing x=a+e
@ where |e|<~3.2^-9. Then exp(a) is obtained from a look-up table and
@ exp(e) is calculated using a power series. The result is exp(x)=exp(a)*exp(e).

@ The first of these tables gives an approximation y~exp(x) Q8, while the
@ second table gives ln y Q29.

.section .text.qfp.fexp,"ax",%progbits
.balign 4
exp_e0:
.short 0x0100, 0x0102, 0x0104, 0x0106, 0x0108, 0x010a, 0x010c, 0x010e
.short 0x0111, 0x0113, 0x0115, 0x0117, 0x0119, 0x011b, 0x011e, 0x0120
.short 0x0122, 0x0124, 0x0127, 0x0129, 0x012b, 0x012e, 0x0130, 0x0132
.short 0x0135, 0x0137, 0x013a, 0x013c, 0x013f, 0x0141, 0x0144, 0x0146
.short 0x0149, 0x014b, 0x014e, 0x0151, 0x0153, 0x0156, 0x0158, 0x015b
.short 0x015e, 0x0161, 0x0163, 0x0166, 0x0169, 0x016c, 0x016f, 0x0172
.short 0x0174, 0x0177, 0x017a, 0x017d, 0x0180, 0x0183, 0x0186, 0x0189
.short 0x018d, 0x0190, 0x0193, 0x0196, 0x0199, 0x019c, 0x01a0, 0x01a3
.short 0x01a6, 0x01a9, 0x01ad, 0x01b0, 0x01b3, 0x01b7, 0x01ba, 0x01be
.short 0x01c1, 0x01c5, 0x01c8, 0x01cc, 0x01d0, 0x01d3, 0x01d7, 0x01db
.short 0x01de, 0x01e2, 0x01e6, 0x01ea, 0x01ed, 0x01f1, 0x01f5, 0x01f9
.short 0x01fd, 0x0201

.balign 4
exp_dt0:
.long 0x00000000, 0x003fc055, 0x007f02a3, 0x00bdc8d8, 0x00fc14d8, 0x0139e87c, 0x0177458f, 0x01b42dd7
.long 0x020eb307, 0x024a7ec6, 0x0285db98, 0x02c0cb15, 0x02fb4ece, 0x03356849, 0x038bca92, 0x03c4e0ee
.long 0x03fd9226, 0x0435df9f, 0x04899bc6, 0x04c0f5c6, 0x04f7f0ac, 0x0549b979, 0x057fcc1c, 0x05b583fa
.long 0x06057028, 0x063a4a38, 0x0688ef08, 0x06bcf254, 0x070a5a15, 0x073d8d54, 0x0789c1dc, 0x07bc2b73
.long 0x08073623, 0x0838dc30, 0x0882c5fd, 0x08cc0697, 0x08fc7fcf, 0x0944ad0a, 0x0974715d, 0x09bb9331
.long 0x0a021843, 0x0a480340, 0x0a764b99, 0x0abb3b8c, 0x0aff9838, 0x0b43640f, 0x0b86a171, 0x0bc952b0
.long 0x0bf57c1e, 0x0c3748cd, 0x0c788f44, 0x0cb951a1, 0x0cf991f6, 0x0d39524b, 0x0d789499, 0x0db75acf
.long 0x0e0a4ff9, 0x0e47fbe4, 0x0e8531d7, 0x0ec1f393, 0x0efe42cd, 0x0f3a2131, 0x0f8947b0, 0x0fc4251b
.long 0x0ffe9704, 0x10389ef0, 0x10855c88, 0x10be72e4, 0x10f7241d, 0x11422025, 0x1179eabc, 0x11c3b81f
.long 0x11faa34c, 0x12434b6f, 0x12795e13, 0x12c0e9ed, 0x1307d733, 0x133ca2ba, 0x138280fe, 0x13c7c800
.long 0x13fb5b85, 0x143f9fe3, 0x148353d2, 0x14c679b0, 0x14f87a3f, 0x153aad06, 0x157c57f3, 0x15bd7d31
.long 0x15fe1edb, 0x163e3f00

.balign 4
.thumb_func
qfp_fexp:
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,#0x00800000       @ insert implied 1
 asrs r1,r0,#23           @ extract offset exponent e+127
 bmi 1f                   @ negative argument?
 subs r1,#0x7e            @ e+1
 bmi 5f                   @ e+1<0 (x<0.5)? no need for range reduction
 cmp r1,#7
 bge 3f                   @ e+1>=7 (x>=64)? potential overflow/Inf/NaN
10:
 lsl r0,r2,r1             @ x now Q24, x<89
 asrs r1,r0,#15           @ Q9
 movw r2,#0xb8aa          @ 1/ln2 Q15, underestimate
 muls r1,r1,r2            @ x/ln2 Q24
 asr r12,r1,#24           @ first result exponent estimate in r12
 movlong r1,0x162E42FF    @ ln2 Q29
 mul r1,r12
 rsb r0,r1,r0,lsl#5       @ residual Q29 ~<ln2
6:
 asrs r1,r0,#22           @ x Q7
 adc r1,#0                @ rounding
 adr r3,exp_e0
 ldrh r3,[r3,r1,lsl#1]    @ y~exp(x) Q8
 adr r2,exp_dt0
 ldr r2,[r2,r1,lsl#2]     @ ln y Q29
 subs r0,r2               @ residual r 0xffd... to 0x003... Q29
 asrs r1,r0,#6            @ r Q23
 mul r2,r1,r1             @ r^2 Q46
 add r0,r0,r2,lsr#18      @ r+r^2/2 Q29
 mul r1,r0,r3             @ y*(r+r^2/2) Q37
 asrs r1,#14              @ y*(r+r^2/2) Q23
 adc r0,r1,r3,lsl#15      @ y*(1+r+r^2/2) Q23 with rounding
 lsls r1,r0,#8            @ addition can overflow into bit 24
 ittt cc
 addcc r12,#0x7e          @ if not, result has exponent r12+0x7f
 addcc r0,r0,r12,lsl#23
 bxcc r14
 lsrs r0,#1               @ if so, shift down one place
 add r12,#0x7f            @ exponent is r12+0x80
 adc r0,r0,r12,lsl#23     @ and pack with rounding
 bx r14

.balign 4
5:                        @ here x<0.5
 adds r1,#5               @ e+6
 bmi 12f                  @ will be shifting down?
 lsl r0,r2,r1             @ x Q29
 movs r12,#0              @ zero result exponent
 b 6b
.balign 4
12:
 rsbs r1,#0
 lsr r0,r2,r1             @ x Q29
 movs r12,#0              @ zero result exponent
 b 6b

.balign 4
3:
 beq 5f                   @ do we need to check if we are on the overflow boundary?
 cmp r0,#0x7f800000       @ NaN? return modified NaN
 bgt 4f
9:
 mov r0,#0x7f800000       @ return +Inf
 bx r14
.balign 4
4:
 orr r0,#0x00400000
 bx r14
.balign 4
5:
 movlong r3,0x42b17217    @ overflow threshold
 cmp r0,r3
 bls 10b                  @ result finite?
 b 9b                     @ result is infinite

.balign 4
1:                        @ negative argument here
 and r1,#0xff             @ remove sign
 rsbs r2,#0               @ make mantissa negative
 subs r1,#0x7e            @ e+1
 bmi 15f                  @ x>-0.5?
 cmp r1,#7
 bge 7f                   @ potential underflow?
11:
 lsl r0,r2,r1             @ x now Q24, x<89
 asrs r1,r0,#15           @ Q9
 movw r2,#0x5c55          @ 1/ln2 Q14, underestimate
 muls r1,r1,r2            @ x/ln2 Q23
 asr r12,r1,#23           @ first result exponent estimate in r12
 movlong r1,0x162E42FF    @ ln2 Q29
 mul r1,r12
 rsb r0,r1,r0,lsl#5       @ residual Q29 ~<ln2
6:
 asrs r1,r0,#22           @ x Q7
 adc r1,#0                @ rounding
 adr r3,exp_e0
 ldrh r3,[r3,r1,lsl#1]    @ y~exp(x) Q8
 adr r2,exp_dt0
 ldr r2,[r2,r1,lsl#2]     @ ln y Q29
 subs r0,r2               @ residual r 0xffd... to 0x003... Q29
 asrs r1,r0,#6            @ r Q23
 mul r2,r1,r1             @ r^2 Q46
 add r0,r0,r2,lsr#18      @ r+r^2/2 Q29
 mul r1,r0,r3             @ y*(r+r^2/2) Q37
 adds r12,#0x7e
 bmi 8f                   @ result denormal or zero?
 asrs r1,#7               @ Q30
 adcs r0,r1,r3,lsl#22     @ y*(1+r+r^2/2) Q30
 lsls r2,r0,#1            @ Q31
 bpl 9f                   @ result mantissa is <1 or >=2
 lsrs r0,#7               @ pack and round
 adc r0,r0,r12,lsl#23
 bx r14
.balign 4
9:
 bcc 10f                  @ result mantissa <1?
 lsrs r0,#8               @ result mantissa >=2: pack and round with extra shift
 add r12,#1               @ and increment exponent
 adc r0,r0,r12,lsl#23
 bx r14
.balign 4
10:
 subs r12,#1              @ and decrement exponent
 lsrs r0,#6               @ result mantissa <1: pack and round with one less shift
 adc r0,r0,r12,lsl#23
 bx r14

.balign 4
15:                       @ small negative argument (x>-0.5)
 adds r1,#5               @ e+6
 bmi 2f                   @ will be shifting down?
 lsl r0,r2,r1             @ x Q29
 movlong r1,0x162E42FF    @ ln2 Q29
 add r0,r1                @ x+ln2 Q29
 mvn r12,#0               @ result exponent=-1
 b 6b

.balign 4
2:
 rsbs r1,#0
 asr r0,r2,r1             @ x Q29
 movlong r1,0x162E42FF    @ ln2 Q29
 add r0,r1                @ x+ln2 Q29
 mvn r12,#0               @ result exponent=-1
 b 6b

.balign 4
7:                        @ large negative argument
 movlong r3,0xc2d00000    @ approximate underflow threshold
 cmp r0,r3
 bls 11b                  @ not below threshold?
 cmp r0,#0xff800000
 bgt 4b                   @ NaN? return modified NaN
 movs r0,#0               @ large negative argument: return 0
 bx r14

.balign 4
8:                        @ denormal result (or just normal)
 asrs r1,#13              @ Q24
 add r0,r1,r3,lsl#16      @ Q24
 rsb r12,#1               @ shift down according to exponent
 lsrs r0,r12
 adc r0,#0                @ rounding
 bx r14

@ The ln function is calculalated by adding ln 2 times the exponent of the argument
@ to the logarithm of the mantissa. The logarithm of the mantissa m is calculated
@ by estimating a~1/m, evaluating ln a using a look-up table and ln(am), which
@ is approximately ln(1), using a power series. The result is then ln m = ln (am) - ln a.

@ This table is -ln(x) Q31 x=1..2.

.section .text.qfp.fln,"ax",%progbits
.balign 4
ln_dt0:
.long 0x00000000,0xff00fead,0xfe03f575,0xfd08dc9f,0xfc0fac9e,0xfb185e12,0xfa22e9c2,0xf92f48a4
.long 0xf83d73d0,0xf74d6489,0xf65f1435,0xf5727c60,0xf48796bb,0xf39e5d15,0xf2b6c963,0xf1d0d5b8
.long 0xf0ec7c49,0xf009b767,0xef288183,0xee48d52b,0xed6aad08,0xec8e03e0,0xebb2d493,0xead91a1c
.long 0xea00cf8f,0xe929f019,0xe85476fe,0xe7805f9b,0xe6ada563,0xe5dc43e0,0xe50c36b1,0xe43d798c
.long 0xe3700838,0xe2a3de95,0xe1d8f892,0xe10f5234,0xe046e793,0xdf7fb4d7,0xdeb9b63b,0xddf4e80d
.long 0xdd3146a9,0xdc6ece7d,0xdbad7c08,0xdaed4bd8,0xda2e3a8a,0xd97044cb,0xd8b36755,0xd7f79ef3
.long 0xd73ce87c,0xd68340d5,0xd5caa4f2,0xd51311d1,0xd45c8481,0xd3a6fa1b,0xd2f26fc3,0xd23ee2ac
.long 0xd18c5013,0xd0dab540,0xd02a0f89,0xcf7a5c4c,0xcecb98f2,0xce1dc2f2,0xcd70d7c8,0xccc4d4fe
.long 0xcc19b827,0xcb6f7ede,0xcac626cb,0xca1dad9b,0xc9761108,0xc8cf4ed3,0xc82964c5,0xc78450b0
.long 0xc6e01071,0xc63ca1e9,0xc59a0304,0xc4f831b5,0xc4572bf6,0xc3b6efcb,0xc3177b3c,0xc278cc5b
.long 0xc1dae141,0xc13db80b,0xc0a14ee1,0xc005a3ef,0xbf6ab569,0xbed08189,0xbe370690,0xbd9e42c3
.long 0xbd06346f,0xbc6ed9e9,0xbbd83187,0xbb4239aa,0xbaacf0b4,0xba185511,0xb984652e,0xb8f11f82
.long 0xb85e8286,0xb7cc8cba,0xb73b3ca1,0xb6aa90c6,0xb61a87b6,0xb58b2005,0xb4fc584b,0xb46e2f24
.long 0xb3e0a333,0xb353b31c,0xb2c75d8a,0xb23ba12b,0xb1b07cb3,0xb125eed8,0xb09bf656,0xb01291ed
.long 0xaf89c05f,0xaf018074,0xae79d0f8,0xadf2b0b8,0xad6c1e89,0xace61941,0xac609fb9,0xabdbb0d0
.long 0xab574b67,0xaad36e62,0xaa5018a9,0xa9cd4929,0xa94afecf,0xa8c9388e,0xa847f55b,0xa7c7342f
.long 0xa746f404

.macro ln_s0              @ calculate ln(x) up to quadratic term in first approximation error x in r2 Q24 .5<=x<1
 movs r0,#0
 movt r0,#0x807f          @ 0x807f0000~1 Q31
 udiv r0,r0,r2            @ y~1/x Q7
 adr r3,ln_dt0-512        @ offset as 128<y<=256
 ldr r3,[r3,r0,lsl#2]     @ -ln y Q31
 muls r0,r2,r0            @ xy=1+a Q31, |a|<~2^-9
 sub r0,#0x80000000       @ a Q31
 add r3,r0                @ a-ln y Q31
 asrs r0,#8               @ a Q23
 muls r0,r0,r0            @ a^2 Q46
 sub r0,r3,r0,lsr#16      @ a-a^2/2-ln y Q31 ~ ln(1+a)-ln y = ln((1+a)/y) = ln x
.endm

.macro ln_s1              @ calculate ln(x) up to cubic term in first approximation error x in r2 Q24 .5<=x<1
 movs r0,#0
 movt r0,#0x807f          @ 0x807f0000~q Q31
 udiv r0,r0,r2            @ y~1/x Q7
 adr r3,ln_dt0-512        @ offset as 128<y<=256
 ldr r3,[r3,r0,lsl#2]     @ -ln y Q31
 muls r0,r2,r0            @ xy=1+a Q31, |a|<~2^-9
 sub r0,#0x80000000       @ a Q31
 add r3,r0                @ a-ln y Q31
 asrs r0,#8               @ a Q23
 mul r2,r0,r0             @ a^2 Q46
 lsrs r2,#16              @ a^2 Q30
 subs r3,r3,r2            @ a-a^2/2-ln y Q31
 muls r2,r0,r2            @ a^3 Q53
 add r2,r2,r2,asr#2       @ ~4/3 a^3 Q53
 add r0,r3,r2,asr#24      @ a-a^2/2+a^3/3-ln y Q31 ~ ln(1+a)-ln y = ln((1+a)/y) = ln x
.endm

.balign 4
.thumb_func
qfp_fln:
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,#0x00800000       @ insert implied 1
 asrs r1,r0,#23           @ extract offset exponent
 bmi 1f                   @ negative argument?
 beq 2f                   @ zero/denormal argument?
 subs r1,#0x7e            @ e+1
 bmi 5f                   @ x<.5?
 cmp r1,#2
 blo 6f                   @ .5<=x<2?
 cmp r1,#0x81             @ x>=2
 beq 8f                   @ +Inf/NaN?
 ln_s0                    @ calculate ln of mantissa
 movlong r2,0x0162E430    @ ln2 Q25
 muls r1,r1,r2            @ (e+1)*ln2
 add r0,r1,r0,asr#6       @ result Q25
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x7c             @ offset exponent obtained from normalisation
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
5: @ x<.5
 ln_s0                    @ calculate ln of mantissa
 movlong r2,0xFE9D1BD0    @ ln2 Q25
 muls r1,r1,r2            @ (e+1)*ln2
 sub r0,r1,r0,asr#6       @ negated result Q25
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x17c            @ offset exponent obtained from normalisation + sign bit
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
6: @ .5<=x<2
 subs r3,r0,#0x3f800000   @ x-1 Q23 in case where 1<=x<2
 bmi 10f                  @ x<1?
 cmp r3,#0x00040000
 blt 7f                   @ x<2^-5?
 ln_s1                    @ calculate ln of mantissa
 movlong r2,0x58B90BFC    @ ln2 Q31
 add r0,r2
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x76             @ offset exponent obtained from normalisation
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
7:                        @ x=1+a, a<2^-5 in r3 Q23
 lsls r3,#6               @ a Q29<2^-5
 clz r1,r3                @ normalisation shift r1>=8
 lsls r0,r3,r1            @ now normalised to 1..2 Q31 = a Q(29+r1)
 beq 11f                  @ a=0? ln(1)=0
 umull r12,r2,r0,r3       @ a^2 Q(58+r1)<2^-10
 sub r0,r0,r2,lsl#2       @ a-a^2/2 Q(29+r1)
 lsrs r3,#9               @ a Q20<2^-5
 lsrs r2,#8               @ a^2 Q(18+r1)<2^-10
 mul r12,r2,r3            @ a^3 Q(38+r1)<2^-15
 add r12,r12,r12,lsr#2
 add r12,r12,r12,lsr#4
 add r12,r12,r12,lsr#8    @ 4/3 a^3 Q(38+r1)<2^-14
 add r0,r0,r12,lsr#11     @ a-a^2/2+a^3/3 Q(29+r1)
 lsrs r12,#16             @ 4/3 a^3 Q(22+r1)<2^-14
 mul r12,r12,r3           @ 4/3 a^4 Q(42+r1)<2^-19
 add r12,r12,r12,lsr#1    @ 2a^4 Q(42+r1)<2^-21 =a^4 (Q43+r1)<2^-20
 sub r0,r0,r12,lsr#16     @ a-a^2/2+a^3/3-a^4/4 Q(29+r1)
 lsrs r12,#15             @ a^4 Q(28+r1)<2^-20
 mul r12,r12,r3           @ a^5 Q(48+r1)<2^-25
 add r12,r12,r12,lsr#1    @ 3/2 a^5 Q(48+r1) ~ a^5/5 Q(51+r1)
 adds r0,r0,r12,lsr#22    @ a-a^2/2+a^3/3-a^4/4+a^5/5 Q(29+r1)
 bpl 12f                  @ underflow?
 lsrs r0,#8               @ Q(21+r1)
 rsb r1,#0x80             @ offset exponent
 adc r0,r0,r1,lsl#23      @ pack and round
11:
 bx r14
.balign 4
12:
 lsrs r0,#7               @ Q(22+r1)
 rsb r1,#0x7f             @ offset exponent
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
10:                       @ 0.5<=x<1
 cmn r3,#0x00040000       @ -(1-x) Q24
 bgt 7f                   @ (1-x)<2^-6?
 ln_s1                    @ calculate ln of mantissa
 rsbs r0,#0               @ negate
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x176            @ offset exponent obtained from normalisation
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
7:                        @ x=1-a, a<2^-6 negated in r3 Q24
 rsbs r3,#0               @ a
 lsls r3,#5               @ a Q29<2^-6
 clz r1,r3                @ normalisation shift r1>=9
 lsls r0,r3,r1            @ now normalised to 1..2 Q31 = a Q(29+r1)
 lsrs r0,#1               @ 1..2 Q30; a Q(28+r1)
 umull r12,r2,r0,r3       @ a^2 Q(57+r1)<2^-10
 add r0,r0,r2,lsl#2       @ a+a^2/2 Q(28+r1)
 lsrs r3,#9               @ a Q20<2^-5
 lsrs r2,#7               @ a^2 Q(18+r1)<2^-10
 mul r12,r2,r3            @ a^3 Q(38+r1)<2^-15
 add r12,r12,r12,lsr#2
 add r12,r12,r12,lsr#4
 add r12,r12,r12,lsr#8    @ 4/3 a^3 Q(38+r1)<2^-14
 add r0,r0,r12,lsr#12     @ a+a^2/2+a^3/3 Q(28+r1)
 lsrs r12,#16             @ 4/3 a^3 Q(22+r1)<2^-14
 mul r12,r12,r3           @ 4/3 a^4 Q(42+r1)<2^-19
 add r12,r12,r12,lsr#1    @ 2a^4 Q(42+r1)<2^-21=a^4 (Q43+r1)<2^-20
 add r0,r0,r12,lsr#17     @ a+a^2/2+a^3/3+a^4/4 Q(28+r1)
 lsrs r12,#15             @ a^4 Q(28+r1)<2^-20
 mul r12,r12,r3           @ a^5 Q(48+r1)<2^-25
 add r12,r12,r12,lsr#1    @ 3/2 a^5 Q(48+r1) ~ a^5/5 Q(51+r1)
 adds r0,r0,r12,lsr#23    @ a+a^2/2+a^3/3+a^4/4+a^5/5 Q(28+r1)
 bmi 12f                  @ overflow?
 lsrs r0,#7               @ Q(21+r1)
 rsb r1,#0x180            @ offset exponent + sign bit
 adc r0,r0,r1,lsl#23      @ pack and round
11:
 bx r14
.balign 4
12:
 lsrs r0,#8               @ Q(20+r1)
 rsb r1,#0x180            @ offset exponent + sign bit
 add r1,#1
 adc r0,r0,r1,lsl#23      @ pack and round
 bx r14

.balign 4
8:                        @ +Inf/NaN
 cmp r0,#0x7f800000
 bhi 9f                   @ NaN?
 bx r14                   @ +Inf: return it

.balign 4
1:                        @ negative argument
 cmp r0,#0x80000000
 beq 3f                   @ -0?
 cmp r0,#0xff800000
 bhi 9f                   @ NaN? return modified NaN
 mov r0,#0x7f800000       @ return NaN
9:
 orr r0,#0x00400000
 bx r14

.balign 4
3:                        @ return -Inf
 mov r0,#0xff000000
 orr r0,#0x00800000
 bx r14

.balign 4
2:                        @ 0/denormal
 cmp r0,#0
 beq 3b                   @ 0? return -Inf
 clz r1,r0                @ denormal: normalise
 subs r3,r1,#8
 lsl r2,r0,r3
 rsbs r1,#0
 subs r1,#0x75            @ fix exponent from normalising shift
 b 5b



@ After range reduction (so that 0<=x<1) sin x and cos x are calculated
@ by writing x=a+e where a has 7 bits of significance and |e|<=2^-8. Look-up
@ tables provide sin a and cos a, and sin e and cos e are approximated
@ using power series. Then
@ 
@   sin(a+e)=sin a cos e + cos a sin e
@ 
@ and
@ 
@   cos(a+e)=cos a cos e - sin a sin e.

@ cos x Q31 0<=x<=1

.section .text.qfp.fsinfcos,"ax",%progbits
.balign 4
cos_t0:
.long 0x80000000,0x7fff0000,0x7ffc0005,0x7ff7001b,0x7ff00055,0x7fe700d0,0x7fdc01b0,0x7fcf0320
.long 0x7fc00555,0x7faf088b,0x7f9c0d05,0x7f87130f,0x7f701afe,0x7f57252d,0x7f3c3200,0x7f1f41e3
.long 0x7f00554a,0x7edf6cb0,0x7ebc8899,0x7e97a990,0x7e70d02a,0x7e47fd01,0x7e1d30b8,0x7df06bfc
.long 0x7dc1af7e,0x7d90fbfb,0x7d5e5234,0x7d29b2f4,0x7cf31f0f,0x7cba975d,0x7c801cc2,0x7c43b027
.long 0x7c05527e,0x7bc504c0,0x7b82c7ef,0x7b3e9d13,0x7af8853e,0x7ab08187,0x7a66930f,0x7a1abafd
.long 0x79ccfa80,0x797d52d1,0x792bc52d,0x78d852da,0x7882fd27,0x782bc568,0x77d2acfb,0x7777b544
.long 0x771adfae,0x76bc2dae,0x765ba0be,0x75f93a60,0x7594fc1d,0x752ee787,0x74c6fe36,0x745d41ca
.long 0x73f1b3e9,0x73845642,0x73152a8b,0x72a4327f,0x72316fe3,0x71bce482,0x7146922e,0x70ce7ac0
.long 0x7054a019,0x6fd90420,0x6f5ba8c4,0x6edc8ffa,0x6e5bbbbd,0x6dd92e13,0x6d54e904,0x6cceeea2
.long 0x6c474106,0x6bbde24c,0x6b32d49c,0x6aa61a21,0x6a17b50e,0x6987a79d,0x68f5f40d,0x68629ca7
.long 0x67cda3b6,0x67370b90,0x669ed68e,0x66050711,0x65699f80,0x64cca24a,0x642e11e1,0x638df0c1
.long 0x62ec4169,0x62490661,0x61a44236,0x60fdf77a,0x605628c6,0x5facd8bb,0x5f0209fc,0x5e55bf36
.long 0x5da7fb19,0x5cf8c05d,0x5c4811bf,0x5b95f200,0x5ae263ea,0x5a2d6a4c,0x597707f8,0x58bf3fc8
.long 0x5806149c,0x574b8958,0x568fa0e6,0x55d25e36,0x5513c43d,0x5453d5f6,0x53929660,0x52d0087f
.long 0x520c2f5f,0x51470e0f,0x5080a7a3,0x4fb8ff35,0x4ef017e3,0x4e25f4d1,0x4d5a9928,0x4c8e0815
.long 0x4bc044ca,0x4af1527e,0x4a21346d,0x494fedd8,0x487d8204,0x47a9f43a,0x46d547c9,0x45ff8003
.long 0x4528a03f

@ sin x Q31 0<=x<=1

.balign 4
sin_t0:
.long 0x00000000,0x00ffff55,0x01fffaab,0x02ffee00,0x03ffd556,0x04ffacac,0x05ff7004,0x06ff1b5e
.long 0x07feaabc,0x08fe1a1f,0x09fd6589,0x0afc88ff,0x0bfb8082,0x0cfa4817,0x0df8dbc3,0x0ef7378b
.long 0x0ff55777,0x10f3378e,0x11f0d3d8,0x12ee285e,0x13eb312c,0x14e7ea4e,0x15e44fd0,0x16e05dc0
.long 0x17dc1030,0x18d7632f,0x19d252d1,0x1accdb29,0x1bc6f84f,0x1cc0a659,0x1db9e160,0x1eb2a580
.long 0x1faaeed5,0x20a2b97f,0x219a019e,0x2290c355,0x2386faca,0x247ca422,0x2571bb88,0x26663d28
.long 0x275a252f,0x284d6fcd,0x29401936,0x2a321d9f,0x2b23793f,0x2c142852,0x2d042714,0x2df371c6
.long 0x2ee204ab,0x2fcfdc08,0x30bcf425,0x31a9494f,0x3294d7d4,0x337f9c06,0x3469923a,0x3552b6c7
.long 0x363b060a,0x37227c62,0x3809162f,0x38eecfd9,0x39d3a5c8,0x3ab79468,0x3b9a982a,0x3c7cad83
.long 0x3d5dd0e9,0x3e3dfed8,0x3f1d33cf,0x3ffb6c52,0x40d8a4e7,0x41b4da1a,0x4290087a,0x436a2c9b
.long 0x44434313,0x451b487e,0x45f2397d,0x46c812b3,0x479cd0c9,0x4870706c,0x4942ee4e,0x4a144724
.long 0x4ae477aa,0x4bb37c9e,0x4c8152c5,0x4d4df6e7,0x4e1965d1,0x4ee39c57,0x4fac974e,0x50745393
.long 0x513ace07,0x52000391,0x52c3f11b,0x53869395,0x5447e7f6,0x5507eb37,0x55c69a5a,0x5683f262
.long 0x573ff05b,0x57fa9155,0x58b3d265,0x596bb0a6,0x5a222938,0x5ad73943,0x5b8addf1,0x5c3d1474
.long 0x5cedda03,0x5d9d2bdc,0x5e4b0740,0x5ef76979,0x5fa24fd4,0x604bb7a7,0x60f39e4b,0x619a0121
.long 0x623edd90,0x62e23104,0x6383f8f0,0x642432cc,0x64c2dc18,0x655ff259,0x65fb731b,0x66955bef
.long 0x672daa6f,0x67c45c38,0x68596ef1,0x68ece044,0x697eade5,0x6a0ed58b,0x6a9d54f7,0x6b2a29ed
.long 0x6bb5523c

@ This macro calculates tab0[a], tab1[a], sin e and cos e.
@ Results are tab0[a] * sin e Q31 in r0 and tab1[a] * cos e Q31 in r1.

.macro sc_s0 tab0 tab1
 lsrs r3,r0,#25           @ extract a
 adc r3,#0                @ rounding
 sub r0,r0,r3,lsl#25      @ e=x-a Q32
 asrs r1,r0,#9            @ e Q23
 mul r2,r1,r1             @ e^2 Q46 |e^2|<=2^-16
 lsrs r2,#16              @ e^2 Q30 ~ 1-cos e Q31
 muls r1,r1,r2            @ e^3 Q23*Q30=Q53 |e^3|<=2^-24
 add r1,r1,r1,asr#2       @ 
 add r1,r1,r1,asr#4       @ 4/3 e^3 Q53
 add r1,#6<<21            @ reduce systematic error
 sub r0,r0,r1,asr#24      @ e-e^3/6 Q32 ~ sin e
 adr r1,\tab0
 ldr r1,[r1,r3,lsl#2]     @ tab0[a] Q31
 smull r1,r0,r1,r0        @ tab0[a] * sin e Q31 in r0
 adr r1,\tab1
 ldr r1,[r1,r3,lsl#2]     @ tab1[a] Q31
 lsrs r3,r1,#15           @ Q16
 muls r3,r2,r3            @ tab1[a] * (1-cos e) Q47
 sub r1,r1,r3,lsr#16      @ tab1[a] * cos e Q31
.endm

.balign 4
.thumb_func
qfp_fsin:
 lsr r12,r0,#31           @ save sign
 ubfx r1,r0,#23,#8        @ extract exponent
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,r2,#0x00800000    @ implied 1 Q23
 subs r1,#0x7a            @ e+5
 bmi 1f                   @ e<-5?
 cmp r1,#5
 bge 2f                   @ e>=0? need to do range reduction (or Inf or NaN)
 lsl r0,r2,r1             @ x Q28 2^-5<=x<1
 lsls r0,#4               @ x Q32
sin_reentry:
 sc_s0 cos_t0,sin_t0      @ calculate sin a cos e, cos a sin e Q31
 add r0,r0,r1             @ sin x = sin a cos e + cos a sin e Q31
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x76             @ offset exponent
 adc r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
 bx r14

.balign 4
1:                        @ here e<-5
 rsb r1,#5                @ -e
 cmp r1,#14
 bge 5f                   @ small enough to return x?
sin_eps_reentry:
 lsls r0,r2,#1            @ x Q24
 lsrs r2,#9               @ x Q14
 mul r3,r2,r2             @ x^2 Q28
 lsrs r3,#14              @ x^2 Q14
 muls r3,r2,r3            @ x^3 Q28
 add r3,r3,r3,lsr#2
 add r3,r3,r3,lsr#4
 add r3,r3,r3,lsr#8       @ 4x^3/3 Q28 = x^3/6 Q31
 add r3,r3,r3,lsr#14      @ rounding
 lsrs r3,r1
 lsrs r3,r1               @ x^3/6 Q(31+2*r1)
 sub r0,r0,r3,lsr#7       @ Q24 (this subtraction can just take us down to the next lower exponent)
 tst r0,#0xff000000
 beq 6f                   @ exponent decrement?
 lsrs r0,#1               @ Q23
 rsb r1,#0x7e             @ offset exponent
 add r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
5:
 bx r14

.balign 4
6:
 rsb r1,#0x7d             @ offset exponent
 add r0,r0,r1,lsl#23      @ pack
 add r0,r0,r12,lsl#31     @ replace sign
 bx r14

.balign 4
2:                        @ here e>=0
 cmp r1,#0x85             @ Inf/NaN?
 beq 3f
 lsl r12,#1               @ r12 is now quadrant count (0 or 2 for x>0 or x<0)
22:
 subs r1,#5               @ e
 cmp r1,#8
 bgt 7f                   @ very large argument (e>8)
 lsls r2,r1               @ x Q23 exact x<512
 lsrs r3,r2,#16           @ x Q7
 movw r1,#0xa2fa          @ 2/pi Q16
 muls r1,r1,r3            @ x*2/pi Q23 <326
 lsrs r1,#23
 adc r1,#0                @ q=round(x*2/pi) = quadrant count
 add r12,r12,r1           @ save quadrant count
 movlong r3,0xc90fdaa2    @ K0~pi/2 Q31
 muls r3,r1,r3            @ q*K0 Q31
 movlong r0,0x85A309      @ K1~pi/2-K0 Q57
 muls r0,r1,r0            @ q*K1 Q57
 add r3,r3,r0,lsr#26      @ nearest multiple of pi/2 is now (r3):(r0<<6) Q63
 rsbs r2,r3,r2,lsl#8      @ remainder Q31 -1<r<1
 bmi 8f                   @ negative remainder?
 cmp r2,#0x04000000
 blt 9f                   @ r<2^-5? need to do better range reduction
 lsls r0,r2,#1            @ Q32
 lsrs r12,#1
 bcs cos_reentry          @ odd quadrant count? drop into cos
 b sin_reentry            @ even quadrant count: drop into sin with sign bit in r12b0

.balign 4
9:                        @ do another 23 bits of range reduction
 lsls r1,r0,#6            @ Q63
 rsbs r1,#0               @ Q63
 sbcs r0,r2,#0            @ now r0:r1 is remainder Q63
 bmi 11f                  @ negative? (cannot be zero)
13:
 clz r3,r0                @ normalise remainder
 lsls r0,r3
 rsb r3,#32
 lsrs r1,r3
 orr r2,r0,r1
 rsb r1,r3,#32            @ exponent of reduced value
 lsrs r12,#1
 bcs 12f                  @ odd quadrant count: calculate cos
 adds r2,#64              @ here calculating sin: rounding
 lsrs r0,r2,#7
 lsrs r2,#8
 b sin_eps_reentry        @ drop into code for sin of small argument

.balign 4
12:
 lsrs r2,#16
 b cos_eps_reentry        @ drop into code for cos of small argument

.balign 4
11:                       @ here remainder is small and negative
 rsbs r1,#0
 sbc r0,r0,r0,lsl#1       @ make positive
 eor r12,r12,r12,lsl#1    @ fix quadrant count
 eor r12,#2
 b 13b                    @ drop into code for small positive remainder

.balign 4
8:                        @ here first remainder is negative
 cmn r2,#0x04000000
 bgt 9b                   @ r>-2^-5? need to do better range reduction
 rsbs r2,#0               @ make remainder positive
 lsls r0,r2,#1
 lsrs r12,#1
 bcs cos_reentry          @ odd quadrant count? drop into cos
 eor r12,#1               @ fix sign
 b sin_reentry            @ drop into sin

.balign 4
7:                        @ range reduction for very large arguments
 and r0,r1,#7             @ least significant 3 bits of exponent
 lsls r2,r0               @ apply to mantissa
 adr r3,recippi-12
 sub r3,r3,r1,lsr#3       @ use rest of exponent as offset into table of 1/2pi
 ldr r0,[r3]
 umull r1,r0,r0,r2
 ldr r1,[r3,#8]
 muls r1,r1,r2
 ldr r3,[r3,#4]
 umlal r0,r1,r2,r3        @ frac(x/2pi) now in r1:r0 Q64
 lsrs r2,r1,#30           @ contribution to quadrant count
 adc r12,r12,r2           @ bump quadrant count with rounding
 lsls r1,#2
 orrs r2,r1,r0,lsr#30
 lsl r0,#2                @ now frac(x/(pi/2)) is r2:r0 Q64
 bmi 70f                  @ remainder will be negative?
 movlong r3,0xc90fdaa2    @ pi/2 Q31
 umull r0,r1,r0,r3
 movs r0,#0
 umlal r1,r0,r2,r3        @ remainder pi/2*frac(x/(pi/2)) now in r0:r1 Q63
 cmp r0,#0x04000000
 blt 13b                  @ remainder <2^-5? drop into small remainder code
 lsls r0,#1
 lsrs r12,#1
 bcs cos_reentry          @ odd quadrant count? drop into cos
 b sin_reentry            @ even quadrant count: drop into sin

.balign 4
70:                       @ here with negative remainder
 rsbs r0,#0
 sbc r2,r2,r2,lsl#1       @ make remainder positive
 movlong r3,0xc90fdaa2    @ pi/2 Q31
 umull r0,r1,r0,r3
 movs r0,#0
 umlal r1,r0,r2,r3        @ remainder pi/2*frac(x/(pi/2)) now in r0:r1 Q63
 cmp r0,#0x04000000
 blt 71f                  @ remainder <2^-5?
 lsls r0,#1
 lsrs r12,#1
 bcs cos_reentry          @ odd quadrant count? drop into cos
 eor r12,#1               @ fix sign
 b sin_reentry            @ even quadrant count: drop into sin

.balign 4
71:                       @ small negative remainder
 eor r12,r12,r12,lsl#1    @ fix sign and quadrant count
 eor r12,#2
 b 13b                    @ drop into small remainder code

.balign 4
@ This table is 1/2pi to about 320 bits
.byte 0x03
.byte 0x70, 0xE9, 0x26, 0x21, 0x1D, 0x37, 0x92, 0xDB
.byte 0xB0, 0xE2, 0x5D, 0xEF, 0xD5, 0xB1, 0x28, 0xFF
.byte 0x20, 0xC8, 0x21, 0x9E, 0xCC, 0x4A, 0xB1, 0x6D
.byte 0xE0, 0x6E, 0x9A, 0xFA, 0xE8, 0xAB, 0x13, 0xFE
.byte 0x94, 0x0A, 0x22, 0x27, 0xB7, 0xC1, 0x7C, 0x51
.byte 0x00, 0x00, 0x00
recippi:                  @ table above is indexed backwards

.balign 4
3:                        @ Inf/NaN
 lsls r2,r0,#9
 bne 4f                   @ NaN?
 mov r0,#0xff000000       @ sin(Inf): return NaN
 orr r0,#0x00c00000
 bx r14
.balign 4
4:
 orrs r0,#0x00400000      @ sin(NaN): return modified NaN
 bx r14

.balign 4
.thumb_func
qfp_fcos:
 movs r12,#0              @ set sign to positive
 ubfx r1,r0,#23,#8        @ extract exponent
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,r2,#0x00800000    @ implied 1 Q23
 subs r1,#0x7a            @ e+5
 bmi 1f                   @ e<-5?
 cmp r1,#5
 bge 2f                   @ e>0: need to do range reduction (or Inf or NaN)
 lsl r0,r2,r1             @ x Q28 2^-5<=x<1
 lsls r0,#4               @ x Q32
cos_reentry:
 sc_s0 sin_t0,cos_t0      @ calculate cos a cos e, sin a sin e Q31
 subs r0,r1,r0            @ cos x = cos a cos e - sin a sin e Q31
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x76             @ offset exponent
 adc r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
 bx r14

.balign 4
1:                        @ here e<-5
 rsb r1,#5                @ -e
 cmp r1,#12
 bgt 5f                   @ small enough to return 1?
 lsrs r2,#8               @ Q15
cos_eps_reentry:
 muls r2,r2,r2            @ x^2 Q30 = x^2/2 Q31
 lsrs r2,r1
 lsrs r2,r1               @ x^2/2 Q(31+2*r1)
 lsrs r2,#7               @ Q(24+2*r1)
 adc r2,#0                @ rounding
 rsb r0,r2,#0x3f800000    @ 1-x^2/2
 add r0,r0,r12,lsl#31     @ insert sign
 bx r14
.balign 4
5:
 mov r0,#0x3f800000       @ return 1
 bx r14

.balign 4
2:
 cmp r1,#0x85             @ Inf/NaN?
 beq 3b                   @ treat as sin
 movs r12,#1              @ add 1 quadrant
 b 22b                    @ and treat as sin



@ After range reduction (so that 0<=x<1) tan x is calculated by writing
@ x=a+e where a is an entry in the second table above and |e|<~2^-8.
@ The first table above provides tan a, while tan e is calculated
@ using a power series.  Then
@ 
@   tan x = tan (a+e) = (tan a + tan e) / (1 - tan a tan e).
@ 
@ Since tan a tan e is small, 1 / (1 - tan a tan e) can be approximated
@ by 1 + tan a tan e + (tan a tan e)^2 + ...
@ 
@ For arguments near odd multiples of pi/2 the cotangent of the range-reduced
@ argument is calculated, using
@ 
@   cot (a+e) = (1 - tan a tan e) / (tan a + tan e).
@ 
@ This is evaluated using a direct division.

@ Entries in the second table are Q10 tangents of Q31 entries in the first.

.section .text.qfp.ftan,"ax",%progbits
.balign 4
tan_t0:
.long 0x00000000,0x00fffeab,0x01fff556,0x02ffdc03,0x03ffaab7,0x04ff597c,0x05fee061,0x06fe377c
.long 0x07fd56ee,0x08fc36e0,0x09facf87,0x0af91928,0x0bf70c13,0x0cf4a0aa,0x0df1cf5f,0x0f0e20d4
.long 0x100a5e4d,0x11061ec7,0x12015b0a,0x12fc0bf4,0x13f62a7b,0x150ed53a,0x1607a5cd,0x16ffceac
.long 0x17f7493e,0x190cdabb,0x1a02cd8c,0x1af7fe37,0x1c0ae54d,0x1cfe654e,0x1df11086,0x1f010bb9
.long 0x1ff1e00d,0x20ffbb98,0x220c6c6c,0x22fa41d8,0x2404aad0,0x250dd4b9,0x25f87705,0x26ff3478
.long 0x2804a073,0x2908b50a,0x2a0b6c8d,0x2b0cc188,0x2c0caec2,0x2d0b2f3f,0x2e083e3c,0x2f03d734
.long 0x2ffdf5d8,0x30f69618,0x32091137,0x32fe7dd3,0x340d6194,0x34ff8b7a,0x360ac841,0x36f9a265
.long 0x38012acb,0x3906bb64,0x3a0a515e,0x3b0bea36,0x3c0b83b1,0x3d091bdd,0x3e04b10e,0x3efe41e0
.long 0x3ff5cd2d,0x4103c2d7,0x41f70cc9,0x430052f4,0x44072469,0x450b8151,0x45f611d1,0x46f5c050
.long 0x4809e2e7,0x4904753f,0x49fc984a,0x4b088578,0x4bfb978a,0x4d02032d,0x4e059429,0x4f064e24
.long 0x5004350c,0x50ff4d13,0x51f79aa7,0x5301788b,0x54081ba7,0x54f7b29c,0x55f83237,0x5708e671
.long 0x5802e1c4,0x58f9c51d,0x5a00393a,0x5b032944,0x5c029f16,0x5cfea4b4,0x5df7444c,0x5efdec62
.long 0x6000c657,0x60ffdf19,0x61fb43ab,0x63036451,0x64076cfc,0x65076da6,0x66037651,0x66fb96f7
.long 0x67ff03c7,0x68fe2cbf,0x69f924d8,0x6afe63d0,0x6bff1b59,0x6cfb617c,0x6e00f1ad,0x6f01c040
.long 0x6ffde643,0x71026471,0x7201f104,0x72fca7ef,0x73fed20c,0x74fbe531,0x75ffb152,0x76fe2b9a
.long 0x7802ab9f,0x7901a5d3,0x79fb3f08,0x7b049ca1,0x7bfdb3f8,0x7cfb7149,0x7dfd4e05,0x7f02c5c8
.long 0x800200e5

tan_t1:
.short 0x0000,0x0008,0x0010,0x0018,0x0020,0x0028,0x0030,0x0038
.short 0x0040,0x0048,0x0050,0x0058,0x0060,0x0068,0x0070,0x0079
.short 0x0081,0x0089,0x0091,0x0099,0x00a1,0x00aa,0x00b2,0x00ba
.short 0x00c2,0x00cb,0x00d3,0x00db,0x00e4,0x00ec,0x00f4,0x00fd
.short 0x0105,0x010e,0x0117,0x011f,0x0128,0x0131,0x0139,0x0142
.short 0x014b,0x0154,0x015d,0x0166,0x016f,0x0178,0x0181,0x018a
.short 0x0193,0x019c,0x01a6,0x01af,0x01b9,0x01c2,0x01cc,0x01d5
.short 0x01df,0x01e9,0x01f3,0x01fd,0x0207,0x0211,0x021b,0x0225
.short 0x022f,0x023a,0x0244,0x024f,0x025a,0x0265,0x026f,0x027a
.short 0x0286,0x0291,0x029c,0x02a8,0x02b3,0x02bf,0x02cb,0x02d7
.short 0x02e3,0x02ef,0x02fb,0x0308,0x0315,0x0321,0x032e,0x033c
.short 0x0349,0x0356,0x0364,0x0372,0x0380,0x038e,0x039c,0x03ab
.short 0x03ba,0x03c9,0x03d8,0x03e8,0x03f8,0x0408,0x0418,0x0428
.short 0x0439,0x044a,0x045b,0x046d,0x047f,0x0491,0x04a4,0x04b7
.short 0x04ca,0x04de,0x04f2,0x0506,0x051b,0x0530,0x0546,0x055c
.short 0x0573,0x058a,0x05a1,0x05ba,0x05d2,0x05eb,0x0605,0x0620
.short 0x063b

@ This macro calculates the numerator and denominator of the
@ above fractions.

.macro tan_s0
 lsrs r3,r0,#25           @ extract bits for lookup
 adc r3,#0                @ rounding
 adr r2,tan_t0
 ldr r1,[r2,r3,lsl#2]     @ a Q31
 sub r0,r0,r1,lsl#1       @ e Q32 |e|<~2^-8
 adr r2,tan_t1
 ldrh r3,[r2,r3,lsl#1]    @ tan a Q10
 asrs r1,r0,#10           @ e Q22
 mul r2,r1,r1             @ e^2 Q44 e^2<~2^-16
 lsrs r2,#16              @ e^2 Q28
 muls r1,r1,r2            @ e^3 Q22*Q28=Q50 |x^3|<~2^-24
 add r1,r1,r1,asr#2
 add r1,r1,r1,asr#4
 add r1,#1<<20            @ rounding
 add r0,r0,r1,asr#20      @ e+e^3/3~tan e Q32
 asrs r2,r0,#4            @ tan e Q28
 muls r2,r2,r3            @ tan a * tan e Q10*Q28=Q38
 lsls r3,#20
 add r3,r3,r0,asr#2       @ tan a + tan e Q30
@ returning:
@ r2: u=tan a * tan e Q38 (signed)
@ r3: v=tan a + tan e Q30 (unsigned)
@ so
@   tan x=v/(1-u)
@   cot x=(1-u)/v
.endm

.balign 4
.thumb_func
qfp_ftan:
 lsr r12,r0,#31           @ save sign
 ubfx r1,r0,#23,#8        @ extract exponent
 bic r2,r0,#0xff000000    @ clear exponent
 orr r2,r2,#0x00800000    @ implied 1 Q23
 subs r1,#0x7a            @ e+5
 bmi 1f                   @ e<-5?
 cmp r1,#5
 bge 2f                   @ e>=0: need to do range reduction (or Inf or NaN)
 lsl r0,r2,r1             @ x Q28 2^-5<=x<1
 lsls r0,#4               @ x Q32
tan_reentry:
 tan_s0
 smull r0,r1,r2,r3        @ uv Q68
 add r3,r3,r1,asr#6       @ v+uv Q30
 asrs r2,#16              @ u Q22
 muls r2,r2,r2            @ u^2 Q44
 lsrs r2,#14              @ u^2 Q30
 lsrs r1,r3,#15           @ v+uv Q15
 muls r1,r1,r2            @ u^2(v+uv) Q45
 add r0,r3,r1,lsr#15      @ v+uv+u^2(v+uv)=v(1+u+u^2+u^3)~v/(1-u) Q30
 clz r1,r0
 rsb r1,#8
 lsrs r0,r1               @ normalise
 add r1,#0x77             @ offset exponent
 adc r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
 bx r14

.balign 4
1:                        @ here e<-5
 rsb r1,#5                @ -e
 cmp r1,#14
 bge 5f                   @ small enough to return x?
 lsls r2,#8               @ mantissa 1..2 Q31
tan_eps_reentry:
 lsrs r0,r2,r1            @ x Q31 <2^-5
 lsrs r0,#11              @ x Q20 <2^-5
 mul r3,r0,r0             @ x^2 Q40 <2^-10
 lsrs r0,r3,#16           @ x^2 Q24 <2^-10
 add r3,r3,r3,lsr#2
 add r3,r3,r3,lsr#4
 add r3,r3,r3,lsr#8       @ 4x^2/3 Q40 = x^2/3 Q42
 muls r0,r0,r0            @ x^4 Q48 = x^4/8 Q51 <2^-20
 add r3,r3,r0,lsr#9       @ x^2/3 + x^4/8 Q42 <2^-11
 lsrs r3,#16              @ x^2/3 + x^4/8 Q26
 lsrs r0,r2,#16           @ x Q15
 muls r0,r0,r3            @ x(x^2/3 + x^4/8)=x^3/3 + x^5/8 Q41
 add r0,#6<<15            @ rounding
 adds r0,r2,r0,lsr#10     @ x+x^3/3 + x^5/8 Q31: this addition can just take us up to the next higher exponent
 bcs 6f                   @ exponent increment?
 rsb r1,#0x7e             @ exponent offset
 lsrs r0,#8               @ result Q23
 adc r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
5:
 bx r14

.balign 4
6:
 lsrs r0,#9               @ result Q23
 rsb r1,#0x80             @ exponent offset
 add r0,r0,r1,lsl#23      @ pack and round
 add r0,r0,r12,lsl#31     @ replace sign
 bx r14

.balign 4
2:                        @ here e>=0
 cmp r1,#0x85             @ Inf/NaN?
 beq 3f
22:
 subs r1,#5               @ e
 cmp r1,#8
 bgt 7f                   @ very large argument (e>8)
 lsls r2,r1               @ x Q23 exact <512
 lsrs r3,r2,#16           @ x Q7
 movw r1,#0xa2fa          @ 2/pi Q16
 muls r1,r1,r3            @ x*2/pi Q23 <326
 lsrs r1,#23
 adc r1,#0                @ round(x*2/pi) = quadrant count
 add r12,r12,r1,lsl#1     @ now r12b0=sign, r12b1=odd quadrant
 movlong r3,0xc90fdaa2    @ K0~pi/2 Q31
 muls r3,r1,r3            @ q*K0 Q31
 movlong r0,0x85A309      @ K1~pi/2-K0 Q57
 muls r0,r1,r0            @ q*K1 Q57
 add r3,r3,r0,lsr#26      @ nearest multiple of pi/2 is now (r3):(r0<<6) Q63
 rsbs r2,r3,r2,lsl#8      @ remainder Q31 -1<r<1
 bmi 8f                   @ negative remainder?
 cmp r2,#0x04000000
 blt 9f                   @ r<2^-5? need to do better reduction
 lsls r0,r2,#1            @ Q32
 tst r12,#2
 beq tan_reentry          @ even quadrant count? drop into tan
 eor r12,#1               @ flip sign and drop into cot

cot_reentry:
 tan_s0
 asrs r2,#7               @ u Q31
 rsb r2,#0x80000000       @ 1-u Q31
 clz r1,r3
 add r12,r1,r12,lsl#8     @ provisional exponent of result
 adds r12,#0x7c
 lsls r3,r1               @ normalise v to "Q32"
 lsrs r3,#4               @ v Q28
 lsrs r1,r3,#12           @ v Q16
 udiv r0,r2,r1            @ (1-u)/v Q15 first approximation
 lsls r2,#12              @ 1-u Q43
 mls r2,r0,r3,r2          @ first remainder Q43
 lsls r2,#3               @ first remainder Q46
 sdiv r1,r2,r1            @ remainder/v second approximation Q30
 adds r0,r1,r0,lsl#15     @ (1-u)/v Q30
 bmi 17f                  @ quotient >=1?
 cmp r0,#0x40000000
 blt 1f                   @ quotient <.5?
 lsrs r0,#7               @ Q23
 adc r0,r0,r12,lsl#23     @ pack and round
 bx r14
.balign 4
1:
 subs r12,#1              @ adjust exponent
 lsrs r0,#6               @ Q23
 adc r0,r0,r12,lsl#23     @ pack and round
 bx r14
.balign 4
17:
 adds r12,#1              @ adjust exponent
 lsrs r0,#8               @ Q23
 adc r0,r0,r12,lsl#23     @ pack and round
 bx r14

.balign 4
9:                        @ do another 23 bits of range reduction
 lsls r1,r0,#6            @ Q63
 rsbs r1,#0               @ Q63
 sbcs r0,r2,#0            @ now r0:r1 is remainder Q63
 bmi 11f                  @ negative? (cannot be zero)
13:
 clz r3,r0                @ normalise remainder
 lsls r0,r3
 rsb r3,#32
 lsrs r1,r3
 orr r2,r0,r1
 rsb r1,r3,#32            @ exponent of reduced value
 tst r12,#2
 beq tan_eps_reentry      @ even quadrant count: calculate tan

 eor r12,#1
@ "cot_eps_reentry:"
 add r12,r1,r12,lsl#8     @ save exponent under sign
 mvn r0,#0x8000           @ 0xffff7fff  ~1 Q32
 lsrs r3,r2,#16           @ mantissa m Q15 1<=m<2
 udiv r0,r0,r3            @ ~1/m Q17
 lsrs r2,#4               @ m Q27
 mul r1,r2,r0             @ approximation error Q44
 lsls r1,#2               @ Q46
 sdiv r3,r1,r3            @ correction Q31
 rsb r0,r3,r0,lsl#14      @ 1/m Q31

 add r2,r2,r2,lsr#2
 add r2,r2,r2,lsr#4
 add r2,r2,r2,lsr#8
 lsr r2,r12
 lsr r2,r12               @ 4/3 m Q(27-2*r12)
 sub r0,r0,r2,lsl#2       @ 1/m - m/3 Q31 ~cot x
 cmp r0,#0x40000000
 blt 16f                  @ <0.5?
 adds r12,#0x7d           @ offset exponent
 lsrs r0,#7               @ Q23
 adcs r0,r0,r12,lsl#23    @ pack and round
 bx r14

.balign 4
16:
 adds r12,#0x7c           @ offset exponent
 lsrs r0,#6               @ Q23
 adcs r0,r0,r12,lsl#23    @ pack and round
 bx r14

.balign 4
11:                       @ here remainder is negative
 rsbs r1,#0
 sbc r0,r0,r0,lsl#1       @ negate
 eor r12,#1               @ flip sign
 b 13b                    @ return to main code

.balign 4
8:
 cmn r2,#0x04000000
 bgt 9b                   @ r>-2^-5? need to do better reduction
 rsbs r2,#0               @ make remainder positive
 lsls r0,r2,#1
 tst r12,#2               @ odd quadrant count? drop into cot
 bne cot_reentry
 eor r12,#1               @ fix sign
 b tan_reentry            @ drop into tan

.balign 4
7:                        @ range reduction for very large arguments
 and r0,r1,#7             @ least significant 3 bits of exponent
 lsls r2,r0               @ apply to mantissa
 adr r3,recippi2-12
 sub r3,r3,r1,lsr#3       @ use rest of exponent as offset into table of 1/2pi
 ldr r0,[r3]
 umull r1,r0,r0,r2
 ldr r1,[r3,#8]
 muls r1,r1,r2
 ldr r3,[r3,#4]
 umlal r0,r1,r2,r3        @ frac(x/2pi) now in r1:r0 Q64
 lsrs r2,r1,#30           @ contribution to quadrant count
 adc r2,#0                @ rounding
 add r12,r12,r2,lsl#1     @ save quadrant count
 lsls r1,#2
 orrs r2,r1,r0,lsr#30
 lsl r0,#2                @ now frac(x/(pi/2)) is r2:r0 Q64
 bmi 70f                  @ remainder will be negative?
 movlong r3,0xc90fdaa2    @ pi/2 Q31
 umull r0,r1,r0,r3
 movs r0,#0
 umlal r1,r0,r2,r3        @ remainder pi/2*frac(x/(pi/2)) now in r0:r1 Q63
 cmp r0,#0x04000000
 blt 13b                  @ remainder <2^-5? drop into small remainder code
 lsls r0,#1
 tst r12,#2
 beq tan_reentry          @ even quadrant count? drop into tan
 eor r12,#1               @ flip sign
 b cot_reentry            @ odd quadrant count: drop into cot

.balign 4
70:                       @ here with negative remainder
 rsbs r0,#0
 sbc r2,r2,r2,lsl#1       @ make remainder positive
 eor r12,#1               @ flip sign
 movlong r3,0xc90fdaa2    @ pi/2 Q31
 umull r0,r1,r0,r3
 movs r0,#0
 umlal r1,r0,r2,r3        @ remainder pi/2*frac(x/(pi/2))now in r0:r1 Q63
 cmp r0,#0x04000000
 blt 13b                  @ remainder <2^-5?
 lsls r0,#1
 tst r12,#2
 beq tan_reentry          @ even quadrant count? drop into tan
 eor r12,#1               @ flip sign
 b cot_reentry            @ odd quadrant count: drop into cot

.balign 4
.byte 0x03
.byte 0x70, 0xE9, 0x26, 0x21, 0x1D, 0x37, 0x92, 0xDB
.byte 0xB0, 0xE2, 0x5D, 0xEF, 0xD5, 0xB1, 0x28, 0xFF
.byte 0x20, 0xC8, 0x21, 0x9E, 0xCC, 0x4A, 0xB1, 0x6D
.byte 0xE0, 0x6E, 0x9A, 0xFA, 0xE8, 0xAB, 0x13, 0xFE
.byte 0x94, 0x0A, 0x22, 0x27, 0xB7, 0xC1, 0x7C, 0x51
.byte 0x00, 0x00, 0x00
recippi2: @ table above is indexed backwards

.balign 4
3:                        @ Inf/NaN
 lsls r2,r0,#9
 bne 4f                   @ NaN? return modified NaN
 mov r0,#0xff000000       @ tan(Inf): return NaN
 orr r0,#0x00c00000
 bx r14
.balign 4
4:
 orrs r0,#0x00400000      @ tan(NaN)
 bx r14




@ This table is atn(x) Q32

.section .text.qfp.fatan2,"ax",%progbits
.balign 4
atan_t0:
.long 0x00000000,0x01fffd55,0x03ffeaab,0x05ffb806,0x07ff556f,0x09feb2f9,0x0bfdc0c2,0x0dfc6ef9
.long 0x0ffaaddc,0x11f86dbf,0x13f59f0e,0x15f23250,0x17ee1826,0x19e94154,0x1be39ebe,0x1ddd2170
.long 0x1fd5ba9b,0x21cd5b9a,0x23c3f5f6,0x25b97b66,0x27adddd2,0x29a10f54,0x2b93023c,0x2d83a913
.long 0x2f72f698,0x3160ddc5,0x334d51d3,0x35384637,0x3721aea5,0x39097f15,0x3aefabbe,0x3cd4291d
.long 0x3eb6ebf2,0x4097e944,0x4277165f,0x445468d9,0x462fd68c,0x480955a0,0x49e0dc81,0x4bb661ea
.long 0x4d89dcdc,0x4f5b44a5,0x512a90db,0x52f7b962,0x54c2b665,0x568b805d,0x5852100c,0x5a165e7d
.long 0x5bd86508,0x5d981d4a,0x5f55812e,0x61108ae3,0x62c934e5,0x647f79f3,0x66335515,0x67e4c198
.long 0x6993bb0f,0x6b403d51,0x6cea4477,0x6e91ccde,0x7036d325,0x71d9542b,0x73794d0d,0x7516bb28
.long 0x76b19c16,0x7849edac,0x79dfadfc,0x7b72db51,0x7d03742d,0x7e91774c,0x801ce39e,0x81a5b849
.long 0x832bf4a7,0x84af9843,0x8630a2db,0x87af145b,0x892aece0,0x8aa42cb2,0x8c1ad446,0x8d8ee43d
.long 0x8f005d5f,0x906f409f,0x91db8f16,0x93454a03,0x94ac72ca,0x96110af0,0x97731420,0x98d29024
.long 0x9a2f80e6,0x9b89e870,0x9ce1c8e7,0x9e37248e,0x9f89fdc5,0xa0da5703,0xa22832dc,0xa37393f8
.long 0xa4bc7d19,0xa602f117,0xa746f2de,0xa888856e,0xa9c7abdc,0xab04694e,0xac3ec0fc,0xad76b62d
.long 0xaeac4c39,0xafdf8687,0xb110688b,0xb23ef5c7,0xb36b31c9,0xb495202b,0xb5bcc490,0xb6e222a9
.long 0xb8053e2c,0xb9261ada,0xba44bc7e,0xbb6126e6,0xbc7b5deb,0xbd936569,0xbea94145,0xbfbcf566
.long 0xc0ce85b9,0xc1ddf62e,0xc2eb4abb,0xc3f68757,0xc4ffaffb,0xc606c8a3,0xc70bd54d,0xc80ed9f7
.long 0xc90fdaa2

@ The atan2 function is calculated by first estimating the tan of
@ an angle through which the vector (x,y) can be rotated to give
@ a vector (x',y') which is close to the x axis. Then two further
@ small rotations are estimated. The rotation angles (the first
@ obtained from a look-up table, the second by a power series and
@ the third by linearisation) are added to give the result.

@ This macro calculates atan2(y,x) where y and x mantissas are in the
@ specified registers, both Q24. r2 contains the difference in exponents.
@ y <= x, so result is <=pi/4.

.macro atan_s0 ym xm
 lsls \ym,#7              @ y Q31
 lsrs \ym,r2              @ correct y for exponent difference: now 1<=x<2, 2^-5<=y<=x
 udiv r2,\ym,\xm          @ a~y/x Q7
 adr r3,atan_t0
 ldr r12,[r3,r2,lsl#2]    @ u=atn a Q32
 mls r3,r2,\xm,\ym        @ y'=y-ax Q31 <~2^-7 signed
 lsrs \ym,#9              @ y Q22
 muls \ym,\ym,r2          @ ay Q29
 add r1,\ym,\xm,lsl#5     @ x'=x+ay Q29 1<=x'<4; now all values are in fixed registers
 lsls r3,#5               @ y' Q36
 lsrs r2,r1,#9            @ x' Q20
 sdiv r2,r3,r2            @ b=y'/x' Q16
 add r12,r12,r2,lsl#16    @ u+b Q32
 mul r0,r2,r2
 muls r0,r0,r2            @ b^3 Q48
 add r0,r0,r0,asr#2
 add r0,r0,r0,asr#4
 add r0,r0,r0,asr#8       @ 4/3 b^3 Q48
 sub r12,r12,r0,asr#18    @ u+b-b^3/3 Q32 ~ atn a + atn b
 asrs r0,r3,#8            @ y' Q28
 muls r0,r0,r2            @ by' Q44
 add r0,r1,r0,asr#15      @ x''=x'+by' Q29
 lsrs r1,#1               @ x' Q28
 muls r1,r1,r2            @ bx' Q44
 rsb r3,r1,r3,lsl#8       @ y''=y'-bx' Q44
 lsrs r0,#15              @ x'' Q14
 sdiv r0,r3,r0            @ c=y''/x'' Q30
 add r0,r12,r0,lsl#2      @ u+b-b^3/3+c ~ atn a + atn b + atn c
.endm

@ This macro packs the result with given exponent offset.

.macro atan_s1 eoff
 clz r1,r0
 lsls r0,r1               @ normalise
 rsb r1,#\eoff            @ offset exponent
 lsrs r0,#8               @ Q23
 adcs r0,r0,r1,lsl#23     @ pack and round
 bx r14
.endm

@ This macro packs the result with sum of given exponent offsets.

.macro atan_s1x eoff0 eoff1
 clz r1,r0
 lsls r0,r1               @ normalise
 rsb r1,#\eoff0           @ offset exponent
 adds r1,#\eoff1
 lsrs r0,#8               @ Q23
 adcs r0,r0,r1,lsl#23     @ pack and round
 bx r14
.endm

@ This macro calculates atan2(y,x) where y and x mantissas are in the
@ specified registers, both Q24. r2 contains the difference in exponents,
@ which in this case is large.

.macro atan_s2 ym xm
 lsls \ym,#7              @ y Q31
 lsls \xm,#4              @ x Q28
 lsrs r12,\xm,#12         @ x Q16
 udiv r3,\ym,r12          @ a~y/x Q15
 lsls \ym,#12             @ y Q43
 mls \ym,r3,\xm,\ym       @ remainder r Q43
 lsls \ym,#2              @ r Q45
 sdiv \ym,\ym,r12         @ correction Q29
 adds r0,\ym,r3,lsl#14    @ quotient q=y/x Q29 q<2; now all values are in fixed registers
 cmp r2,#255              @ exponent difference >255? clamp it
 it gt
 movgt r2,#255
 lsrs r1,r0,#15           @ q Q14
 mul r3,r1,r1             @ q^2 Q28
 lsrs r3,#15              @ q^2 Q13
 muls r1,r3,r1            @ q^3 Q27 q^3<8
 lsrs r1,r2
 lsrs r1,r2               @ q^3 Q(27-2*r2)
 add r1,r1,r1,lsr#2
 add r1,r1,r1,lsr#4
 add r1,r1,r1,lsr#8       @ 4/3 q^3 Q(27-2*r2)
 adds r1,#8               @ rounding
 subs r0,r0,r1            @ q-q^3/3 Q29
 lsrs r1,#8
 muls r1,r1,r3            @ 4/3 q^5 Q(32-2*r2)
 lsrs r1,r2
 lsrs r1,r2               @ 4/3 q^5 Q(32-4*r2)
 add r0,r0,r1,lsr#6       @ ~q-q^3/3+q^5/5 ~ atn q Q29
.endm

@ This macro packs the result with sign when the exponent (in r2)
@ is small.

.macro atan_s3 sign
 clz r3,r0
 lsls r0,r3               @ normalise
 add r2,r3
 subs r3,r2,#0x80
 bgt 100f                 @ will result be denormal?
 rsb r2,#0x80+0x100*\sign @ offset exponent, add sign
 lsrs r0,#8               @ Q23
 adc r0,r0,r2,lsl#23      @ pack and round
 bx r14
.balign 4
100:
 cmp r3,#32
 bgt 101f                 @ will result be zero?
 lsrs r0,r3               @ create denormal result
 lsrs r0,#8
 adc r0,#0x80000000*\sign @ add sign
 bx r14
.balign 4
101:
.if \sign
 mov r0,#0x80000000 @ return +/-0
.else
 movs r0,#0
.endif
 bx r14
.endm

.balign 4
.thumb_func
qfp_fatan2:
 ubfx r3,r1,#23,#8        @ extract x exponent
 ubfx r2,r0,#23,#8        @ extract y exponent
 cbz r3,86f               @ x 0/denormal?
 cmp r3,#0xff
 beq 87f                  @ x Inf/NaN?
62:
 cbz r2,84f               @ y 0/denormal?
 cmp r2,#0xff
 beq 85f                  @ y Inf/NaN?
42:

 movs r12,#1
 bfi r0,r12,#23,#8        @ insert implied 1:s
 bfi r1,r12,#23,#8        @ now both Q23 1..2 plus sign bit
 lsls r1,#1               @ Q24
 bcs 1f                   @ x<0?
 lsls r0,#1               @ Q24
 bcs 2f                   @ y<0?
                          @ here x positive, y positive
 subs r2,r3               @ exponent difference ey-ex
 it eq
 cmpeq r0,r1
 bge 3f                   @ |y|>|x|?
 rsbs r2,#0               @ exponent difference ex-ey
 cmp r2,#5
 bhi 4f                   @ ex-ey>5?
 b 99f

.balign 4
86:
 lsls r1,#1               @ x 0/denormal: save sign
 beq 61f                  @ x=0?
 clz r3,r1
 lsl r1,r3                @ normalise
 lsr r1,#7                @ Q23
 rrxs r1,r1               @ restore sign
 rsb r3,#8                @ fix exponent
 b 62b

.balign 4
84:
 lsls r0,#1               @ y 0/denormal: save sign
 beq 41f                  @ y=0?
 clz r2,r0
 lsl r0,r2                @ normalise
 lsr r0,#7                @ Q23
 rrxs r0,r0               @ restore sign
 rsb r2,#8                @ fix exponent
 b 42b

.balign 4
61:                       @ x is +/-0
 rrxs r1,r1               @ restore sign
 lsls r12,r0,#1
 beq 63f                  @ y also +/-0?
 orr r1,r1,#0x00800000
 mvn r3,#256              @ treat 0 as very small (signed) value
 b 62b

.balign 4
63:                       @ both x and y are 0
 movlong r0,0x40490fdb*2  @ pi in IEEE format <<1
 teq r1,#0                @ x=+0?
 it eq
 moveq r0,#0              @ then result is signed zero
 rrxs r0,r0               @ bring in sign bit from y
 bx r14

.balign 4
41:                       @ y is +/-0
 rrxs r0,r0               @ restore sign
 orr r0,r0,#0x00800000
 mvn r2,#256              @ treat 0 as very small (signed) value
 b 42b

.balign 4
87:                       @ x is Inf or NaN
 lsls r3,r1,#9
 beq 71f                  @ x=Inf?
 orr r0,r1,#0x00400000    @ return modified NaN
 bx r14
.balign 4
71:
 mov r3,#1024             @ treat Inf as very large (signed) value
 b 62b

.balign 4
85:                       @ y is Inf or NaN
 lsls r2,r0,#9
 beq 51f                  @ y=Inf?
 orr r0,r0,#0x00400000    @ return modified NaN
 bx r14
.balign 4
51:
 mov r2,#1024             @ treat Inf as very large (signed) value
 b 42b

.balign 4
99:
 atan_s0 r0,r1            @ compute result
 atan_s1 0x7d
.balign 4
4:
 atan_s2 r0,r1            @ compute result
 atan_s3 0

.balign 4
3:
 cmp r2,#5
 bhi 4f                   @ ey-ex>5?
 atan_s0 r1,r0            @ compute first octant result
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 sub r0,r1,r0,lsr#1       @ subtract from pi/2
 atan_s1 0x7e
.balign 4
4:
 atan_s2 r1,r0            @ compute first octant result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 sub r0,r1,r0,lsl#2       @ subtract from pi/2
 atan_s1 0x7e

.balign 4
2:                        @ here x positive, y negative
 subs r2,r3               @ exponent difference ey-ex
 it eq
 cmpeq r0,r1
 bge 3f                   @ |y|>|x|?
 rsbs r2,#0               @ exponent difference ex-ey
 cmp r2,#5
 bhi 4f                   @ ex-ey>5?
 atan_s0 r0,r1            @ compute result
 atan_s1x 0x7d,0x100
.balign 4
4:
 atan_s2 r0,r1            @ compute result
 atan_s3 1

.balign 4
3:
 cmp r2,#5
 bhi 4f                   @ ey-ex>5?
 atan_s0 r1,r0            @ compute first octant result
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 sub r0,r1,r0,lsr#1       @ subtract from pi/2
 atan_s1 0x17e
.balign 4
4:
 atan_s2 r1,r0            @ compute first octant result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 sub r0,r1,r0,lsl#2       @ subtract from pi/2
 atan_s1 0x17e

.balign 4
1:                        @ here x<0
 lsls r0,#1
 bcs 2f                   @ y<0?
                          @ here x negative, y positive
 subs r2,r3               @ exponent difference ey-ex
 it eq
 cmpeq r0,r1
 bge 3f                   @ |y|>|x|?
 rsbs r2,#0               @ exponent difference ex-ey
 cmp r2,#5
 bhi 4f                   @ ex-ey>5?
 atan_s0 r0,r1            @ compute result
 movlong r1,0xC90FDAA2    @ pi Q30
 sub r0,r1,r0,lsr#2       @ subtract from pi
 atan_s1 0x7f
.balign 4
4:
 atan_s2 r0,r1            @ compute result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi Q30
 sub r0,r1,r0,lsl#1       @ subtract from pi
 atan_s1 0x7f

.balign 4
3:
 cmp r2,#5
 bhi 4f                   @ ey-ex>5?
 atan_s0 r1,r0            @ compute result
 movlong r1,0x6487ED51    @ pi/2 Q30
 add r0,r1,r0,lsr#2       @ add pi/2
 atan_s1 0x7f
.balign 4
4:
 atan_s2 r1,r0            @ compute result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 add r0,r1,r0,lsl#2       @ add pi/2
 atan_s1 0x7e

2:                        @ here x negative, y negative
 subs r2,r3               @ exponent difference ey-ex
 it eq
 cmpeq r0,r1
 bge 3f                   @ |y|>|x|?
 rsbs r2,#0               @ exponent difference ex-ey
 cmp r2,#5
 bhi 4f                   @ ex-ey>5?
 atan_s0 r0,r1            @ compute result
 movlong r1,0xC90FDAA2    @ pi Q30
 sub r0,r1,r0,lsr#2       @ subtract from pi
 atan_s1x 0x7f,0x100
.balign 4
4:
 atan_s2 r0,r1            @ compute result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi Q30
 sub r0,r1,r0,lsl#1       @ subtract from pi
 atan_s1x 0x7f,0x100

.balign 4
3:
 cmp r2,#5
 bhi 4f                   @ ey-ex>5?
 atan_s0 r1,r0            @ compute result
 movlong r1,0x6487ED51    @ pi/2 Q30
 add r0,r1,r0,lsr#2       @ add pi/2
 atan_s1x 0x7f,0x100
.balign 4
4:
 atan_s2 r1,r0            @ compute result
 lsrs r0,r2               @ return to fixed point
 movlong r1,0xC90FDAA2    @ pi/2 Q31
 add r0,r1,r0,lsl#2       @ add pi/2
 atan_s1 0x17e

.thumb_func
qfplib_end:
