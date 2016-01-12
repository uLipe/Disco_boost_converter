@
@		SERIE DIGITAL POWER COM A DISCOVERY EMBARCADOS
@	@fn DigitalLoopControl
@	@brief esse arquivo possui a rotina de compesador digital
@          otimizada para velocidade de execucao usando a FPU
@          do processador
@


			.thumb
			.syntax unified @sintaxe compativel entre ARMs


@
@ 	Exporta simbolos externos usados em boost.c
@

			.extern loopwork
			.globl  DigitalPowerLoopFunc

			.section .data


			.align 4
xn:			.word 0x00000000
yn:			.word 0x00000000




			.section .text
			.thumb_func
@
@ DigitalPowerLoopFunc(void)
@
DigitalPowerLoopFunc:

		push {r1 - r9}
		@
		@ Puxa todos os arrays de estados e coeficientes
		@

		ldr r0, =loopwork
		ldmia r0!, {r1 - r8}

		@
		@ Nesse momento temos:
		@ r1 ->	float *pfeed;
		@ r2 ->	float *perr;
		@ r3 ->	float  *kp;
		@ r4 -> float  *ki;
		@ r5 ->	float  *kd;
		@ r6 ->	float *integrator;
		@ r7 -> floar *prevError;
		@ r8 -> float  *pout;


		@loads PID float point workspace:
		vldr   s0, [r1] @ feed
		vldr   s1, [r2] @ ref
		vldr   s2, [r3] @ kp
		vldr   s3, [r4] @ ki
		vldr   s4, [r5] @ kd
		vldr   s5, [r6] @integrator
		vldr   s7, [r7] @prevErr


		@computes the error:
		vsub.F32 s0, s1, s0
		@computes error derivative:
		vsub.F32 s6, s0, s7
		@compute integral term:
		vadd.F32 s5, s0, s5


		@compute macs:
		vsub.F32 s31,s31   @zero the acc;
		vmla.F32 s31,s0,s2 @acc += kp * err
		vmla.F32 s31,s5,s3 @acc += ki * integrator
		vmla.F32 s31,s6,s4 @acc += kd * derivative


		@stores the new output value:
		vstr s31,[r8]
		@updates error history:
		vstr s0, [r7]
		@saves the current integrator:
		vstr s5, [r6]


		pop {r1 - r8}
		bx lr

		.end
