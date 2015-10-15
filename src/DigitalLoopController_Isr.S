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


			.section .text
			.thumb_func
@
@ DigitalPowerLoopFunc(void)
@
DigitalPowerLoopFunc:

		push {r1 - r7}
		@
		@ Puxa todos os arrays de estados e coeficientes
		@

		ldr r0, =loopwork
		ldmia r0!, {r1 - r7}
		@
		@ Nesse momento temos:
		@ r1 ->	float *pfeed;
		@ r2 ->	float *perr;
		@ r3 ->	float  *pAcoeff;
		@ r4 -> float  *pBcoeff;
		@ r5 ->	float *pxn;
		@ r6 ->	float *pyn;
		@ r7 -> int16_t *pOut;
		@


		@calcula o erro:
		vldr s0, [r1]
		vldr s1, [r2]

		@err = ref - feed
		vsub.F32 s0, s1, s0

		@write back no vetor xn:
		vstr.F32 s0, [r5]

		@pega o array de coeficientes:
		vldmia.F32 r3!, {s0 - s2}
		vldmia.F32 r4!, {s3 - s5}

		@pega xn:
		vldr.F32   s6,  [r5]
		subs   r5, #4
		ands   r5, #~3
		vldr.F32   s7,  [r5]
		subs   r5, #4
		ands   r5, #~3
		vldr.F32   s8,  [r5]


		@pega yn
		vldr.F32   s9,  [r6]
		subs   r6, #4
		ands   r6, #~3
		vldr.F32   s10,  [r6]
		subs   r6, #4
		ands   r6, #~3
		vldr.F32   s11,  [r6]


		@performa a multiplicacao e acumulacao:
		@(calcula filtro PID)
		vsub.F32  s31, s31
		vmla.F32  s31, s0,s6
		vmla.F32  s31, s1,s7
		vmla.F32  s31, s2,s8
		vmla.F32  s31, s3,s9
		vmla.F32  s31, s4,s10
		vmla.F32  s31, s5,s11

		@ajusta os ponteiros para a posicao inicial:
		adds r5, #4
		ands r5, #~3
		adds r5, #4
		ands r5,#~3
		adds r5, #4
		ands r5, #~3

		adds r6, #4
		ands r6,#~3
		adds r6, #4
		ands r6,#~3
		adds r6, #4
		ands r6,#~3

		@write back do resultado para o yn mais recente:
		vstr.F32 s31, [r6]

		@normaliza valor em Q5.10 e deposita na saida:
		vcvt.F32.s16 s31, s31, #10
		vstr.F32  s31, [r7]

		@atualiza workspace do pid:
		stmia r0!, {r1 - r7}
		pop {r0 - r7}
		bx lr
		@encerra aqui.........................


		.end
