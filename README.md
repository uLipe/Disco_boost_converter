# Disco_boost_converter
Convesor dc-dc boost sincrono na placa stm32f334-disco para a série de introdução a Digital Power.

Nesse pequeno projeto o usuário que possuir uma placa discovery stm32f334, pode descarregar esse firmware
na memória do microcontrolador e avaliar a operação de um conversor DC-DC do tipo boost, bastando que ele conecte
uma fonte no conector dedicado a entrada do conversor.

o conversor é bem configurável, podendo operar em malha aberta para estudo de suas propriedades em modo
estacionário ou em malha fechada, rodando um compensador digital de tempo real para controlar variações
de tensões ou carga exigida na saida bastando modificar a definição BOOST_CTL_TYPE em /inc/boost.h

Enjoy!
 

