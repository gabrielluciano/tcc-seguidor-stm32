# Código do TCC: Desenvolvimento de um sistema de controle para mapeamento de percursos utilizando um robô seguidor de linha

## Sobre

Este respositório contém o código utilizado no meu TCC desenvolvido com o tema "Desenvolvimento de um sistema de controle para mapeamento de percursos utilizando um robô seguidor de linha". Trata-se um código para controle de um robô seguidor de linha construído com o microcontrolador STM32 Blue Pill. O código em C++ utiliza as bibliotecas Arduino e STM32Duino, além de implementações próprias para controlar os diversos periféricos do robô. Veja a seção funcionalidades para mais detalhes

## Funcionalidades

- Código para leitura de sensores de refletância analógicos utilizando DMA (Direct Memory Access). A função DMA permite que a leitura dos sensores seja enviada do Conversor Analógico Digital (ADC) diretamente para a memória sem a necessidade de processamento dos samples pela CPU, aumentando a velocidade de leitura dos dados brutos convertidos pelo ADC.
- Utilização de entradas e saídas digitais GPIO para leitura de botões e ativação do buzzer e leds através de funcões da biblioteca HAL (Hardware Abstract Layer)
- Uso de ISR (Interrupt Service Routine) baseadas no clock da CPU para executar cálculos em intervalos de tempos pré-definidos
- Uso de ISR ativadas por GPIO para contagem dos pulsos do encoder
- Implementação de PWM (Pulse Width Modulation) com frequência personalizável a partir de controle direto dos timers
- Código para controle via bluetooth a partir de comunicação serial realizada com o módulo HC-05
- Cálculos de posição da linha a partir da leitura dos sensores frontais
- Lógica para detecção e contagem de marcações laterais
- Cálculo do ângulo de curvatura e coordenadas XY e theta do robô.
