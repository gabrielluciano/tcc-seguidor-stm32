/* ---------------------- Bibliotecas ------------------------- */
#include <Arduino.h>  // Framework Arduino (STM32Duino)
#include <math.h>     // Biblioteca de funções matemáticas

/* ------------------------ Instâncias ------------------------ */

// Struct que receberá as configurações do ADC
ADC_HandleTypeDef adc_handle;

// Struct para comunicação serial com o módulo Bluetooth
//                      RX    TX
HardwareSerial Serial3(PB11, PB10);

// Instâncias do timer usado para realizar os cálculos
TIM_TypeDef *Instancia = TIM1;
HardwareTimer *TimerCalculos = new HardwareTimer(Instancia);

/* ----------------------- Definições ------------------------- */

// Driver
#define MOTOR_DIREITO_AIN1    PA15 // GPIOA -> GPIO_PIN_15
#define MOTOR_DIREITO_AIN2    PB3  // GPIOB -> GPIO_PIN_3
#define MOTOR_ESQUERDO_BIN2   PA10 // GPIOA -> GPIO_PIN_10
#define MOTOR_ESQUERDO_BIN1   PA11 // GPIOA -> GPIO_PIN_11
#define STBY PA12 // GPIOA -> GPIO_PIN_12

// Botoes
#define BOTAO1 PA8   // GPIOA -> GPIO_PIN_8
#define BOTAO2 PB15  // GPIOB -> GPIO_PIN_15

// LED BluePill
#define PINO_LED PC13  // GPIOC -> GPIO_PIN_13

// Buzzer
#define BUZZER  PA9   // GPIOA -> GPIO_PIN_9

// Sensor
#define IR PC15 // IR -> HIGH = Emissor de luz ligado

// Encoder esquerdo
#define ENCODER_E1 PB4    // GPIOB -> GPIO_PIN_4
#define ENCODER_E2 PB5    // GPIOB -> GPIO_PIN_5

// Encoder direito
#define ENCODER_D1 PB8    // GPIOB -> GPIO_PIN_8
#define ENCODER_D2 PB9    // GPIOB -> GPIO_PIN_9

// Valores tabela ASCII utilizados para decodificar a comunicação bluetooth
#define PROPORCIONAL  112   // Letra "p"
#define DERIVATIVA 100      // Letra "d"
#define BASE  98            // Letra "b"
#define MAXIMA   109        // Letra "m"
#define C 99                // Letra "c"

// Perímetro por pulso das rodas esquerda e direita
#define PPP_E 0.3581 // Esquerda - [mm/pulso]
#define PPP_D 0.3578 // Direita  - [mm/pulso]

// Distancia entre as rodas
#define L 122.3 // [mm]

// Limite de leitura dos sensores direito e esquerdo
// Valores menores que este limite representam a detecção da linha
#define LEFT_SENSOR_THRESHOLD 10
#define RIGHT_SENSOR_THRESHOLD 10

/* ----------------------- Protótipo de funções ------------------------ */

// Função executada a cada 1 milisegundo
void Interrupcao_1ms(void);
// Função que realiza os cálculos
void fazer_calculos(void);
// Função que altera a velocidade dos motores
void setar_velocidades(int16_t velocidade_esquerdo, int16_t velocidade_direito);
// Realiza a calibração dos sensores salvando os valores máximos e mínimos
void calibrar_sensores(void);
// Faz a leitura dos sensores utilizando os valores calibrados
uint16_t* ler_sensores(void);
// Utilizado para configurar os pinos ligados a porta usb
void USB_Pins_Config(void);
// Funções que setam o duty cicle dos motores
void SET_PWM_DIREITO(uint16_t duty);
void SET_PWM_ESQUERDO(uint16_t duty);
// Configura o modo de pwm
static void PWM_Config(void);
// Configura a interrupção de 1 milisegundo
static void TIMER_Interrupt_Config(void);

// Rotinas de interrupção dos encoders
void ENCODER_E1_ISR(void);  // Encoder esquerdo
void ENCODER_D1_ISR(void);  // Encoder direito


/* --------------------------- Variáveis ------------------------------ */


// Controle do usb
bool usb_ativado = true;

// Váriavel para controlar se o robô está correndo ou nao: false = parado
bool correndo = false;

// Array para guardar as leituras dos sensores (4 leituras para cada canal = 40/10 sensores)
// Utilizado pelo DMA
uint16_t leituras[40];

// Arrays para guardar as leituras máximas e mínimas obtidas nas calibrações
static uint16_t leiturasMinimas[10] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
static uint16_t leiturasMaximas[10] = {0};

// Variáveis de controle das contagens das marcações
uint8_t contagem_esquerda = 0; // Sensor esquerdo
uint8_t contagem_direita = 0;  // Sensor direito

uint8_t ultima_contagem_esquerda = 0;

// Variável que salva o estado da última leitura dos sensores laterais
bool ultimo_estado_esquerdo = false;
bool ultimo_estado_direito = false;

// Último erro dos sensores
float ultimo_erro = 0;

// Constantes do controle PID
float KP = 0.6; // Constante Proporcional
float KD = 5;   // Constante Derivativa

// Velocidades máximas e mínimas (valores em duty cicle do pwm)
// Min: 0
// Máx: 1999
uint16_t VELOCIDADE_BASE_E = 500; // Utilizadas durante as retas
uint16_t VELOCIDADE_BASE_D = 500;

uint16_t VELOCIDADE_MAX_E = 1999; // Max. vel. de cada motor durante a curva
uint16_t VELOCIDADE_MAX_D = 1999;

// Arrays para guardar os pulsos dos encoders
volatile long int contagem_encoders[2] = {0};
long int ultima_contagem_encoders[2] = {0};

// Guarda o último tempo lido
unsigned long ultimo_tempo = 0;

// Variável para controlar a frequência dos cálculos de mapeamento
uint16_t contagem_de_ciclos = 1;
uint16_t constante_de_tempo = 1;

// Variáveis para guardar a posição da última iteração
float ultimo_theta = 0;
float ultimo_x = 0;
float ultimo_y = 0;

/* ---------------------- Funções para a configuração do ADC --------------------- */


// Funções para controlar as interrupções do ADC e DMA
extern "C" void ADC1_2_IRQHandler(void) {
  HAL_ADC_IRQHandler(&adc_handle);
}
extern "C" void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(adc_handle.DMA_Handle);
}

// Função para inicialização do ADC com DMA (chamada pela função HAL_ADC_Init())
// Também são feitas configurações do clock do ADC aqui
extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
  /* Structs para configuração de portas, DMA e Clock */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  static DMA_HandleTypeDef dma_handle;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Configuração do clock do ADC1 */
  __HAL_RCC_ADC1_CLK_ENABLE();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configuração dos pinos utilizados */
  // PORT A
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PORT B
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configuração do DMA com ADC1 */
  dma_handle.Instance = DMA1_Channel1;

  dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
  dma_handle.Init.MemInc = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  dma_handle.Init.Mode = DMA_CIRCULAR;
  dma_handle.Init.Priority = DMA_PRIORITY_LOW;

  HAL_DMA_DeInit(&dma_handle);

  if(HAL_DMA_Init(&dma_handle) != HAL_OK) {
    Error_Handler();
  }

  __HAL_LINKDMA(hadc,DMA_Handle,dma_handle);

  /* Inicialização das Interrrupções do DMA */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

// Função para configuração do ADC e seus canais
static void ADC_Init(void) {
  /* Struct para configuração dos canais do ADC */
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Configuração do modo de ADC utilizado */
  adc_handle.Instance = ADC1;

  adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  adc_handle.Init.ContinuousConvMode = ENABLE;
  adc_handle.Init.DiscontinuousConvMode = DISABLE;
  adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc_handle.Init.NbrOfConversion = 10;

  if(HAL_ADC_Init(&adc_handle) != HAL_OK) {
    Error_Handler();
  }

  /* Configuração dos canais do ADC */

  // Canal 0
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Canal 1
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 2
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 3
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 4
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 5
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 6
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 7
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 8
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Canal 9
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&adc_handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


/* --------------------------------- Setup --------------------------------------- */


void setup() {
  /* Configuração de IO dos pinos */

  // Pinos de controle do driver como saída
  pinMode(MOTOR_DIREITO_AIN1, OUTPUT);
  pinMode(MOTOR_DIREITO_AIN2, OUTPUT);
  pinMode(MOTOR_ESQUERDO_BIN2, OUTPUT);

  // LED
  pinMode(PINO_LED, OUTPUT);

  // Botoes
  pinMode(BOTAO1, INPUT_PULLUP);
  pinMode(BOTAO2, INPUT_PULLUP);

  // Buzzer
  pinMode(BUZZER, OUTPUT);

  // Sensor
  pinMode(IR, OUTPUT);

  // Pinos do encoder usados para verificar direção
  pinMode(ENCODER_E2, INPUT);
  pinMode(ENCODER_D2, INPUT);

  /* Executa a função de configuração do PWM */

  PWM_Config();

  /* Configuração do timer para realizar os cálculos */
  
  TIMER_Interrupt_Config();

  /* Inicialização das saídas */

  // Inicia com o driver em modo Standby
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  // Inicia com o led apagado
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // Buzzer desligado
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  // Ativa o emissor dos sensores frontais
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  /* Registros das funções de interrupção externa dos encoders */
  attachInterrupt(digitalPinToInterrupt(ENCODER_E1), ENCODER_E1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D1), ENCODER_D1_ISR, RISING);

  /* Configuração do Conversor analógico digital (ADC) */

  // Inicia o ADC
  ADC_Init();

  // Calibra o ADC
  if (HAL_ADCEx_Calibration_Start(&adc_handle) != HAL_OK) {
    Error_Handler();
  }

  // Inicia as conversões com Direct Memory Acess (DMA)
  // As leituras dos sensores são salvas no buffer da variável "leituras"
  if (HAL_ADC_Start_DMA(&adc_handle, (uint32_t*)&leituras, 40)) {
    Error_Handler();
  }

  // Delay de 1000 milisegundos
  HAL_Delay(1000);

  //////////////// Calibração //////////////////////////
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  for (uint16_t i=0; i < 700; i++)  {
    calibrar_sensores();
    HAL_Delay(10);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  //////////////////////////////////////////////////////

  /**
   * 
   * Aguarda o botão ser pressionado para desligar o usb e configurar os pinos ligados ao usb
   * 
   *  Este trecho é importante pois a comunicação por USB entre o microcontrolador e o computador
   *  utiliza pinos que são utilizados pelo robô, e caso esses pinos sejam configurados a comuni-
   *  cação com o PC é interrompida inviabilizando o upload do código
   * 
  **/
  while (usb_ativado) {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
      usb_ativado = false;
      Serial.end();

      // Soa o buzzer e o led duas vezes
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_Delay(120);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

      HAL_Delay(400);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      HAL_Delay(120);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

      // Configura os pinos anteriormente ligados ao USB
      USB_Pins_Config();
      HAL_Delay(10);

      // Desativa o modo standby
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    }
  }

  // Inicializa a comunicação Bluetooth
  // Baud rate 115200 deve ser configurado no módulo HC-06
  Serial3.begin(115200);
}


/* --------------------------------- Loop ---------------------------------------- */

void loop() {
  // Inicia a volta quando o botão é pressionado
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) {
    // Soa buzzer e pisca o led
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_Delay(1200);

    // Inicia a volta
    correndo = true;
    TimerCalculos->resume();
    ultimo_tempo = millis();
  }

  // Modifica as variáveis via bluetooth
  if (!correndo) {
    if (Serial3.available()) {
      byte peek = Serial3.peek();
      Serial3.read();

      if (peek == PROPORCIONAL) {   // Constante proporcional
        KP = Serial3.parseFloat();
        Serial3.print("kp = ");
        Serial3.println(KP);
      }
      if (peek == DERIVATIVA) {   // Constante derivativa
        KD = Serial3.parseFloat();
        Serial3.print("kd = ");
        Serial3.println(KD);
      }

      if (peek == BASE) {   // Velocidade base
        uint16_t vel_base = Serial3.parseInt();
        VELOCIDADE_BASE_E = vel_base;
        VELOCIDADE_BASE_D = vel_base;
        Serial3.print("base = ");
        Serial3.println(vel_base);
      }
      if (peek == MAXIMA) {   // Velocidade máxima
        uint16_t vel_max = Serial3.parseInt();
        VELOCIDADE_MAX_E = vel_max;
        VELOCIDADE_MAX_D = vel_max;
        Serial3.print("max = ");
        Serial3.println(vel_max);
      }

      if (peek == C) {  // Número de ciclos de 1ms para realizar os cálculos
        uint16_t c = Serial3.parseInt();
        constante_de_tempo = c;
        Serial3.print("const = ");
        Serial3.println(c);
      }
    }
  }

}


/* --------------------------- Outras Funções ------------------------------- */

// Esta função é executada pelo microcontrolador a cada 1 milisegundo
// Isto permite que os cálculos sejam executados em um intervalo regular
void Interrupcao_1ms(void) {
  // Se está realizando a volta faça os cálculos
  if (correndo) {
    fazer_calculos();

  // Caso contrário pausa o timer, pára o robô e reseta as variáveis
  } else {
    // Desativa o timer
    TimerCalculos->pause();

    HAL_Delay(150);

    // Desliga o buzzer e o led
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // Limpa as variáveis para execução futura
    ultimo_estado_esquerdo = false;
    ultimo_estado_direito = false;
    contagem_esquerda = 0;
    contagem_direita = 0;
    ultima_contagem_esquerda = 0;

    // Pára os motores
    SET_PWM_ESQUERDO(0);
    SET_PWM_DIREITO(0);

    Serial3.begin(115200);
  }
}

// Função que realiza a lógica de controle principal do robô
void fazer_calculos(void) {

  ////////////////////////////////////////////////////////////////////
  ///////////////// LÓGICA DO SISTEMA DE LOCOMOÇÃO ///////////////////
  ////////////////////////////////////////////////////////////////////

  // Leitura dos sensores calibrada e escalada entre 0 e 1000
  uint16_t *leitura_calibrada = ler_sensores();

  // Calcula a posição da linha
  float posicao =  leitura_calibrada[1]*0
                  +leitura_calibrada[2]*1000
                  +leitura_calibrada[3]*2000
                  +leitura_calibrada[4]*3000
                  +leitura_calibrada[5]*4000
                  +leitura_calibrada[6]*5000
                  +leitura_calibrada[7]*6000
                  +leitura_calibrada[8]*7000;

  posicao = posicao / (leitura_calibrada[1]
                  +leitura_calibrada[2]
                  +leitura_calibrada[3]
                  +leitura_calibrada[4]
                  +leitura_calibrada[5]
                  +leitura_calibrada[6]
                  +leitura_calibrada[7]
                  +leitura_calibrada[8]);

  // Calcula o erro
  float erro = posicao - 3500; // Erro = posição - referência

  // Cálculo do PID em sua forma discreta
  int16_t incremento_velocidade = KP * erro + KD * (erro - ultimo_erro);

  // Com base no erro incrementa e decrementa as velocidades de cada roda
  // Incremento/decremento depende do sinal do erro
  int16_t velocidade_esquerdo = VELOCIDADE_BASE_E + incremento_velocidade;
  int16_t velocidade_direito  = VELOCIDADE_BASE_D - incremento_velocidade;

  // Salva o erro para ser utilizado no cálculo do PID da próxima iteração
  ultimo_erro = erro;

  // Define as novas velocidades para cada motor
  setar_velocidades(velocidade_esquerdo, velocidade_direito);

  ////////////////////////////////////////////////////////////////////
  ///////////////// LÓGICA DO SISTEMA DE MAPEAMENTO //////////////////
  ////////////////////////////////////////////////////////////////////

  // Executa uma vez a cada "1ms * constante_de_tempo"
  // Exemplo se constante_de_tempo = 5 -> executa a cada 5ms
  if (contagem_de_ciclos < constante_de_tempo) { // Não atingiu o tempo
    contagem_de_ciclos++;
  } else {  // Atingiu o tempo mínimo então realiza os cálculos
    
    // Diferença entre a leitura atual e anterior dos encoders
    int8_t diferenca_esquerda = contagem_encoders[0] - ultima_contagem_encoders[0];
    int8_t diferenca_direita = contagem_encoders[1] - ultima_contagem_encoders[1];

    // Salva a leitura atual de cada encoder para próx. iteração
    ultima_contagem_encoders[0] = contagem_encoders[0];
    ultima_contagem_encoders[1] = contagem_encoders[1];

    //////////////// Cálculos odométricos //////////////////

    // Raio de curvatura
    float r_icc = (L/2)
        * ((diferenca_esquerda * PPP_E + diferenca_direita * PPP_D)
        / (diferenca_direita * PPP_D - diferenca_esquerda * PPP_E));

    // Variação angular
    float theta_delta = (diferenca_direita * PPP_D - diferenca_esquerda * PPP_E) / L;

    // Coordenadas theta, x e y
    float theta = ultimo_theta + theta_delta;
    float x = ultimo_x + r_icc * (sin(theta) - sin(ultimo_theta));
    float y = ultimo_y - r_icc * (cos(theta) - cos(ultimo_theta));

    // Salva os últimos valores de theta, x e y
    ultimo_theta = theta;
    ultimo_x = x;
    ultimo_y = y;

    // Envia por bluetooth a diferença entre as leituras de cada encoder
    Serial3.print(diferenca_esquerda);
    Serial3.write('\t');
    Serial3.println(diferenca_direita);

    // Envia por bluetooth a leitura do sensor lateral sempre
    // que uma marcação é encontrada
    if (contagem_esquerda > ultima_contagem_esquerda) {
      Serial3.println(contagem_esquerda);
      ultima_contagem_esquerda = contagem_esquerda;
    }

    contagem_de_ciclos = 1;
  }

  ////////////////////////////////////////////////////////////////////
  ///////////// LÓGICA AUXILIAR DOS SENSORES LATERAIS ////////////////
  ////////////////////////////////////////////////////////////////////

  // Verifica se a marcação foi encontrada no sensor esquerdo
  if (leitura_calibrada[9] < LEFT_SENSOR_THRESHOLD) 
  {
    if (!ultimo_estado_esquerdo) {
      contagem_esquerda++; // Conta a marcação caso a última leitura seja preta
      ultimo_estado_esquerdo = true;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    }
  }
  else 
  {
    ultimo_estado_esquerdo = false; // Marcação não encontrada -> seta a última leitura como preta
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  }

  // Verifica se a marcação foi encontrada no sensor direito
  if (leitura_calibrada[0] < RIGHT_SENSOR_THRESHOLD) 
  {
    if(!ultimo_estado_direito) {
      contagem_direita++; // Conta a marcação caso a última leitura seja preta
      ultimo_estado_direito = true;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
  }
  else
  {
    ultimo_estado_direito = false; // Marcação não encontrada -> seta a última leitura como preta
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }

  // Última marcação lida, parar o robô
  if (contagem_direita >= 2) {
    correndo = false;
  }

  // Pára o robô caso o botão seja pressionado
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET) {
    correndo = false;
  }
}

// Seta a velocidade dos motores invertendo a rotação caso necessário
// e restringindo as velocidades às máximas definidas
void setar_velocidades(int16_t velocidade_esquerdo, int16_t velocidade_direito) {
  if (velocidade_esquerdo >= 0) {
    if (velocidade_esquerdo > VELOCIDADE_MAX_E) velocidade_esquerdo = VELOCIDADE_MAX_E;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    SET_PWM_ESQUERDO(velocidade_esquerdo);
  } else {
    velocidade_esquerdo *= -1;
    if (velocidade_esquerdo > VELOCIDADE_MAX_E) velocidade_esquerdo = VELOCIDADE_MAX_E;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    SET_PWM_ESQUERDO(velocidade_esquerdo);
  }

  if (velocidade_direito >= 0) {
    if (velocidade_direito > VELOCIDADE_MAX_D) velocidade_direito = VELOCIDADE_MAX_D;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    SET_PWM_DIREITO(velocidade_direito);
  } else {
    velocidade_direito *= -1;
    if (velocidade_direito > VELOCIDADE_MAX_D) velocidade_direito = VELOCIDADE_MAX_D;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    SET_PWM_DIREITO(velocidade_direito);
  }
}

/* Funções para calibração e leitura dos sensores */

// Faz a média entre as 4 últimas leituras dos sensores e salva seus valores máximos e mínimos
void calibrar_sensores(void) {

  for (uint8_t i = 0; i <= 9; i++) {

    uint16_t media = 0;

    for (uint8_t j = 0; j <= 30; j++) {
      media += leituras[j + i];
    }

    media = round((float)media / 4);
    if (media < leiturasMinimas[i]) leiturasMinimas[i] = media;
    if (media > leiturasMaximas[i]) leiturasMaximas[i] = media;

  }

}

// Faz a média das quatro últimas leituras dos sensores e retorna a leitura em um intervalo de 0 a 1000
// Utiliza os valores calibrados
uint16_t* ler_sensores(void) {
  static uint16_t leituraMedia[10];

  for (uint8_t i = 0; i <= 9; i++) {

    uint16_t media = 0;

    for (uint8_t j = 0; j <= 30; j++) {
      media += leituras[j + i];
    }

    media = round((float)media / 4);

    if (media < leiturasMinimas[i]) {
      leituraMedia[i] = 0;
    } else if (media > leiturasMaximas[i]) {
      leituraMedia[i] = 1000;
    } else {
      leituraMedia[i] = round(1000 * ((float)(media-leiturasMinimas[i]))/(leiturasMaximas[i] - leiturasMinimas[i]));
    }

  }

  return leituraMedia;
}

/* ISR's dos encoders -> ISR = Interruption Service Routine */
// As ISR's são acionadas a cada vez que o imã passa pelo sensor magnético,
// disparando um pulso que é lido pelo STM32
void ENCODER_E1_ISR(void) {
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? contagem_encoders[0]++ : contagem_encoders[0]--;
  } else {
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? contagem_encoders[0]-- : contagem_encoders[0]++;
  }
}

void ENCODER_D1_ISR(void) {
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) ? contagem_encoders[1]-- : contagem_encoders[1]++;
  } else {
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) ? contagem_encoders[1]++ : contagem_encoders[1]--;
  }
}


/* Configura os pinos ligados ao USB */
void USB_Pins_Config(void) {
  pinMode(MOTOR_ESQUERDO_BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Coloca o driver em modo standby
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

/* Seta o Duty Cicle diretamente pelo registrador */
/* PWMA do driver */
void SET_PWM_DIREITO(uint16_t duty) {
  if (duty > 1999) duty = 1999; // Duty Cicle tem um range de 2000 valores: 0 - 1999
  // Equivalente a fazer: MyTim->setCaptureCompare(channel, 500, TICK_COMPARE_FORMAT);
  TIM4->CCR2 = duty;
}

/* Seta o Duty Cicle diretamente pelo registrador */
/* PWMB do driver */
void SET_PWM_ESQUERDO(uint16_t duty) {
  if (duty > 1999) duty = 1999; // Duty Cicle tem um range de 2000 valores: 0 - 1999
  TIM4->CCR1 = duty;
}

/* Configuração dos Timers para geração do PWM com frequência de 1000 Hz */
static void PWM_Config(void) {
  // Retorna automaticamente o timer e o canal relacionado aos pinos passados como parâmetro
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PB8), PinMap_PWM); // Neste caso posso utilizar ambos os pinos já que usam o mesmo timer
  uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PB6), PinMap_PWM)); // Channel 1
  uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PB7), PinMap_PWM)); // Channel 2

  // Instanciamos um novo objeto do tipo HardwareTimer. O objeto não será destruido após o setpu() devido ao uso da palavra "new"
  HardwareTimer *MyTim = new HardwareTimer(Instance);

  // Configuração do modo de PWM
  MyTim->setMode(channel1, TIMER_OUTPUT_COMPARE_PWM1, PB6);  // Pino tem nivel lógico alto se counter < channel compare, caso contrário nível baixo
  MyTim->setMode(channel2, TIMER_OUTPUT_COMPARE_PWM1, PB7);  // Pino tem nivel lógico alto se counter < channel compare, caso contrário nível baixo

  // Configuração da frequêcia do PWM
  /*
   *  Frequencia do PWM = 72000000 / (prescaleFactor * overflowTick)
   */
  MyTim->setPrescaleFactor(72); // Divide o clock por 72 -> 72000000 / 72 = 1000000 (timer vai contar uma vez a cada 72 pulsos de clock)
  MyTim->setOverflow(2000, TICK_FORMAT); // Vai contar até 2000 e depois reiniciar tendo assim 1000000 / 2000 = 500 Hz de frequência

  // Inicia com duty cicle 0
  SET_PWM_ESQUERDO(0);
  SET_PWM_DIREITO(0);
  MyTim->resume();
}

/* Configuração da interrupção que realiza os cálculos a cada 1ms */
static void TIMER_Interrupt_Config(void) {
  TimerCalculos->setOverflow(1000, HERTZ_FORMAT); // 1000 Hz ou 1ms de periodo
  TimerCalculos->attachInterrupt(Interrupcao_1ms);
}