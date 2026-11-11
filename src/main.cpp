#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include "driver/twai.h"   // controlador CAN interno do ESP32 (já vem no core, não precisa de lib)

/*
  ============================================================
  Aquisição de dados - ESP32 DUAL CORE
  ============================================================
  Core 1: lê MPU6050 (acelerômetro) + ADS1115 (sensores analógicos)
          a ~100 Hz e envia cada amostra para uma fila (queue).
  Core 0: recebe da fila, acumula em um buffer em RAM e grava
          no cartão SD em lotes (menos operações de I/O = mais rápido).

  Isso evita que a escrita no SD (lenta e variável) atrase ou
  bloqueie a leitura dos sensores (que precisa ser constante).

  A cada boot, um NOVO arquivo CSV é criado (aqs_001.csv,
  aqs_002.csv, ...), usando um contador persistido no
  próprio SD (/contador.txt), já que o ESP32 não tem RTC.
  ============================================================

  Bibliotecas necessárias (Library Manager do Arduino IDE):
    "Adafruit MPU6050"
    "Adafruit ADS1X15"
    "Adafruit Unified Sensor"
    "SD" (já vem com o core do ESP32)

  Ligações I2C (MPU6050 e ADS1115 no mesmo barramento):
    ESP32          MPU6050         ADS1115
    -----          -------         -------
    3.3V     ->    VCC             VDD
    GND      ->    GND             GND
    GPIO21   ->    SDA             SDA
    GPIO22   ->    SCL             SCL

  Ligações do módulo SD (SPI - VSPI padrão do ESP32):
    ESP32          Módulo SD
    -----          ---------
    3.3V     ->    VCC
    GND      ->    GND
    GPIO23   ->    MOSI
    GPIO19   ->    MISO
    GPIO18   ->    SCK
    GPIO5    ->    CS
*/

// ---------------------------------------------------------------
// Flag de DEBUG: coloque em 1 quando estiver com o cabo USB
// conectado ao computador. Em 0, remove todos os prints e ganha
// desempenho para rodar "no carro", sem monitor serial.
// ---------------------------------------------------------------
#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ---------------------------------------------------------------
// Configurações gerais
// ---------------------------------------------------------------
const int PINO_CS = 5;
const char* ARQUIVO_CONTADOR = "/contador.txt"; // guarda o número da última sessão

char nomeArquivo[32]; // preenchido em tempo de execução, ex: "/aqs_003.csv"

const int FREQUENCIA_HZ = 50;                       // taxa de amostragem alvo
const int PERIODO_MS = 1000 / FREQUENCIA_HZ;          // 

const int TAMANHO_LOTE = 50;      // quantas amostras acumular antes de gravar no SD
const int TAMANHO_FILA = 200;     // capacidade da fila entre os dois cores (margem de segurança)

float TENSAO_MIN = 0.0;          // tensão mínima do sensor (A0)
float TENSAO_MAX = 0.0;          // tensão máxima do sensor (A0)
float ANGULO_MIN = 0.0;          // ângulo correspondente à tensão mínima
float ANGULO_MAX = 0.0;         // ângulo correspondente à tensão máxima

float TENSAO_MIN_PRESSAO = 0.5;          // tensão mínima do sensor (A1)
float TENSAO_MAX_PRESSAO = 4.5;          // tensão máxima do sensor
float PRESSAO_MIN = 0.0;          // pressão correspondente à tensão mínima
float PRESSAO_MAX = 100.0;         // pressão máxima REAL do sensor (impresso no corpo dele)


// ---------------------------------------------------------------
// Estrutura de uma amostra
// ---------------------------------------------------------------
struct Amostra {
  unsigned long tempo_ms;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float esterco, pressao, pressao2;
  float rpm, tps, mapKpa, cltC, matC, battV, P_oleo;
  float afr1, afr2;   // ECU MS2 via CAN (NAN se a ECU não estiver transmitindo)
};

// ---------------------------------------------------------------
// Objetos globais
// ---------------------------------------------------------------
Adafruit_MPU6050 mpu;
bool mpuDisponivel = false; // true se o MPU6050 foi encontrado no boot
Adafruit_ADS1115 ads;

// ---------------------------------------------------------------
// CAN (ECU MegaSquirt-2) via SN65HVD230 + controlador TWAI do ESP32
//   ESP32 GPIO25 (TX) -> pino D (TXD) do SN65HVD230
//   ESP32 GPIO26 (RX) <- pino R (RXD) do SN65HVD230
//   3.3V -> VCC | GND -> GND (GND em comum com a ECU) | CANH/CANL -> barramento
// ---------------------------------------------------------------
const gpio_num_t PINO_CAN_TX = GPIO_NUM_25;
const gpio_num_t PINO_CAN_RX = GPIO_NUM_26;

const uint32_t BASE_ID_MS2 = 1520;          // base padrão do broadcast em tempo real (0x5F0)
const unsigned long TIMEOUT_ECU_MS = 1000;  // sem frames por mais que isso = ECU considerada offline

bool canDisponivel = false;                 // true se o driver TWAI iniciou no boot

// Últimos valores recebidos da ECU (escritos pela tarefaCAN, lidos pela tarefa de
// leitura rápida). Valores de 32 bits são lidos/escritos atomicamente no ESP32.
volatile float ecuRpm = NAN, ecuTps = NAN, ecuMap = NAN, ecuClt = NAN;
volatile float ecuMat = NAN, ecuBatt = NAN, ecuP_oleo = NAN, ecuafr1 = NAN, ecuafr2 = NAN;
volatile unsigned long ecuUltimoRxMs = 0;   // millis() do último frame válido recebido

QueueHandle_t filaAmostras;

// ---------------------------------------------------------------
// funções existentes no código
// ---------------------------------------------------------------
void tarefaLeituraSensores(void *parametro);
void tarefaGravacaoSD(void *parametro);
void tarefaCAN(void *parametro);
void decodificarFrameMS2(uint32_t id, const uint8_t *d);
void gravarLoteNoSD(Amostra *buffer, int quantidade);
float processaesterco(float tensaoA0);
float processapressao(float tensaoA1);
int obterProximoNumeroSessao();

// ---------------------------------------------------------------
// SETUP - roda no core principal (Arduino usa o Core 1 por padrão
// para setup()/loop(), mas aqui vamos criar tasks fixadas manualmente)
// ---------------------------------------------------------------
void setup() {
#if DEBUG
  Serial.begin(115200);
  while (!Serial) { delay(10); }
#endif

  Wire.begin(21, 22);

  // --- MPU6050 ---
  DEBUG_PRINTLN("Iniciando MPU6050...");
  if (!mpu.begin()) {
    DEBUG_PRINTLN("AVISO: MPU6050 não encontrado. Prosseguindo sem ele (accel/gyro ficarão zerados).");
    mpuDisponivel = false;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G); // range maior p/ picos de frenagem/impacto
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpuDisponivel = true;
    DEBUG_PRINTLN("MPU6050 OK.");
  }

  // --- ADS1115 ---
  //DEBUG_PRINTLN("Iniciando ADS1115...");
  //ads.setGain(GAIN_TWOTHIRDS);
  //ads.setDataRate(RATE_ADS1115_860SPS);
  //if (!ads.begin()) {
  //  DEBUG_PRINTLN("ERRO: ADS1115 não encontrado.");
  //  while (1) { delay(1000); }
  //}
  //DEBUG_PRINTLN("ADS1115 OK.");

  // --- Cartão SD ---
  DEBUG_PRINTLN("Iniciando cartão SD...");
  if (!SD.begin(PINO_CS)) {
    DEBUG_PRINTLN("ERRO: cartão SD não encontrado.");
    while (1) { delay(1000); }
  }

  // Gera um nome de arquivo NOVO para esta sessão (ex: /aqs_003.csv),
  // usando um contador persistido no SD, já que o ESP32 não tem RTC.
  int numeroSessao = obterProximoNumeroSessao();
  snprintf(nomeArquivo, sizeof(nomeArquivo), "/aqs_%03d.csv", numeroSessao);
  DEBUG_PRINT("Arquivo desta sessão: ");
  DEBUG_PRINTLN(nomeArquivo);

  File arquivo = SD.open(nomeArquivo, FILE_WRITE);
  if (arquivo) {
    arquivo.println("tempo_ms;accelX;accelY;accelZ;gyroX;gyroY;gyroZ;esterco;pressao;pressao2;rpm;tps;map_kPa;clt_C;mat_C;batt_V;P_oleo;afr1;afr2"); // cabeçalho (ponto-e-vírgula, igual aos dados)
    arquivo.close();
  } else {
    DEBUG_PRINTLN("ERRO: não foi possível criar o arquivo desta sessão.");
  }
  DEBUG_PRINTLN("Cartão SD OK.");

  // --- CAN (TWAI) ---
  // Modo NORMAL: o ESP32 só envia o ACK dos frames recebidos (necessário para a
  // ECU não ficar retransmitindo sem resposta) e nunca transmite dados próprios.
  // 500 kbit/s é o padrão do broadcast do MS2 - confirme na tela CAN Parameters.
  DEBUG_PRINTLN("Iniciando CAN (TWAI)...");
  twai_general_config_t cfgGeral = TWAI_GENERAL_CONFIG_DEFAULT(PINO_CAN_TX, PINO_CAN_RX, TWAI_MODE_NORMAL);
  cfgGeral.rx_queue_len = 64; // folga para o SD travar a task de CAN por alguns ms
  twai_timing_config_t cfgTempo = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t cfgFiltro = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // filtramos por ID no software

  if (twai_driver_install(&cfgGeral, &cfgTempo, &cfgFiltro) == ESP_OK && twai_start() == ESP_OK) {
    canDisponivel = true;
    DEBUG_PRINTLN("CAN OK.");
  } else {
    DEBUG_PRINTLN("AVISO: falha ao iniciar o CAN. Prosseguindo sem dados da ECU.");
  }

  // --- Fila entre os cores ---
  filaAmostras = xQueueCreate(TAMANHO_FILA, sizeof(Amostra));
  if (filaAmostras == NULL) {
    DEBUG_PRINTLN("ERRO: falha ao criar a fila.");
    while (1) { delay(1000); }
  }

  // --- Cria as duas tasks, cada uma fixada em um core ---
  xTaskCreatePinnedToCore(
    tarefaLeituraSensores,
    "LeituraSensores",
    4096,
    NULL,
    2,      // prioridade um pouco mais alta: leitura não pode atrasar
    NULL,
    1       // Core 1
  );

  xTaskCreatePinnedToCore(
    tarefaGravacaoSD,
    "GravacaoSD",
    10240,  // pilha maior: buffer de lote (50 amostras) + File consomem stack
    NULL,
    1,
    NULL,
    0       // Core 0
  );

  // Task de recepção CAN no Core 0 (deixa o Core 1 livre para o loop de 100Hz).
  // Só é criada se o driver TWAI iniciou com sucesso.
  if (canDisponivel) {
    xTaskCreatePinnedToCore(
      tarefaCAN,
      "CAN",
      4096,
      NULL,
      1,
      NULL,
      0       // Core 0
    );
  }

  DEBUG_PRINTLN("Tasks criadas. Iniciando aquisição...");
  DEBUG_PRINTLN("----------------------------------------------------");
}

// loop() principal fica praticamente vazio: todo o trabalho
// acontece nas duas tasks criadas acima. 
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =================================================================
// Lê o último número de sessão salvo em /contador.txt, incrementa,
// salva o novo valor de volta, e retorna o número desta sessão.
// Se o arquivo não existir (primeiro boot), começa em 1.
// =================================================================
int obterProximoNumeroSessao() {
  int numero = 1;

  if (SD.exists(ARQUIVO_CONTADOR)) {
    File f = SD.open(ARQUIVO_CONTADOR, FILE_READ);
    if (f) {
      numero = f.parseInt(); // lê o número salvo
      f.close();
      numero++; // esta é uma nova sessão
    }
  }

  // Salva o novo número de volta (FILE_WRITE sobrescreve o conteúdo antigo)
  File f = SD.open(ARQUIVO_CONTADOR, FILE_WRITE);
  if (f) {
    f.print(numero);
    f.close();
  } else {
    DEBUG_PRINTLN("AVISO: não foi possível atualizar o contador de sessão.");
  }

  return numero;
}

// =================================================================
// TASK 1 (Core 1): leitura dos sensores em alta frequência
// =================================================================
void tarefaLeituraSensores(void *parametro) {
  TickType_t ultimoAcordar = xTaskGetTickCount();

  for (;;) {
    sensors_event_t accel, gyro, temp;
    if (mpuDisponivel) {
      mpu.getEvent(&accel, &gyro, &temp);
    }
    // Se o MPU não foi encontrado no boot, accel/gyro simplesmente não são
    // lidos aqui - os campos da Amostra abaixo ficam com valor 0 (variáveis
    // sensors_event_t começam zeradas por padrão).

    //int16_t brutoA0 = ads.readADC_SingleEnded(0);
    //int16_t brutoA1 = ads.readADC_SingleEnded(1);
    //int16_t brutoA2 = ads.readADC_SingleEnded(2);

    //float tensaoA0 = ads.computeVolts(brutoA0);
    //float tensaoA1 = ads.computeVolts(brutoA1);
    //float tensaoA2 = ads.computeVolts(brutoA2);

    Amostra a;
    a.tempo_ms = millis();
    a.accelX = mpuDisponivel ? accel.acceleration.x : 0.0;
    a.accelY = mpuDisponivel ? accel.acceleration.y : 0.0;
    a.accelZ = mpuDisponivel ? accel.acceleration.z : 0.0;
    a.gyroX = mpuDisponivel ? gyro.gyro.x : 0.0;
    a.gyroY = mpuDisponivel ? gyro.gyro.y : 0.0;
    a.gyroZ = mpuDisponivel ? gyro.gyro.z : 0.0;
    a.esterco = 0; // processaesterco(tensaoA0);
    a.pressao = 0; // processapressao(tensaoA1);
    a.pressao2 = 0; // processapressao(tensaoA2);

    // Dados da ECU: se nenhum frame chegou nos últimos TIMEOUT_ECU_MS (ECU desligada,
    // cabo solto, CAN desabilitado), grava NAN em vez de repetir o último valor
    // como se fosse atual.
    bool ecuAtiva = canDisponivel && ecuUltimoRxMs != 0 &&
                    (millis() - ecuUltimoRxMs) < TIMEOUT_ECU_MS;
    a.rpm    = ecuAtiva ? ecuRpm  : NAN;
    a.tps    = ecuAtiva ? ecuTps  : NAN;
    a.mapKpa = ecuAtiva ? ecuMap  : NAN;
    a.cltC   = ecuAtiva ? ecuClt  : NAN;
    a.matC   = ecuAtiva ? ecuMat  : NAN;
    a.battV  = ecuAtiva ? ecuBatt : NAN;
    a.P_oleo  = ecuAtiva ? ecuP_oleo : NAN;
    a.afr1 = ecuAtiva ? ecuafr1 : NAN;
    a.afr2 = ecuAtiva ? ecuafr2 : NAN;

    // Print de verificação rápida no Serial Monitor. Para deixar legível,
    // reduza FREQUENCIA_HZ (ex: 2-5 Hz) durante esse teste - depois volte
    // para 100Hz (ou o valor final desejado) para a aquisição de verdade.
    // DEBUG_PRINT("accelXYZ: ");
    // DEBUG_PRINT(a.accelX); DEBUG_PRINT(", ");
    // DEBUG_PRINT(a.accelY); DEBUG_PRINT(", ");
    // DEBUG_PRINT(a.accelZ);
    // DEBUG_PRINT("  |  gyroXYZ: ");
    // DEBUG_PRINT(a.gyroX); DEBUG_PRINT(", ");
    // DEBUG_PRINT(a.gyroY); DEBUG_PRINT(", ");
    // DEBUG_PRINT(a.gyroZ);
    // DEBUG_PRINT("  |  esterco(V): ");
    // DEBUG_PRINT(a.esterco);
    // DEBUG_PRINT("  pressao(V): ");
    // DEBUG_PRINT(a.pressao);
    // DEBUG_PRINT("  pressao2(V): ");
    // DEBUG_PRINT(a.pressao2);
    // DEBUG_PRINT("  |  ECU rpm: ");
    // DEBUG_PRINT(a.rpm);
    // DEBUG_PRINT("  tps: ");
    // DEBUG_PRINT(a.tps);
    // DEBUG_PRINT("  map: ");
    // DEBUG_PRINT(a.mapKpa);
    // DEBUG_PRINT("  clt(C): ");
    // DEBUG_PRINTLN(a.cltC);

    // Tenta enviar para a fila sem bloquear (timeout 0). Se a fila estiver
    // cheia (gravação no SD atrasada), a amostra atual é simplesmente
    // descartada e o loop segue — não mexe no conteúdo já enfileirado.
    if (xQueueSend(filaAmostras, &a, 0) != pdTRUE) {
      DEBUG_PRINTLN("AVISO: fila cheia, amostra descartada.");
    }

    // vTaskDelayUntil mantém o período constante mesmo que a leitura
    // em si demore um pouco (compensa o tempo já gasto no ciclo).
    vTaskDelayUntil(&ultimoAcordar, pdMS_TO_TICKS(PERIODO_MS));
  }
}

float processaesterco(float tensaoA0){

  float angulo = (tensaoA0 - TENSAO_MIN) * (ANGULO_MAX - ANGULO_MIN) / (TENSAO_MAX - TENSAO_MIN) + ANGULO_MIN;
  return angulo;
  
}

float processapressao(float tensaoA1){

  float pressao = (tensaoA1 - TENSAO_MIN_PRESSAO) * (PRESSAO_MAX - PRESSAO_MIN) / (TENSAO_MAX_PRESSAO - TENSAO_MIN_PRESSAO) + PRESSAO_MIN;
  return pressao;
  
}


// =================================================================
// Decodificação do broadcast em tempo real do MS2/Extra
// Todos os campos são big-endian, 2 bytes. ID = BASE_ID_MS2 + grupo.
// =================================================================
static inline int16_t  lerI16(const uint8_t *d, int i) { return (int16_t)((d[i] << 8) | d[i + 1]); }
static inline uint16_t lerU16(const uint8_t *d, int i) { return (uint16_t)((d[i] << 8) | d[i + 1]); }

// Temperaturas do MS2 chegam em °F multiplicados por 10
static inline float fx10ParaC(int16_t bruto) { return (bruto / 10.0f - 32.0f) * 5.0f / 9.0f; }

void decodificarFrameMS2(uint32_t id, const uint8_t *d) {
  if (id < BASE_ID_MS2 || id > BASE_ID_MS2 + 15) return; // fora da faixa dos grupos 0-15

  switch (id - BASE_ID_MS2) {
    case 0:  // seconds(0-1) pw1(2-3) pw2(4-5) rpm(6-7)
      ecuRpm = lerU16(d, 6);
      break;
    case 2:  // baro(0-1) map(2-3) mat(4-5) clt(6-7), todos com sinal e /10
      ecuMap = lerI16(d, 2) / 10.0f;
      ecuMat = fx10ParaC(lerI16(d, 4));
      ecuClt = fx10ParaC(lerI16(d, 6));
      break;
    case 3:  // tps(0-1, /10 %) batt(2-3, /10 V) ...
      ecuTps  = lerI16(d, 0) / 10.0f;
      ecuBatt = lerI16(d, 2) / 10.0f;
      ecuafr1  = lerI16(d, 4) / 10.0f;
      ecuafr2 = lerI16(d, 6) / 10.0f;
      break;
    case 15:  // sensors9 (bytes 0-1) = adc6 do MS2 = pressão de óleo, com sinal, /10
      ecuP_oleo = lerI16(d, 0) / 10.0f;
      break;
    default:
      return;  // outros grupos não são usados por enquanto
  }

  ecuUltimoRxMs = millis(); // marca que a ECU está viva
}

// =================================================================
// TASK CAN (Core 0): recebe frames e atualiza as variáveis da ECU
// =================================================================
void tarefaCAN(void *parametro) {
  twai_message_t msg;

  for (;;) {
    // Bloqueia até chegar um frame (ou 100 ms), sem gastar CPU à toa
    if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
      // Só frames de dados, ID padrão (11 bits), com os 8 bytes do broadcast
      if (msg.extd || msg.rtr || msg.data_length_code < 8) continue;
      decodificarFrameMS2(msg.identifier, msg.data);
    }
  }
}

// =================================================================
// TASK 2 (Core 0): acumula em buffer e grava no SD em lotes
// =================================================================
void tarefaGravacaoSD(void *parametro) {
  Amostra buffer[TAMANHO_LOTE];
  int indice = 0;

  Amostra recebida;

  for (;;) {
    // Espera bloqueada até chegar uma amostra na fila (não gasta CPU à toa)
    if (xQueueReceive(filaAmostras, &recebida, portMAX_DELAY) == pdTRUE) {
      buffer[indice] = recebida;
      indice++;

      if (indice >= TAMANHO_LOTE) {
        gravarLoteNoSD(buffer, indice);
        indice = 0;
      }
    }
  }
}

void gravarLoteNoSD(Amostra *buffer, int quantidade) {
  unsigned long inicio = millis();

  File arquivo = SD.open(nomeArquivo, FILE_APPEND);
  if (!arquivo) {
    DEBUG_PRINTLN("ERRO: falha ao abrir o arquivo para gravação em lote.");
    return;
  }

  for (int i = 0; i < quantidade; i++) {
    arquivo.print(buffer[i].tempo_ms);
    arquivo.print(";");
    arquivo.print(buffer[i].accelX, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].accelY, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].accelZ, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].gyroX, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].gyroY, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].gyroZ, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].esterco, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].pressao, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].pressao2, 3);
    arquivo.print(";");
    arquivo.print(buffer[i].rpm, 0);
    arquivo.print(";");
    arquivo.print(buffer[i].tps, 1);
    arquivo.print(";");
    arquivo.print(buffer[i].mapKpa, 1);
    arquivo.print(";");
    arquivo.print(buffer[i].cltC, 1);
    arquivo.print(";");
    arquivo.print(buffer[i].matC, 1);
    arquivo.print(";");
    arquivo.print(buffer[i].battV, 2);
    arquivo.print(";");
    arquivo.print(buffer[i].P_oleo, 1);
    arquivo.print(";");
    arquivo.print(buffer[i].afr1, 2);
    arquivo.print(";");
    arquivo.println(buffer[i].afr2, 2);
  }

  arquivo.close();

}