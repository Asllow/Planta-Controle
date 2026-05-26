# Sistema de Controle Digital PI para Bancada de Motor DC

Este repositório contém o firmware para o microcontrolador ESP32-S3, responsável pelo controle de velocidade em malha fechada de um motor DC acoplado a um tacogerador. O projeto implementa um controlador PI Digital discreto com monitoramento via telemetria Wi-Fi.

## 📋 Funcionalidades

### Controle
- **Malha Fechada (PI Digital):** Implementação de controlador discretizado (ZOH, $T_s = 5ms$) com método de Síntese Direta.
- **Anti-Windup:** Proteção contra saturação do integrador utilizando método de *Clamping*.
- **Atuação:** PWM de alta resolução (1 kHz) via periférico MCPWM.
- **Leitura:** Amostragem de tensão via ADC com calibração (esp_adc_cali).

### Telemetria e Conectividade
- **Wi-Fi Station:** Conexão automática com reconexão resiliente.
- **HTTP Client Otimizado:** Envio de dados em *batch* (lotes) com conexão persistente (*Keep-Alive*) para alta vazão e baixa latência.
- **Protocolo:** JSON contendo Timestamp, Tensão (mV), ADC Raw e Sinal de Controle (%).

### Segurança e Diagnóstico
- **Failsafe (Watchdog):** Desligamento automático do motor em caso de perda de comunicação (> 5s).
- **Modos de Teste:**
    - **Multímetro:** Leitura de tensão com *Peak Hold*.
    - **Varredura (Sweep):** Rampa automática 0-100% para identificação de sistemas.
    - **Calibração:** Rotina automática para detecção de fundo de escala.

## 🛠️ Hardware Necessário

- **MCU:** ESP32-S3 (DevKit).
- **Driver de Motor:** Ponte H ou Driver PWM compatível (3.3V Logic).
- **Sensor:** Tacogerador (Saída analógica 0-3.1V condicionada).
- **Fonte de Alimentação:** Adequada para o motor DC utilizado.

### Pinout (Configuração Padrão)

| Periférico | Pino ESP32 | Função |
|------------|------------|--------|
| Motor PWM  | GPIO 21    | Sinal de Controle (Gate Driver) |
| Tacogerador| GPIO 13    | Entrada Analógica (ADC2_CH2) |

*(Nota: Verifique `control_task.c` para confirmar as definições `MOTOR_PWM_PIN` e canais ADC)*

## 🚀 Como Compilar e Rodar

Este projeto utiliza o **ESP-IDF** (Espressif IoT Development Framework).

### Pré-requisitos
- ESP-IDF v5.0 ou superior instalado.
- Compilador Xtensa (para ESP32/S3).

### Passos
1. **Clonar o repositório:**
   ```bash
   git clone [https://github.com/seu-usuario/planta-controle.git](https://github.com/seu-usuario/planta-controle.git)
   cd planta-controle
   ```

2. **Configurar o Alvo:**
   ```bash
   idf.py set-target esp32s3
   ```

3. **Configurar Wi-Fi e Parâmetros:**
   Abra o `menuconfig` para definir SSID/Senha ou edite diretamente em `main.c` / `wifi_manager.c` (não recomendado para produção).
   ```bash
   idf.py menuconfig
   ```
   *Navegue até: Component config > FreeRTOS > Tick Rate (Hz) e garanta que está em 1000Hz.*

4. **Compilar e Flash:**
   ```bash
   idf.py build flash monitor
   ```

## 📊 Estrutura do Projeto

```
main/
├── control_task.c      # Loop de controle (200Hz), PID e Modos de Teste
├── http_client_task.c  # Gerenciamento de envio de dados (JSON/HTTP)
├── wifi_manager.c      # Máquina de estados da conexão Wi-Fi
├── main.c              # Inicialização do sistema e criação das tasks
└── shared_resources.h  # Filas (Queues), Semáforos e Definições Globais
```

## 📈 Desempenho do Controlador

O sistema foi validado experimentalmente com os seguintes resultados para um degrau de 1.0V -> 1.5V:

- **Tempo de Subida ($t_r$):** ~270 ms
- **Tempo de Acomodação ($t_s$ 5%):** ~365 ms
- **Sobressinal:** < 4.1%

## 📝 Licença

Este projeto é de uso acadêmico/educacional.

---
**Desenvolvido para a disciplina de Sistemas de Controle I.**