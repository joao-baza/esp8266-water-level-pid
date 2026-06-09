# Controle de Nível — PID

Sistema embarcado de controle de nível de água com **ESP8266**, sensor ultrassônico **HC-SR04**, bomba com **PWM** e alarme sonoro. Inclui painel web em tempo real via **WebSocket**, com modos manual e automático (PID), gráfico histórico e ajuste de ganhos pelo navegador.

## Funcionalidades

- **Modo Manual** — controle direto da potência da bomba (0–100 %)
- **Modo Auto** — controle PID para manter o nível no setpoint desejado
- **Transição suave** — ao alternar Manual ↔ Auto, evita saltos bruscos na saída (bumpless transfer)
- **Filtro EMA** — suavização da leitura de nível, com fator α ajustável
- **Alarme sonoro** — buzzer com intensidade progressiva quando o nível se aproxima do fundo ou do topo (≤ 2 cm)
- **Proteção do sensor** — bomba desligada automaticamente ao atingir 15 cm (protege o HC-SR04)
- **Painel web** — leituras ao vivo, sliders, ganhos PID e gráfico de 180 s (nível, setpoint, PWM e erro %)
- **Sensor não bloqueante** — máquina de estados + ISR no pino Echo, sem travar o loop principal

## Hardware


| Componente                        | Função                               |
| --------------------------------- | ------------------------------------ |
| ESP8266 (NodeMCU / Wemos D1 Mini) | Microcontrolador e servidor web      |
| HC-SR04                           | Medição de distância / nível de água |
| Bomba DC + driver/MOSFET          | Enchimento do tanque (PWM)           |
| Buzzer passivo ou ativo           | Alerta sonoro (PWM)                  |
| Fonte e regulador 3,3 V           | Alimentação do ESP8266               |


### Ligação dos pinos


| Pino ESP8266 | GPIO   | Dispositivo     |
| ------------ | ------ | --------------- |
| D6           | GPIO12 | HC-SR04 Trigger |
| D7           | GPIO13 | HC-SR04 Echo    |
| D1           | GPIO5  | Bomba (PWM)     |
| D2           | GPIO4  | Buzzer (PWM)    |


> O pino Echo deve suportar interrupções. GND e VCC do HC-SR04 conforme a lógica do seu módulo (5 V no Trigger/Echo costuma funcionar com divisor de tensão no Echo).

## Estrutura do projeto

```
controle/
├── README.md
└── main/
    ├── main.ino          # Firmware (PID, sensor, WebSocket)
    └── data/             # Arquivos servidos pelo LittleFS
        ├── index.html
        ├── script.js
        └── style.css
```

## Dependências (Arduino)

Instale via **Gerenciador de Bibliotecas** ou clone manualmente:


| Biblioteca                                                          | Uso                                            |
| ------------------------------------------------------------------- | ---------------------------------------------- |
| [ESP8266WiFi](https://github.com/esp8266/Arduino)                   | Core ESP8266 (incluído no board package)       |
| [ESPAsyncTCP](https://github.com/me-no-dev/ESPAsyncTCP)             | TCP assíncrono                                 |
| [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) | Servidor HTTP + WebSocket                      |
| LittleFS                                                            | Sistema de arquivos (incluído no core ESP8266) |


### Board package

1. **Arduino IDE** → Preferências → URL adicional:
  ```
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
  ```
2. **Ferramentas → Placa** → **LOLIN(WEMOS) D1 R2 & mini** (ou equivalente ESP8266)
3. **Flash Size**: pelo menos **4 MB (FS:2MB OTA:~1019KB)** ou similar com partição para LittleFS

## Configuração

### 1. Credenciais Wi-Fi

Copie o template e preencha com os dados da sua rede:

```bash
cp main/secrets.h.example main/secrets.h
```

Edite `main/secrets.h` (arquivo local, não versionado):

```cpp
const char* ssid     = "SUA_REDE";
const char* password = "SUA_SENHA";
```

### 2. Calibração do tanque

Ajuste os parâmetros físicos conforme seu cilindro:

```cpp
#define TANK_HEIGHT        15.5f   // altura útil (cm)
#define RAW_AT_EMPTY       16.28f  // distância HC-SR04 com tanque vazio
#define RAW_AT_FULL         1.7f   // distância HC-SR04 com tanque cheio
#define PUMP_CUTOFF_LEVEL_CM  15.0f // desliga bomba acima deste nível
```

Meça a distância bruta do sensor com o tanque vazio e cheio e substitua `RAW_AT_EMPTY` e `RAW_AT_FULL`.

### 3. Ganhos PID padrão

Valores iniciais no firmware:


| Parâmetro      | Valor |
| -------------- | ----- |
| Kp             | 5,0   |
| Ki             | 0,5   |
| Kd             | 0,0   |
| α (filtro EMA) | 0,3   |


Podem ser alterados em tempo real pelo painel web.

## Upload do firmware e do painel web

### Firmware

1. Abra `main/main.ino` no Arduino IDE
2. Selecione a placa ESP8266 correta e a porta serial
3. Compile e envie (**Upload**)

### Arquivos web (LittleFS)

Os arquivos em `main/data/` precisam ser gravados na partição LittleFS:

1. Instale a ferramenta **ESP8266 LittleFS Data Upload**
2. Com o sketch aberto em `main/`, use **Ferramentas → ESP8266 LittleFS Data Upload** (ou equivalente)
3. Aguarde o upload concluir e reinicie o ESP8266

Após conectar à rede Wi-Fi, anote o IP exibido no **Monitor Serial** (115200 baud) e acesse no navegador:

```
http://<IP_DO_ESP8266>/
```

## Painel web

O frontend conecta automaticamente via WebSocket em `ws://<IP>/ws` e recebe atualizações a cada leitura válida do sensor (~500 ms).


| Controle                  | Descrição                        |
| ------------------------- | -------------------------------- |
| Manual / Auto             | Modo de operação                 |
| Alarme ligado / desligado | Habilita ou silencia o buzzer    |
| Setpoint                  | Nível desejado (cm)              |
| Bomba PWM (manual)        | Potência da bomba no modo Manual |
| Kp, Ki, Kd                | Ganhos do controlador PID        |
| α filtro                  | Suavização da leitura (0,05–1)   |


O gráfico exibe, nos últimos **180 segundos**:

- **Nível** (cm) — eixo esquerdo
- **Setpoint** (cm) — tracejado
- **PWM bomba** (%) — eixo direito
- **Erro** (%) — |nível − setpoint| normalizado pela altura do tanque

## Protocolo WebSocket

Mensagens **Frontend → ESP** (texto):


| Comando           | Ação                         |
| ----------------- | ---------------------------- |
| `getValues`       | Solicita estado atual        |
| `1s<v>`           | *PWM* manual (0 - 100%)      |
| `sp<v>`           | *Setpoint* (cm)              |
| `kp<v>`           | Ganho proporcional           |
| `ki<v>`           | Ganho integral               |
| `kd<v>`           | Ganho derivativo             |
| `em<v>`           | Alpha do filtro EMA (0,05–1) |
| `modem`           | Modo Manual                  |
| `modea`           | Modo Auto                    |
| `alrm1` / `alrm0` | Alarme ligado / desligado    |


Resposta **ESP → Frontend** (JSON):

```json
{
  "h": 7.25,
  "raw": 9.12,
  "sp": 8.0,
  "pwm": 42,
  "manualPwm": 0,
  "mode": "auto",
  "kp": 5.0,
  "ki": 0.5,
  "kd": 0.0,
  "alpha": 0.3,
  "buz": 0,
  "alrm": 1,
  "status": "ok",
  "tank": 15.5,
  "t": 123456
}
```


| Campo    | Significado                                |
| -------- | ------------------------------------------ |
| `h`      | Nível filtrado (cm)                        |
| `raw`    | Distância bruta do HC-SR04 (cm), ou `null` |
| `sp`     | *Setpoint* (cm)                            |
| `pwm`    | Saída atual da bomba (%)                   |
| `status` | `waiting`, `ok` ou `timeout`               |
| `buz`    | Intensidade do alerta (0 - 100%)           |


## Como funciona o controle

```
HC-SR04 → distância bruta → conversão p/ nível (cm) → filtro EMA
                                                      ↓
                              ┌── Modo Manual → PWM do slider
                              └── Modo Auto   → PID → PWM bomba
                                                      ↓
                              Intertravamento: PWM = 0 se nível ≥ 15 cm
```

O **PID** usa *anti-windup* condicional. O ***buzzer*** calcula uma rampa linear quando o nível está a ≤ 2 cm do fundo ou do topo. Em timeout do sensor, a leitura é descartada sem corromper o filtro.

## Solução de problemas


| Sintoma                   | Possível causa                                             |
| ------------------------- | ---------------------------------------------------------- |
| Página não abre           | LittleFS não gravado ou IP incorreto                       |
| "Reconectando…" no painel | ESP offline ou firewall bloqueando WebSocket               |
| `status: timeout`         | Sensor mal posicionado, cabo longo ou superfície irregular |
| Nível impreciso           | Recalibrar `RAW_AT_EMPTY` e `RAW_AT_FULL`                  |
| Bomba não liga no Auto    | Setpoint acima do nível atual; verificar Kp/Ki/Kd          |
| Bomba desliga cedo        | `PUMP_CUTOFF_LEVEL_CM` atingido (proteção do sensor)       |


## Licença

Projeto de uso livre. Adapte pinos, calibração e ganhos PID ao seu hardware.