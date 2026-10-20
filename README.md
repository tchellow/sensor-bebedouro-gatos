# Sensor para Bebedouro dos Gatos

Firmware para ESP-01 (ESP8266) que liga a fonte/bebedouro dos gatos automaticamente quando um gato se aproxima. A distância é medida por um sensor laser VL53L0X e a bomba é acionada por um relé. A configuração é feita por uma página web servida pelo próprio ESP, em modo Access Point.

## Hardware

| Pino ESP-01 | Função | Observação |
|---|---|---|
| GPIO1 (TX) | SDA do VL53L0X | Por isso o `Serial` **não** é usado |
| GPIO2 | SCL do VL53L0X | |
| GPIO0 | Relé (ativo em LOW) | LOW = ligado |
| GPIO3 (RX) | Botão de factory reset | `INPUT_PULLUP`, pressionar = LOW |

## Como funciona

1. **Leitura:** o VL53L0X é lido a cada 40 ms (~25 Hz), com alcance de até 125 cm. Uma distância só é aceita depois de 2 leituras válidas seguidas. Depois de 5 leituras inválidas, a distância passa a "fora de alcance".
2. **Máquina de estados:**
   - `IDLE`: aguardando.
   - `PRESENCE_COUNTING`: algo está mais perto que a *Distância p/ Acionar* e conta-se a *Permanência p/ Acionar*.
   - `TRIGGERED`: o relé liga. Quando o gato sai, começa a contagem do *Tempo Acionado após Ausência* e, ao terminar, o relé desliga.
   - `MANUAL`: o relé é ligado pelo botão da página web e fica ligado até ser parado.
   - `BLOCKED`: se o relé ficar ligado mais que o *Tempo Máx. de Acionamento*, ele desliga e fica bloqueado pelo *Tempo de Bloqueio*. É uma proteção para a bomba.
3. **Interface web:** o ESP cria uma rede Wi-Fi `SENSOR_XXXX` (os 4 últimos dígitos do MAC), com senha padrão `12345678`. Acesse `http://192.168.4.1` ou `http://sensor.net`. A página mostra o status em tempo real e permite alterar os parâmetros, o SSID e a senha.
4. **Persistência:** os parâmetros do sensor ficam na EEPROM e o SSID/senha do AP ficam em `/config_ap.json` no LittleFS.
5. **Factory reset:** segure o botão em GPIO3 por 5 s para restaurar os padrões.

### Valores de fábrica

| Parâmetro | Padrão |
|---|---|
| Distância p/ Acionar | 50 cm |
| Permanência p/ Acionar | 3 s |
| Tempo Acionado após Ausência | 10 s |
| Tempo Máx. de Acionamento | 3 min |
| Tempo de Bloqueio | 2 min |

### Endpoints HTTP

| Rota | Método | Descrição |
|---|---|---|
| `/` | GET | Página de configuração |
| `/status` | GET | Estado atual em JSON (consultado a cada 350 ms) |
| `/distance` | GET | Distância e estado de acionamento |
| `/getParams` | GET | Parâmetros atuais |
| `/setParams` | POST | Salva os parâmetros (JSON) |
| `/manualStart` / `/manualStop` | POST | Liga/desliga o acionamento manual |

## Arquivos

- `sensor_gatos_ESP01_VL53L0X_v1.9.6.2_relay_v4.ino`: todo o firmware (sensor, máquina de estados, servidor web, página HTML embutida, EEPROM/LittleFS, botão de reset).
- `.vscode/c_cpp_properties.json`: configuração do IntelliSense do VS Code para o core ESP8266.

## Compilação

- Arduino IDE ou `arduino-cli` com o core **esp8266 3.1.2**, placa *Generic ESP8266 Module* (`esp8266:esp8266:generic`).
- Bibliotecas: **ArduinoJson** (v7) e **VL53L0X** (Pololu).

```sh
arduino-cli compile --fqbn esp8266:esp8266:generic .
```
