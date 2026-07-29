/*
  ID = 16*MUX + 2*Canal + AD0  | MUX(0-7) Canal(0-7) AD0(0-1)

  ESP32 + TCA9548A + MPU6050
  - Você define INITIAL_NUM_MUX (<=8) no início e o código cria todos os sensores possíveis
  - A cada 120s faz scan de muxes para detectar conexões/desconexões
  
  Comandos: *Algumas IDEs se tu escrever e apagar mas reescrever o comando corretamente ele nao vai reconhecer o comando
  "detect" detecta
  "calibra" calibra
  "reset" reseta
  "resetflash" reseta a memoria de todo o flash
  "off" para todas as funçoes até receber "on"
  "on" nem te conto
  "w" mostra a intensidade do sinal de wifi em mdB, quanto menor melhor e se encontrar nada mostrará 0
  "lista" lista todos os dados de todos mux criados
  "lista d" Lista dados de todos sensores presentes incluindo canal, endereço, raw offsets e scale
  "lista p" mostra todos sensores presentes mas nao detecta se forem desconectados A2 A1 A0 | ch0 ch1 ch2... ch7
*/

#include <WiFi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <Preferences.h>

using std::vector;

// WIFI CONFIG 
#define WIFI_SSID           "ALHN-9400" //nome do wifi com prioridade
#define WIFI_PASSWORD       "23482348"  //senha do wifi com prioridade
#define WIFI_SSID_2         "meu nome"
#define WIFI_PASSWORD_2     "12345678"

#define INFLUXDB_URL        "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_ORG        "f3d1fc827270e716"
#define INFLUXDB_TOKEN      "gslQFbd4dmXb_qGopaLVdtOiVYDnE_2qlZ-FE1JuHXe9Xm7pEjjEfAowDaQ0Yj-lcsbEXGlydBV8dFe0nnSovA=="
#define INFLUXDB_BUCKET     "MPU6090 Bucket"
#define DEVICE              "ESP32"
#define INFLUXDB_SEND_TIME  5000u // ms

// I2C CONFIG 
#define I2C_SDA_PIN   21   
#define I2C_SCL_PIN   22   
#define I2C_FREQ_HZ   400000  

// I2C MUX and MPU addresses
#define BASE_MUX_ADDR   0x70
#define MAX_MUX         8      // addresses 0x70 .. 0x77
#define MUX_CHANNELS    8
#define MPU_ADDR_0      0x68
#define MPU_ADDR_1      0x69

// Quantos muxes você quer criar no início (valor entre 1 e 8)
// Esses serão criados no setup; o scanner periódico pode descobrir mais até o limite 0x70..0x77.
static uint8_t INITIAL_NUM_MUX = 2; // <-- Mude aqui conforme necessidade 
#define DEFAULT_SAMPLES_CALIB 1000
#define DEFAULT_SAMPLES_RMS   100

// InfluxDB 
WiFiMulti wifiMulti;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point pointSensor("sensor_data");
Point pointNet("network_status");
static uint32_t lastInfluxMs = 0u;

Preferences prefs;

//Sensor struct
struct Sensor {
  uint8_t muxAddr;   // 0x70..0x77
  uint8_t channel;   // 0..7 do mux
  uint8_t addr;      // MPU address 0x68/0x69
  int16_t offx;
  int16_t offy;
  int16_t offz;

  float rms;         // ultimo rms calculado
  float rmsSum;      // acumulador
  int16_t rmsCount;  // contador de samples

  float scale;
  float ax;
  float ay;
  float az;

  bool present;      // sensor existe?
  bool known;        // se já temos prefs / calibrado

  Sensor()
    : muxAddr(0), channel(0), addr(0),
      offx(0), offy(0), offz(0),
      rms(0.0f), rmsSum(0.0f), rmsCount(0),
      scale(-1.0f), ax(-1.0f), ay(-1.0f), az(-1.0f),
      present(false), known(false)
  {}
};

static vector<Sensor> sensors;

// Sampling params
static int32_t samplesCalib = DEFAULT_SAMPLES_CALIB;
static int32_t samplesRMS   = DEFAULT_SAMPLES_RMS;

// Mux scan timer 
static uint32_t lastMuxScanMs = 0;
static uint32_t MUX_SCAN_INTERVAL_MS = 120000; // 120 segundos

static bool timeOK = false;
static bool influxOK = false;
static uint32_t lastNtpTry = 0;

// Funções
void selectMux(uint8_t muxAddr, uint8_t channel);
void deselectMux(uint8_t muxAddr);
int32_t sensorIndex(uint8_t muxAddr, uint8_t channel, uint8_t addr);
bool carregarSensorFlash(Sensor &s);
void salvarSensorFlash(Sensor &s);
void apagarFlashTotal();
void detectarSensores(); // varre muxes e canais e atualiza sensors[].present
void axisRefresh(Sensor &s);
void calibrarSensor(Sensor &s, int32_t samples);
void calibrarTodosSensores(int32_t samples);
void Influx_init();
void Influx_task();
void sendAllToInflux();
void comandos();
void printRMS(int32_t index);
void printSensorInfo(Sensor &s, int32_t idx);
void checkWiFi();
void createSensorsForMux(uint8_t muxAddr);
void scanMuxesPeriodic(); // checa conexões/desconexões de muxes (a cada 120s) use isso no loop ao invez do detect para hotswap
void ntpTask();


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  prefs.begin("mpu_prefs", false);

  Serial.println("Inicializando sensores possiveis...");
  
  //cria quantidade de mux dos define
  uint8_t num = INITIAL_NUM_MUX;
  if (num < 1) num = 1;
  if (num > MAX_MUX) num = MAX_MUX;
  for (uint8_t i = 0; i < num; ++i) {
    uint8_t muxAddr = BASE_MUX_ADDR + i;
    createSensorsForMux(muxAddr);
  }

  Serial.printf("Criados %u sensores (para %u muxes iniciais).\n", (unsigned)sensors.size(), (unsigned)num);


  Serial.println("Conectando WiFi...");
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_PASSWORD_2);
  checkWiFi();

  //tempo desregulado pro wifi nao sei pq
  struct tm tm;
  tm.tm_year = 2024 - 1900;
  tm.tm_mon  = 0;
  tm.tm_mday = 1;
  tm.tm_hour = 0;
  tm.tm_min  = 0;
  tm.tm_sec  = 0;
  time_t fake = mktime(&tm);
  struct timeval tv;
  tv.tv_sec = fake;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  Serial.println("Hora base configurada (fallback TLS)");

  Influx_init();

  detectarSensores();
  lastMuxScanMs = millis();
  Serial.println("Setup completo.");
}

void loop(){  //42ms per loop 

  checkWiFi();
  ntpTask();
  Influx_init();
  Influx_task();

  // faz scan a cada 120s de muxes para detectar novos/desconectados
  // scanMuxesPeriodic();

  //  atualiza RMS para todos sensores presentes e printa
  for (int32_t i = 0; i < (int32_t)sensors.size(); ++i) {
    if (sensors[i].present) {
        axisRefresh(sensors[i]);  // Lê sensor
        printRMS(i);              // Calcula RMS
    }
  }
  
  comandos();
}

// Escolhe mux e canal pra testar sensor especifico
void selectMux(uint8_t muxAddr, uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(1);
}
// remove comunicação com um mux especifico
void deselectMux(uint8_t muxAddr) {
  Wire.beginTransmission(muxAddr);
  Wire.write(0x00); // disable all channels
  Wire.endTransmission();
  delay(1);
}
 
// cria sensores (todos canais x 2 endereços) para um muxAddr se não existirem
void createSensorsForMux(uint8_t muxAddr) {
  for (uint8_t ch = 0; ch < MUX_CHANNELS; ++ch) {
    for (uint8_t addr : {MPU_ADDR_0, MPU_ADDR_1}) {
      if (sensorIndex(muxAddr, ch, addr) < 0) {
        Sensor s;
        s.muxAddr = muxAddr;
        s.channel = ch;
        s.addr = addr;
        // offxyz scale já inicializados no construtor
        sensors.push_back(s);
        // não marcamos present aqui pq detectarSensores fará isso
      }
    }
  }
}

// Retorna o ID do sensor
int32_t sensorIndex(uint8_t muxAddr, uint8_t channel, uint8_t addr) {
  for (int32_t i = 0; i < (int32_t)sensors.size(); ++i) {
    Sensor &s = sensors[i];
    if (s.muxAddr == muxAddr && s.channel == channel && s.addr == addr) return i;
  }
  return -1;
}

// Salva dados de sensores detectados e calibrados no flash 
// "s_scle" e "s_vald" ta assim pois há limite de char
void salvarSensorFlash(Sensor &s) {
  char base[48];
  snprintf(base, sizeof(base), "M%02X_C%u_A%02X", s.muxAddr, s.channel, s.addr);
  char key[64];
  snprintf(key, sizeof(key), "%s_offx", base); prefs.putInt(key, (int32_t)s.offx);
  snprintf(key, sizeof(key), "%s_offy", base); prefs.putInt(key, (int32_t)s.offy);
  snprintf(key, sizeof(key), "%s_offz", base); prefs.putInt(key, (int32_t)s.offz);
  snprintf(key, sizeof(key), "%s_scle", base); prefs.putFloat(key, s.scale);
  snprintf(key, sizeof(key), "%s_vald", base); prefs.putInt(key, 1);
  Serial.printf("Saved sensor: %s\n", base);
}
// carrega os dados do flash
bool carregarSensorFlash(Sensor &s) {
  char base[48];
  snprintf(base, sizeof(base), "M%02X_C%u_A%02X", s.muxAddr, s.channel, s.addr);
  char key[64];
  snprintf(key, sizeof(key), "%s_vald", base);
  if (!prefs.getInt(key, 0)) return false;
  snprintf(key, sizeof(key), "%s_offx", base); s.offx = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_offy", base); s.offy = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_offz", base); s.offz = prefs.getInt(key, 0);
  snprintf(key, sizeof(key), "%s_scle", base); s.scale = prefs.getFloat(key, -1.0f);
  s.known = true;
  return true;
}
// adivinha
void apagarFlashTotal() {
  prefs.clear();
  Serial.println("Flash apagada.");
}

// varre mux addresses & channels
void detectarSensores() {
  // marca todos como não presentes;
  for (auto &s : sensors) s.present = false;

  // varre todo range de muxes (0x70 -> 0x77)
  for (uint8_t muxOff = 0; muxOff < MAX_MUX; ++muxOff) {
    uint8_t muxAddr = BASE_MUX_ADDR + muxOff;

    // verifica se o mux responde (endereço presente)
    Wire.beginTransmission(muxAddr);
    if (Wire.endTransmission()) {
      continue; 
    }

    // se o mux responde, varre canais
    for (uint8_t ch = 0; ch < MUX_CHANNELS; ++ch) {
      selectMux(muxAddr, ch);
      for (uint8_t addr : {MPU_ADDR_0, MPU_ADDR_1}) {
        // se não existe sensor na lista para este mux+ch+addr, crie (novo mux detectado)
        if (sensorIndex(muxAddr, ch, addr) < 0) {
          Serial.printf("Novo mux detectado em 0x%02X — criando sensores CH%u ADDR 0x%02X\n", muxAddr, ch, addr);
          createSensorsForMux(muxAddr); // cria todos canais deste mux 
        }

        int32_t idx = sensorIndex(muxAddr, ch, addr);
        if (idx < 0) continue; // por segurança

        // testa se o sensor responde no endereço do MPU
        Wire.beginTransmission(addr);
        uint8_t res = Wire.endTransmission();
        if (res == 0) {
          sensors[idx].present = true;
          if (!sensors[idx].known) {
            if (carregarSensorFlash(sensors[idx])) {
              Serial.printf("Sensor conhecido carregado: MUX 0x%02X CH%u 0x%02X\n", muxAddr, ch, addr);
            } else {
              Serial.printf("Novo sensor detectado: MUX 0x%02X CH%u 0x%02X — calibrando...\n", muxAddr, ch, addr);
              
              calibrarSensor(sensors[idx], samplesCalib);
              salvarSensorFlash(sensors[idx]);
              sensors[idx].known = true;
            }
          }
        }
      } // end for addrs
      deselectMux(muxAddr);
    } // end for channels
  } // end for mux addresses
}

// Read 
void axisRefresh(Sensor &s) {
  if (!s.present) return;

  selectMux(s.muxAddr, s.channel);

  // acorda MPU6050
  Wire.beginTransmission(s.addr);
  Wire.write(0x6B);        // não sei
  Wire.write(0x00);        // não pergunta e não toca
  if (Wire.endTransmission() != 0) {
    s.present = false;
    deselectMux(s.muxAddr);
    return;
  }
  delayMicroseconds(500);  // só pra ficar esperto

  // inicializa leitura do sensor
  Wire.beginTransmission(s.addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    s.present = false;
    deselectMux(s.muxAddr);
    return;
  }
// o resto só lê o valor do sensor e ajusta pra m/s2 e calibra
  uint8_t got = Wire.requestFrom((uint16_t)s.addr, (uint8_t)6, true);
  if (got < 6) {
    s.present = false;
    deselectMux(s.muxAddr);
    return;
  }

  int16_t rx = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ry = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t rz = (int16_t)((Wire.read() << 8) | Wire.read());

  int32_t cx = (int32_t)rx - s.offx;
  int32_t cy = (int32_t)ry - s.offy;
  int32_t cz = (int32_t)rz - s.offz;

  if (s.scale > 0.0f) {
    s.ax = (float)cx * s.scale;
    s.ay = (float)cy * s.scale;
    s.az = (float)cz * s.scale;
  } else {
    s.ax = s.ay = s.az = -1.0f;
  }

  deselectMux(s.muxAddr);
}

// calibra um sensor especifico "samples" vezes
void calibrarSensor(Sensor &s, int32_t samples) {
  if (!s.present) return;

  selectMux(s.muxAddr, s.channel);

  // acorda MPU6050
  Wire.beginTransmission(s.addr);
  Wire.write(0x6B);        // PWR_MGMT_1
  Wire.write(0x00);        // wake up
  if (Wire.endTransmission() != 0) {
    deselectMux(s.muxAddr);
    return;
  }
  delay(10);               // aumente o valor pra calibração mais precisa mas mais lenta

  int64_t sumX = 0, sumY = 0, sumZ = 0;
  int32_t valid = 0;

  for (int32_t i = 0; i < samples; ++i) {
    Wire.beginTransmission(s.addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      --i;
      yield(); // nao faz tudo para e morre
      continue;
    }

    uint8_t got = Wire.requestFrom((uint16_t)s.addr, (uint8_t)6, true);
    if (got == 6) {
      int16_t rx = (int16_t)((Wire.read() << 8) | Wire.read());
      int16_t ry = (int16_t)((Wire.read() << 8) | Wire.read());
      int16_t rz = (int16_t)((Wire.read() << 8) | Wire.read());
      sumX += rx;
      sumY += ry;
      sumZ += rz;
      ++valid;
    } else {
      --i;
      yield();
    }
  }

  if (valid > 0) {
    s.offx = (int32_t)(sumX / valid);
    s.offy = (int32_t)(sumY / valid);
    s.offz = (int32_t)(sumZ / valid);

    float mag = sqrt(
      (float)s.offx * (float)s.offx +
      (float)s.offy * (float)s.offy +
      (float)s.offz * (float)s.offz
    );

    s.scale = (mag > 0.0f) ? (9.81f / mag) : -1.0f; //converte Gs para m/s2 e verifica erro se scale<=0
  }

  deselectMux(s.muxAddr);
}

// Inicializa Influx 
void Influx_init() {
  // WiFi obrigatório
  if (WiFi.status() != WL_CONNECTED) return;

  // Hora obrigatória (TLS precisa disso)
  if (!timeOK) return;

  // Já conectado? sai
  if (influxOK) return;

  client.setWriteOptions(
    WriteOptions()
      .batchSize(50)
      .flushInterval(0)
      .retryInterval(5000)
  );

  pointSensor.addTag("device", DEVICE);
  pointNet.addTag("device", DEVICE);

  Serial.println("Tentando conectar Influx...");

  if (!client.validateConnection()) {
    Serial.print("Influx falhou: ");
    Serial.println(client.getLastErrorMessage());
    return;
  }

  influxOK = true;
  Serial.println("Influx conectado com sucesso.");
}

void sendAllToInflux() {
  if (!WL_CONNECTED) return;

  // necessario pra nao entupir
  pointNet.clearFields();
  
  pointNet.addField("rssi", WiFi.RSSI());
  client.writePoint(pointNet);
  
  

  //  SENSOR DATA 
  for (uint8_t muxOff = 0; muxOff < MAX_MUX; ++muxOff) {
    uint8_t muxAddr = BASE_MUX_ADDR + muxOff;

    for (uint8_t ch = 0; ch < MUX_CHANNELS; ++ch) {
      for (uint8_t addrIndex = 0; addrIndex < 2; ++addrIndex) {

        uint8_t addr = addrIndex == 0 ? MPU_ADDR_0 : MPU_ADDR_1;
        uint8_t sensorNum = ch + (addr == MPU_ADDR_1 ? 8 : 0);

        int32_t idx = sensorIndex(muxAddr, ch, addr);
        if (idx < 0 || !sensors[idx].present) continue;

        Sensor &s = sensors[idx];

        Point p("sensor_data");

        // TAGS
        p.addTag("device", DEVICE);

        char muxTag[8];
        snprintf(muxTag, sizeof(muxTag), "MUX%u", muxOff);
        p.addTag("mux", muxTag);

        char sensorTag[8];
        snprintf(sensorTag, sizeof(sensorTag), "S%u", sensorNum);
        p.addTag("sensor", sensorTag);

        // CAMPOS
        p.addField("ax",  s.ax);
        p.addField("ay",  s.ay);
        p.addField("az",  s.az);
        p.addField("rms", s.rms);

        
        if (!client.writePoint(p)) Serial.println(client.getLastErrorMessage());
         
      }
    }
  }

  client.flushBuffer();
  Serial.print(WiFi.RSSI());
  // foi feita tentativa de envio mas nao necessariamente foi enviado
  Serial.println("dBm | Influx flush done"); 

}
//millis pro sendAllToInflux
void Influx_task() {
  if (!influxOK) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastInfluxMs >= INFLUXDB_SEND_TIME) {
    lastInfluxMs = millis();
    sendAllToInflux();
  }
}

// printa valores medidos no serial monitor e outras info 
void printSensorInfo(Sensor &s, int32_t idx) {
  Serial.printf("S%03d -> MUX 0x%02X CH%u ADDR 0x%02X pres=%d scale=%.6f off=[%ld,%ld,%ld] ax=%.3f ay=%.3f az=%.3f rms=%.6f\n",
                idx + 1, s.muxAddr, s.channel, s.addr, (int32_t)s.present, s.scale,
                (long)s.offx, (long)s.offy, (long)s.offz,
                s.ax, s.ay, s.az, s.rms);
}

//comandos pra escrever no serial monitor
void comandos() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();  

  //  DETECT 
  if (input == "detect") {
    detectarSensores();
    for (uint32_t i = 0; i < sensors.size(); ++i)
      printSensorInfo(sensors[i], (int32_t)i);

  //  LISTA Presentes
  } else if (input == "lista p") {
    for (uint8_t muxOff = 0; muxOff < MAX_MUX; ++muxOff) {
      uint8_t muxAddr = BASE_MUX_ADDR + muxOff;
      bool anySensor = false;
      uint8_t countPerChannel[8] = {0};

      for (uint32_t i = 0; i < sensors.size(); ++i) {
        if (sensors[i].present && sensors[i].muxAddr == muxAddr) {
          countPerChannel[sensors[i].channel]++;
          anySensor = true;
        }
      }

      if (!anySensor) continue;

      uint8_t aBits = muxAddr & 0x07;
      Serial.printf("%03u | ", aBits);

      for (uint8_t ch = 0; ch < 8; ++ch)
        Serial.printf("%01u", countPerChannel[ch]);

      Serial.println();
    }

  //  LISTA Dados
  } else if (input == "lista d") {
    for (uint32_t i = 0; i < sensors.size(); ++i) {
      if (!sensors[i].present) continue;
      Serial.printf(
        "MUX: 0x%02X, CH: %u, ADDR: 0x%02X, offx: %d, offy: %d, offz: %d, scale: %.3f\n",
        sensors[i].muxAddr,
        sensors[i].channel,
        sensors[i].addr,
        sensors[i].offx,
        sensors[i].offy,
        sensors[i].offz,
        sensors[i].scale
      );
    }

  //  CALIBRA (todos ou individual) 
  } else if (input.startsWith("calibra")) {

    int space = input.indexOf(' ');

    //  calibra TODOS 
    if (space == -1) {
      for (uint32_t i = 0; i < sensors.size(); ++i) {
        if (sensors[i].present) {
          calibrarSensor(sensors[i], samplesCalib);
          salvarSensorFlash(sensors[i]);
          sensors[i].known = true;
        }
      }
      Serial.println("Todos os sensores calibrados e salvos.");

    //  calibra INDIVIDUAL 
    } else {
      int idx = input.substring(space + 1).toInt();

      if (idx < 0 || idx >= (int)sensors.size()) {
        Serial.println("Indice de sensor invalido.");
        return;
      }

      if (!sensors[idx].present) {
        Serial.println("Sensor nao presente.");
        return;
      }

      calibrarSensor(sensors[idx], samplesCalib);
      salvarSensorFlash(sensors[idx]);
      sensors[idx].known = true;

      Serial.printf("Sensor %d calibrado e salvo.\n", idx);
    }

  //  OFF / ON 
  } else if (input == "off") {
    Serial.println("Sistema desligado. Digite 'on' para continuar.");
    while (true) {
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        if (cmd == "on") {
          Serial.println("Sistema ligado.");
          break;
        }
      }
      delay(10);
    }

  //  RESET 
  } else if (input == "reset") {
    ESP.restart();

  //  RSSI / WIFI
  } else if (input == "w") {
    Serial.printf("RSSI: %ld dBm\n", WiFi.RSSI());

  //  LISTA (geral) 
  } else if (input == "lista") {
    for (uint32_t i = 0; i < sensors.size(); ++i)
      printSensorInfo(sensors[i], (int32_t)i);

  //  RESET FLASH 
  } else if (input == "resetflash") {
    apagarFlashTotal();
    Serial.println("Flash apagada.");

  //  SLA TCHE 
  } else {
    Serial.println("Comando desconhecido.");
  }
}

//  printa RMS e o resto (por sensor)
void printRMS(int32_t index) {
  if (index < 0 || index >= (int32_t)sensors.size()) { //segurança
    return;
  }

  Sensor &s = sensors[index];

  if (!s.present) {
    return;
  }

  float axv = s.ax;
  float ayv = s.ay;
  float azv = s.az;

  s.rmsSum += (axv * axv + ayv * ayv + azv * azv);
  s.rmsCount++;

  if (s.rmsCount >= samplesRMS) {
    s.rms = sqrt(s.rmsSum / (float)s.rmsCount);
    s.rmsSum = 0.0f;
    s.rmsCount = 0;
    Serial.print(WiFi.RSSI());
    
    // Adiciona um espaço à esquerda de idx se idx <= 9
    //to com preguiza demais pra fazer bonito
    if (index <= 9) {
        Serial.printf(" | RMS(sensor idx  %d -> MUX 0x%02X CH%u ADDR 0x%02X) rms = %.6f | ax %.2f ay %.2f az %.2f \n",
                      index, s.muxAddr, s.channel, s.addr, s.rms, s.ax, s.ay, s.az);
    } else {
        Serial.printf(" | RMS(sensor idx %d -> MUX 0x%02X CH%u ADDR 0x%02X) rms = %.6f | ax %.2f ay %.2f az %.2f \n",
                      index, s.muxAddr, s.channel, s.addr, s.rms, s.ax, s.ay, s.az);
    }
}
}

// Detecta mux e sensores | use isso no loop ao invez do detect para hotswap
void scanMuxesPeriodic() {
  uint32_t now = millis();
  if (now - lastMuxScanMs < MUX_SCAN_INTERVAL_MS) return;
  lastMuxScanMs = now;

  Serial.println("Scan periodic de muxes (0x70..0x77) ...");

  for (uint8_t muxOff = 0; muxOff < MAX_MUX; ++muxOff) {
    uint8_t muxAddr = BASE_MUX_ADDR + muxOff;
    Wire.beginTransmission(muxAddr);
    uint8_t res = Wire.endTransmission();
    if (res == 0) {
      // mux responde: garante sensores criados
      if (sensorIndex(muxAddr, 0, MPU_ADDR_0) < 0) {
        Serial.printf("Novo mux encontrado em 0x%02X — criando sensores.\n", muxAddr);
        createSensorsForMux(muxAddr);
      }
    } else {
      // mux não responde: marca sensores desse mux como desconectados
      bool any = false;
      for (auto &s : sensors) {
        if (s.muxAddr == muxAddr) {
          if (s.present) any = true;
          s.present = false;
        }
      }
      if (any) Serial.printf("Mux 0x%02X desconectado -> marcando sensores como desconectados.\n", muxAddr);
    }
  }
}

// WiFi check 
void checkWiFi() {
  static bool foiConectado = false;
  static uint32_t wifi_millis = 0;
  static uint8_t qual_Wifi_Conectado = 0; // 0 = WIFI 1, 1 = WIFI 2

  if (WiFi.status() == WL_CONNECTED) {
    if (!foiConectado) {
      Serial.printf("WiFi conectado: %s | RSSI: %d dBm\n", WiFi.SSID().c_str(), WiFi.RSSI());
      foiConectado = true;
    }
    return;
  }

  if (foiConectado) {
    Serial.println("WiFi desconectado!");
    foiConectado = false;
  }

  if (millis() - wifi_millis < 5000) return; // 5s após cada tentativa de conecção
  wifi_millis = millis();

  WiFi.disconnect(true);
  delay(100);

  if (qual_Wifi_Conectado == 0) {
    Serial.println("Tentando WiFi 1...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else {
    Serial.println("Tentando WiFi 2...");
    WiFi.begin(WIFI_SSID_2, WIFI_PASSWORD_2);
  }

  qual_Wifi_Conectado ^= 1; // alterna entre 0 e 1
}

// arruma o horario pro wifi
void ntpTask() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 30000) return;
  lastPrint = millis();

  time_t now;
  time(&now);

  Serial.printf("Epoch atual: %ld\n", now);

  if (now < 1600000000) {
    Serial.println("NTP indisponível (hotspot?)");
    timeOK = false;
  } else {
    Serial.println("Hora válida");
    timeOK = true;
  }
}