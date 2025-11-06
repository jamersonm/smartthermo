// BIBLIOTECAS --------------------------------------------------------------
#include <Arduino.h> 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>                       
#include <DHT.h>
#include <DHT_U.h>
// bibliotecas --------------------------------------------------------------

// CONFIGURACOES ------------------------------------------------------------
#define SERIAL_BAUND 115200

//seletor de dispositivo
//#define USE_SMARTTHERMO_01
#define USE_SMARTTHERMO_02

//chaves de autenticação OTAA
#ifdef USE_SMARTTHERMO_01
  #define APPEUI_KEY 0x32, 0x12, 0x00, 0x00, 0x00, 0x21, 0x33, 0x12                                                          //lsb
  #define DEVEUI_KEY 0x2F, 0x3E, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70                                                          //lsb
  #define APPKEY_KEY 0xE4, 0xCF, 0xEF, 0xE0, 0xA4, 0x74, 0xFB, 0xB9, 0x5B, 0x0A, 0x9F, 0x80, 0x49, 0x9C, 0xE6, 0xF2          //msb
#endif
#ifdef USE_SMARTTHERMO_02
  #define APPEUI_KEY 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22                                                          //lsb
  #define DEVEUI_KEY 0x37, 0x3E, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70                                                          //lsb
  #define APPKEY_KEY 0x11, 0x2B, 0x91, 0x39, 0xE2, 0x45, 0x20, 0x7F, 0x71, 0xBD, 0xD4, 0x79, 0xDB, 0xC9, 0x4F, 0x68          //msb
#endif

//intervalo de envio
const unsigned TX_INTERVAL = 60;

//payload de envio
uint8_t payload[4];

//pinos lora
#define LORA_NSS_PIN  18
#define LORA_RST_PIN  14
#define LORA_DIO0_PIN 26
#define LORA_DIO1_PIN 35
#define LORA_DIO2_PIN 34

//dht type
#define DHTTYPE    DHT11                           // Sensor DHT11
//#define DHTTYPE      DHT22                       // Sensor DHT22 ou AM2302

//dht pino
#define DHTPIN 13
// configuracoes ------------------------------------------------------------

// DHT ----------------------------------------------------------------------
DHT_Unified dht(DHTPIN, DHTTYPE); 
float temp_raw;
float hum_raw;

void dht_setup(){
  dht.begin();
}

void dht_read(){
  sensors_event_t event;
  float last_temp = temp_raw;
  float last_hum = hum_raw;   

  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature))
  {
    temp_raw = event.temperature; 
  }

  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity))
  {
    hum_raw = event.relative_humidity; 
  }
}

void dht_build(){
  dht_read(); 

  // Temperatura
  uint16_t temp_100 = (uint16_t)(temp_raw * 100.0f);
  payload[0] = (temp_100 >> 8) & 0xFF; 
  payload[1] = temp_100 & 0xFF;       

  // Umidade
  uint16_t hum_100 = (uint16_t)(hum_raw * 100.0f);
  payload[2] = (hum_100 >> 8) & 0xFF;
  payload[3] = hum_100 & 0xFF;        
}

void print_readings() {
  Serial.println(F("--- Leituras do Sensor (Raw) ---"));
  Serial.print(F("Temperatura Lida: "));
  Serial.print(temp_raw, 2); // Imprime com 2 casas decimais
  Serial.println(F(" C"));

  Serial.print(F("Umidade Lida: "));
  Serial.print(hum_raw, 2);
  Serial.println(F(" %"));
  Serial.println(F("--------------------------------"));
}

void print_packet() {
  Serial.println(F("--- Payload Embalado (Hex) ---"));
  
  for (int i = 0; i < 4; i++) {
    if (payload[i] < 0x10) {
      Serial.print(F("0")); // Preenche com zero à esquerda se for menor que 16
    }
    Serial.print(payload[i], HEX);
    
    if (i < 3) {
      Serial.print(F(" "));
    }
  }
  Serial.println();
  Serial.println(F("--------------------------------"));
}
// dht ----------------------------------------------------------------------

// LMIC ---------------------------------------------------------------------
static const u1_t PROGMEM APPEUI[8] = { APPEUI_KEY }; //lsb format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8]  = { DEVEUI_KEY }; // lsb format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = { APPKEY_KEY }; //msb format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST_PIN, 
    .dio = {LORA_DIO0_PIN, LORA_DIO1_PIN, LORA_DIO2_PIN},
};

void do_send(osjob_t *j);

// Callback de evento: todo evento do LoRaAN irá chamar essa
// callback, de forma que seja possível saber o status da
// comunicação com o gateway LoRaWAN.
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        LMIC_setLinkCheckMode(0);
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        if (LMIC.dataLen == 1) 
        {
            uint8_t dados_recebidos = LMIC.frame[LMIC.dataBeg + 0];
            Serial.print(F("Dados recebidos: "));
            Serial.write(dados_recebidos);
        }
        // Agenda a transmissão automática com intervalo de TX_INTERVAL
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    //static uint8_t payload[] = "Hello, world!";
    // Verifica se não está ocorrendo uma transmissão no momento TX/RX
    dht_build();
    print_readings();
    print_packet();
    if (LMIC.opmode & OP_TXRXPEND){
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else{
        //envio
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println(F("Sended"));
    }
}
// lmic ---------------------------------------------------------------------

void setup()
{
  Serial.begin(SERIAL_BAUND);
  Serial.println(F("Starting"));

  dht_setup();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  do_send(&sendjob); //Start
}

void loop()
{
    os_runloop_once();
}