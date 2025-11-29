// BIBLIOTECAS --------------------------------------------------------------
#include <Arduino.h> 
#include <Wire.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>                       
#include <DHT.h>
#include <DHT_U.h>
// bibliotecas --------------------------------------------------------------

// CONFIGURACOES ------------------------------------------------------------
#define SERIAL_BAUND 115200

//seletor de dispositivo
//#define USE_SMARTTHERMO_01
#define USE_SMARTTHERMO_02

// pinos
#define PIN_LORA_NSS   18
#define PIN_LORA_RST   14
#define PIN_LORA_DIO0  26
#define PIN_LORA_DIO1  35
#define PIN_LORA_DIO2  34
#define PIN_SDA        4
#define PIN_SCL        15
#define PIN_DHT        13
#define PIN_BUTTON     0
#define PIN_RESET_OLED 16

// lora
// chaves de autenticação OTAA
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
// intervalo de envio
const unsigned TX_INTERVAL = 60;

// sensor
//#define USE_DHT11
#define USE_DHT22

#if defined(USE_DHT11) && defined(USE_DHT22)
  #error "Defina apenas um: USE_DHT11 ou USE_DHT22"
#endif
#if !defined(USE_DHT11) && !defined(USE_DHT22)
  #error "Nenhum sensor definido! Escolha USE_DHT11 ou USE_DHT22"
#endif
#ifdef USE_DHT11
  #define DHTTYPE DHT11
#endif
#ifdef USE_DHT22
  #define DHTTYPE DHT22
#endif
#define DHT_SAMPLE_TIME_MS 2000

// display
#define DISPLAY_LENGTH_LOCAL 128
#define DISPLAY_HEIGHT_LOCAL 64
#define DISPLAY_ADDRESS      0x3C
#define DISPLAY_TIMEOUT_MS   10000
// seletor de orientação
#define DISPLAY_VERTICAL
//#define DISPLAY_HORIZONTAL
// configuracoes ------------------------------------------------------------

// GLOBALS ------------------------------------------------------------------
// flags
volatile bool flag_button_pressed = false;
volatile bool flag_display_on     = false;
volatile bool flag_lora_online    = false;
// temporais
static unsigned long last_reading_time = 0;
static unsigned long oled_on_time      = 0;
// payload para envio
uint8_t payload[4];
// globals -------------------------------------------------------------------

// PROTOTIPOS ----------------------------------------------------------------
// isr
void button_setup();
void button_loop();
// dht
void dht_read();
void dht_build();
void dht_print_values();
void dht_print_packet();
void dht_setup();
void dht_loop();
// display
void display_on();
void display_off();
void display_readings();
void display_setup();
void display_loop();
// lmic
void do_send(osjob_t *j);
void onEvent(ev_t ev);
// prototipos ---------------------------------------------------------------

// ISR ----------------------------------------------------------------------
void IRAM_ATTR ISR_BOTAO() {
  flag_button_pressed = true; 
}

void button_setup(){
  pinMode(PIN_BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), ISR_BOTAO, FALLING);
}

void button_loop(){
  if (flag_button_pressed) {
    flag_button_pressed = false;
    
    // Liga o display e REINICIA o timer
    display_on(); 
    display_readings();
    oled_on_time = millis(); // Inicia a contagem de 10s
  }
}
// isr ----------------------------------------------------------------------

// DHT ----------------------------------------------------------------------
// DHT ----------------------------------------------------------------------
DHT_Unified dht(PIN_DHT, DHTTYPE); 
volatile float temp_raw = 0.0; 
volatile float hum_raw = 0.0;

void dht_read(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    temp_raw = event.temperature; 
  } else {
    temp_raw = 0.0;
  }
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    hum_raw = event.relative_humidity; 
  } else {
    hum_raw = 0.0;
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

void dht_print_values() {
  Serial.println(F("--- Leituras do Sensor (Raw) ---"));
  Serial.print(F("Temperatura Lida: "));
  Serial.print(temp_raw, 2); // Imprime com 2 casas decimais
  Serial.println(F(" C"));

  Serial.print(F("Umidade Lida: "));
  Serial.print(hum_raw, 2);
  Serial.println(F(" %"));
  Serial.println(F("--------------------------------"));
}

void dht_print_packet() {
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

void dht_setup(){
  #ifdef USE_DHT22
    pinMode(PIN_DHT, INPUT_PULLUP);
  #endif
  dht.begin();
}

void dht_loop(){
  if (millis() - last_reading_time >= DHT_SAMPLE_TIME_MS) {
    dht_read();
    //dht_print();
    last_reading_time = millis();
    if (flag_display_on) {
        display_readings();
    }
  }
}
// dht ----------------------------------------------------------------------

// DISPLAY ------------------------------------------------------------------
Adafruit_SSD1306 display(DISPLAY_LENGTH_LOCAL, DISPLAY_HEIGHT_LOCAL, &Wire, PIN_RESET_OLED);

void display_on() {
    if (!flag_display_on) {
        digitalWrite(PIN_RESET_OLED, HIGH); 
        delay(50); // crítico
        
        // re-inicializa a comunicação e os registradores do SSD1306
        display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS); // crítico
        
        display.clearDisplay();
        display.display();
        
        flag_display_on = true;
        //Serial.println("Display LIGADO.");
    }
}

void display_off() {
    if (flag_display_on) {
        display.clearDisplay(); 
        display.display(); 
        
        digitalWrite(PIN_RESET_OLED, LOW); 
        flag_display_on = false;
        //Serial.println("Display DESLIGADO.");
    }
}

void display_readings() {
  
  display.clearDisplay();
  
  #ifdef DISPLAY_HORIZONTAL
  // Título
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TEMPERATURA E UMIDADE");

  // Temperatura
  display.setTextSize(2);
  display.setCursor(0, 15); 
  display.print("T: ");
  display.print(temp_raw, 1);
  display.println(" C"); 

  // Umidade
  display.setTextSize(2);
  display.setCursor(0, 35); 
  display.print("U: ");
  display.print(hum_raw, 1);
  display.println(" %"); 
  
  // Status LoRa
  display.setTextSize(1);
  display.setCursor(0, 56);

  if (flag_lora_online) {
    display.print("Status: ONLINE");
  } else {
    display.print("Status: CONECTANDO");
  }
  #endif
  
  #ifdef DISPLAY_VERTICAL
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Temperatura ---
  display.setTextSize(1);
  display.setCursor(0, 15); 
  display.println("Temp C:");
  display.println("");
  display.setTextSize(2);
  display.print(temp_raw, 2);

  // --- Umidade ---
  display.setTextSize(1);
  display.setCursor(0, 60); 
  display.println("Umid %:");
  display.println("");
  display.setTextSize(2); 
  display.print(hum_raw, 2);

  // --- Status LoRa
  display.setTextSize(1);
  display.setCursor(0, 110); 
  display.println("Status:");

  if (flag_lora_online) {
    display.print("ONLINE");
  } else {
    display.print("CONECTANDO");
  }
  #endif
  
  display.display();
}

void display_setup() {
  // pino de reset do display
  pinMode(PIN_RESET_OLED, OUTPUT);
  digitalWrite(PIN_RESET_OLED, LOW); 
  delay(50);
  digitalWrite(PIN_RESET_OLED, HIGH);
  // inicializacao do display
  if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS)) { 
    Serial.println(F("Falha na inicializacao do SSD1306!"));
    for(;;); 
  }
  
  
  // Exibição inicial SmartThermo
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  #ifdef DISPLAY_HORIZONTAL
  display.setTextSize(2);
  display.setCursor(34, 10);
  display.println("Smart");
  display.setTextSize(2);
  display.setCursor(28, 30);
  display.println("Thermo");
  #endif

  #ifdef DISPLAY_VERTICAL
  // 1 - usb bottom
  // 3 - usb top
  display.setRotation(1); 
  display.setTextSize(2);
  display.setCursor(2, 43); 
  display.println("Smart");
  display.setTextSize(1);
  display.setCursor(14, 65);
  display.println("Thermo");
  #endif

  display.display();

  flag_display_on = true;

  delay(5000);

  display_off();
}

void display_loop() {
  if (flag_display_on) {
    if (millis() - oled_on_time >= DISPLAY_TIMEOUT_MS) {
      display_off(); // Desliga o display
    }
  }
}
// display ------------------------------------------------------------------

// LMIC ---------------------------------------------------------------------
static const u1_t PROGMEM APPEUI[8] = { APPEUI_KEY }; //lsb format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8]  = { DEVEUI_KEY }; // lsb format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = { APPKEY_KEY }; //msb format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
    .nss = PIN_LORA_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PIN_LORA_RST, 
    .dio = {PIN_LORA_DIO0, PIN_LORA_DIO1, PIN_LORA_DIO2},
};

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
        if(!flag_lora_online){
            flag_lora_online = true;
        }
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
    dht_build();
    dht_print_values();
    dht_print_packet();
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

// SETUP --------------------------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_BAUND);
  Serial.println(F("Starting"));
  delay(1000);
  Wire.begin(PIN_SDA, PIN_SCL);

  // setup dos periféricos
  display_setup();
  dht_setup();
  button_setup();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  do_send(&sendjob); //Start
}
// setup --------------------------------------------------------------------

// LOOP ---------------------------------------------------------------------
void loop()
{
    /* a leitura é atualizada a cada envio lora */
    //dht_loop();
    button_loop();
    display_loop();
    os_runloop_once();
}
// loop ---------------------------------------------------------------------