// BIBLIOTECAS --------------------------------------------------------------
#include "esp32-hal.h"
#include <Arduino.h> 
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>                       
#include <DHT.h>
#include <DHT_U.h>
// bibliotecas --------------------------------------------------------------
 
// CONFIGURACOES ------------------------------------------------------------
#define SERIAL_BAUND 115200
// pinos
#define PIN_SDA 4
#define PIN_SCL 15
#define PIN_DHT 13
#define PIN_BUTTON 0
#define PIN_RESET_OLED 16

// sensor
#define DHTTYPE    DHT11
// display
#define DISPLAY_LENGTH_LOCAL 128
#define DISPLAY_HEIGHT_LOCAL 64
#define DISPLAY_ADDRESS      0x3C
#define DISPLAY_TIMEOUT_MS   10000
#define DISPLAY_VERTICAL
//#define DISPLAY_HORIZONTAL
// configuracoes ------------------------------------------------------------

// FLAGS --------------------------------------------------------------------
volatile bool flag_button_pressed = false;
volatile bool flag_display_on     = false;
volatile bool flag_lora_online    = false;
static unsigned long last_reading_time = 0;
static unsigned long oled_on_time      = 0;
// flags --------------------------------------------------------------------

// PROTOTYPES ---------------------------------------------------------------
// isr
void button_setup();
void button_loop();
// dht
void dht_read();
void dht_print();
void dht_setup();
void dht_loop();
// display
void display_on();
void display_off();
void display_readings();
void display_setup();
void display_loop();
// prototypes ---------------------------------------------------------------

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

void dht_print() {
  Serial.printf("\nTemperatura: %.2f C | Umidade: %.2f %%\n", temp_raw, hum_raw);
}

void dht_setup(){
  dht.begin();
}

void dht_loop(){
  if (millis() - last_reading_time >= 2000) {
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
  display.setRotation(3); 
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

// SETUP --------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUND);
  delay(1000);

  Wire.begin(PIN_SDA, PIN_SCL);

  // setup dos periféricos
  display_setup();
  dht_setup();
  button_setup();
}
// setup --------------------------------------------------------------------

// LOOP ---------------------------------------------------------------------
void loop() {
  dht_loop();
  button_loop();
  display_loop();
}
// loop ---------------------------------------------------------------------