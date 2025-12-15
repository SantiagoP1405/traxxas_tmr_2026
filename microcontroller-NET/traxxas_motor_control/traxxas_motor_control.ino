#include <Arduino.h>
#include <Wire.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define SDA_PIN 5
#define SCL_PIN 6
#define I2C_SLAVE_ADDR 0x04

// Identificadores para los datos
#define ID_DIRECTION 0x01
#define ID_THROTTLE 0x02

// Pines PWM - Usando pines seguros para ESP32-C3
#define LED_PIN_DIR 0      // GPIO0 para dirección
#define LED_PIN_THROT 1    // GPIO1 para throttle

// Configuración LEDC
#define LEDC_TIMER_DIR          LEDC_TIMER_0
#define LEDC_TIMER_THROT        LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_DIR        LEDC_CHANNEL_0
#define LEDC_CHANNEL_THROT      LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT  // 14 bits de resolución
#define LEDC_FREQUENCY          100                 // 100 Hz
#define LEDC_MAX_DUTY           16383              // 2^14 - 1

// Variables volátiles para la interrupción
volatile uint16_t directionValue = 0;
volatile uint16_t throttleValue = 0;
volatile bool newDirectionData = false;
volatile bool newThrottleData = false;

void receiveEvent(int howMany);
void setupPWM();
void updatePWM(ledc_channel_t channel, uint16_t value);  // CAMBIADO: tipo correcto

void setup() {
    // Inicializar pines como salida
    pinMode(LED_PIN_THROT, OUTPUT);
    pinMode(LED_PIN_DIR, OUTPUT);
    
    // Inicializar Serial
    USBSerial.begin(115200);
    delay(2000);
    
    USBSerial.println("\n=== Iniciando I2C Slave con PWM 14-bit @ 100Hz ===");
    
    // Configurar PWM usando LEDC nativo
    setupPWM();
    USBSerial.println("✓ PWM configurado: 14-bit @ 100Hz");
    
    // Configurar pines I2C
    if (!Wire.setPins(SDA_PIN, SCL_PIN)) {
        USBSerial.println("ERROR: No se pudo configurar los pines I2C");
        while(1) delay(100);
    }
    USBSerial.println("✓ Pines configurados: SDA=5, SCL=6");

    // Iniciar I2C en modo Esclavo
    if (!Wire.begin(I2C_SLAVE_ADDR)) {
        USBSerial.println("ERROR: No se pudo iniciar Wire como esclavo");
        while(1) delay(100);
    }
    USBSerial.print("✓ I2C iniciado como esclavo en dirección 0x");
    USBSerial.println(I2C_SLAVE_ADDR, HEX);

    // Registrar el callback
    Wire.onReceive(receiveEvent);
    USBSerial.println("✓ Callback registrado");
    USBSerial.println("✓ PWM en GPIO0 (Dir) y GPIO1 (Throttle)");  // CORREGIDO
    USBSerial.println("=== Sistema listo ===");
    USBSerial.println("Rango PWM: 0-16383 (14 bits)\n");
    
    // Inicializar PWM con valor 0
    updatePWM(LEDC_CHANNEL_DIR, 0);
    updatePWM(LEDC_CHANNEL_THROT, 0);
}

void setupPWM() {
    // Configurar timer para Direction
    ledc_timer_config_t ledc_timer_dir = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_DIR,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_dir));
    
    // Configurar timer para Throttle
    ledc_timer_config_t ledc_timer_throt = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_THROT,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_throt));
    
    // Configurar canal para Direction
    ledc_channel_config_t ledc_channel_dir = {
        .gpio_num       = LED_PIN_DIR,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_DIR,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_DIR,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_dir));
    
    // Configurar canal para Throttle
    ledc_channel_config_t ledc_channel_throt = {
        .gpio_num       = LED_PIN_THROT,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_THROT,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_THROT,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_throt));
}

void updatePWM(ledc_channel_t channel, uint16_t value) {  // CAMBIADO: tipo correcto
    // Limitar el valor al máximo de 14 bits
    if (value > LEDC_MAX_DUTY) {
        value = LEDC_MAX_DUTY;
    }
    
    // Actualizar el duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}

void loop() {
    // Si hay datos nuevos de dirección
    if (newDirectionData) {
        USBSerial.print("DIRECTION: ");
        USBSerial.print(directionValue);
        USBSerial.print("/16383 (");
        USBSerial.print((directionValue * 100) / 3276);
        USBSerial.println("%)");
        
        // Actualizar PWM de dirección con el valor recibido
        updatePWM(LEDC_CHANNEL_DIR, directionValue);
        newDirectionData = false;
    }
    
    // Si hay datos nuevos de throttle
    if (newThrottleData) {
        USBSerial.print("THROTTLE: ");
        USBSerial.print(throttleValue);
        USBSerial.print("/16383 (");
        USBSerial.print((throttleValue * 100) / 3276);
        USBSerial.println("%)");
        
        // Actualizar PWM de throttle con el valor recibido
        updatePWM(LEDC_CHANNEL_THROT, throttleValue);
        newThrottleData = false;
    }
    
    delay(50);
}

// FUNCIÓN DE INTERRUPCIÓN - Recibe datos del Master
void receiveEvent(int howMany) {
    // Esperamos 3 bytes: 1 ID + 2 bytes de datos
    if (howMany == 3) {
        // Leer el identificador
        uint8_t id = Wire.read();
        
        // Leer los dos bytes del valor
        byte highByte = Wire.read();    // Bits 15-8
        byte lowByte = Wire.read();     // Bits 7-0
        
        // Reconstruir el valor de 16 bits
        uint16_t receivedValue = (highByte << 8) | lowByte;
        
        // Limitar al rango de 14 bits
        if (receivedValue > LEDC_MAX_DUTY) {
            receivedValue = LEDC_MAX_DUTY;
            // No usar USBSerial en la interrupción para evitar problemas
        }
        
        // Guardar según el identificador
        if (id == ID_DIRECTION) {
            directionValue = receivedValue;
            newDirectionData = true;
        } 
        else if (id == ID_THROTTLE) {
            throttleValue = receivedValue;
            newThrottleData = true;
        }
    }
    else {
        // Limpiar el buffer
        while (Wire.available()) {
            Wire.read();
        }
    }
}
