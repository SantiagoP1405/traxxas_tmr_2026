#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 5
#define SCL_PIN 6
#define I2C_SLAVE_ADDR 0x05  // Dirección para el esclavo de LEDs

#define LED_PIN_L 7
#define LED_PIN_R 10
#define BLINK_INTERVAL 30

// Buffer para recibir strings
#define STRING_BUFFER_SIZE 100
volatile char receivedString[STRING_BUFFER_SIZE];
volatile bool newData = false;

void receiveEvent(int howMany);

void setup() {
    // Inicializar Serial PRIMERO
    USBSerial.begin(115200);
    delay(2000);
    
    USBSerial.println("\n=== Iniciando I2C Slave LED (String) ===");

    // 1. Configurar pines ANTES de iniciar
    if (!Wire.setPins(SDA_PIN, SCL_PIN)) {
        USBSerial.println("ERROR: No se pudo configurar los pines I2C");
        while(1) delay(100);
    }
    USBSerial.println("✓ Pines configurados: SDA=5, SCL=6");

    // 2. Iniciar I2C en modo Esclavo
    if (!Wire.begin(I2C_SLAVE_ADDR)) {
        USBSerial.println("ERROR: No se pudo iniciar Wire como esclavo");
        while(1) delay(100);
    }
    USBSerial.print("✓ I2C iniciado como esclavo en dirección 0x");
    USBSerial.println(I2C_SLAVE_ADDR, HEX);

    // 3. Registrar el callback
    Wire.onReceive(receiveEvent);
    USBSerial.println("✓ Callback registrado");
    
    USBSerial.println("=== I2C Slave LED listo para recibir strings ===\n");
    
    // Inicializar buffer
    memset((void*)receivedString, 0, STRING_BUFFER_SIZE);

    // Setup para intermitentes
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_L, OUTPUT);

    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_L, HIGH);
    delay(200);
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_L, LOW);
    delay(200);
    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_L, HIGH);
    delay(700);
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_L, LOW);
}

void loop() {
    // Si hay datos nuevos, imprime el string
    if (newData) {
        USBSerial.print("String LED recibido: ");
        USBSerial.println((char*)receivedString);
        
        // Aquí puedes procesar el string para controlar los LEDs
        // Por ejemplo, parsear comandos como "ON", "OFF", "BLINK", etc.


        char msg = receivedString[0];
        switch(msg){
          case 'R':
            digitalWrite(LED_PIN_R, HIGH);
            digitalWrite(LED_PIN_L, LOW);
            delay(BLINK_INTERVAL);
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(LED_PIN_L, LOW);
            //delay(BLINK_INTERVAL);
            break;
            
          case 'L':
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(LED_PIN_L, HIGH);
            delay(BLINK_INTERVAL);
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(LED_PIN_L, LOW);
            //delay(BLINK_INTERVAL);
            break;

          case 'S':
            digitalWrite(LED_PIN_R, HIGH);
            digitalWrite(LED_PIN_L, HIGH);
            delay(BLINK_INTERVAL);
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(LED_PIN_L, LOW);
            //delay(BLINK_INTERVAL);
            break;

          case 'F':
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(LED_PIN_L, LOW);
            break;
        }
        
        newData = false;
    }
    
    delay(50);
}

// FUNCIÓN DE INTERRUPCIÓN - Recibe string del Master
void receiveEvent(int howMany) {
    int index = 0;
    
    // Leer todos los bytes disponibles y guardarlos como string
    while (Wire.available() && index < STRING_BUFFER_SIZE - 1) {
        receivedString[index++] = Wire.read();
    }
    
    // Asegurar null termination
    receivedString[index] = '\0';
    
    // Señalizar que hay dato nuevo
    newData = true;
    
    // Limpiar cualquier byte restante si el buffer se llenó
    while (Wire.available()) {
        Wire.read();
    }
}
