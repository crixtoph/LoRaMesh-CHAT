//bibliotecas utilizadas
#include <WiFi.h> //Utilizada para establecer al ESP32 en modo AP.
#include <LoraMesher.h> //Facilita la comunicación mesh usando LoRa.
#include <SPIFFS.h>  //Gestiona el sistema de archivos en la memoria flash del ESP32.
#include <AsyncTCP.h> //Proporciona una implementación asíncrona de TCP.
#include <ESPAsyncWebServer.h> // Crea servidores web asíncronos en el ESP32.
#include <ArduinoJson.h> // Maneja la serialización y deserialización de datos JSON.
#include <TinyGPS++.h> // Interactúa con módulos GPS para obtener datos de ubicación.
#include <Crypto.h> // Biblioteca para cifrado
#include <AES.h> // Biblioteca para AES
#include <CTR.h> // Biblioteca para AES CTR
#include <esp_random.h> // Generador de números aleatorios

// Nombre y contraseña de la red WiFi
const char* ssid = "nodo3";
const char* password = "12345678";
// Crear instancia de AsyncWebServer en el puerto 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
// Estructura del paquete de datos (payload)
struct dataPacket {
    char message[200];
    char userName[6];
    uint16_t channelId;
    uint8_t iv[16];
};
// Paquete para enviar datos
dataPacket* helloPacket = new dataPacket;
// Instancia de LoraMesher
LoraMesher &radio = LoraMesher::getInstance();
// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
double latitude = 0.0, longitude = 0.0;
unsigned long lastGPSTime = 0;
// Clave para el cifrado AES
const uint8_t aesKey[16] = {0x5A, 0xB1, 0x23, 0x8F, 0x9C, 0x2E, 0x7D, 0x6F, 0xA4, 0x55, 0x1B, 0x8D, 0xE9, 0x3F, 0x72, 0xC0};

// Declaración de funciones
void createReceiveMessages();
void processReceivedPackets(void*);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void readGPSData();
void aesEncryptDecrypt(char* data, size_t dataLen, const uint8_t* key, const uint8_t* iv, bool encrypt);

// Función para procesar paquetes recibidos
void processReceivedPackets(void*) {
    for (;;) {
        //Espera la notificación de processRecivedPackets y entrar en bloqueo
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        //Iterar a través de todos los paquetes dentro de los paquetes de usuario recibidos FiFo
        while (radio.getReceivedQueueSize() > 0) {
            //Obtener el primer elemento dentro de los paquetes de usuario recibidos FiFo
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();
            // Mostrar el mensaje cifrado recibido en el monitor
            Serial.printf("Mensaje cifrado recibido: %s\n", packet->payload->message);
            // Calcular la longitud real del mensaje
            size_t messageLen = strlen(packet->payload->message);
            // Descifrar el mensaje
            aesEncryptDecrypt(packet->payload->message, messageLen, aesKey, packet->payload->iv, false);
            // Crear el mensaje a enviar a través de WebSocket
            String receivedMessage = "(ID_" + String(packet->src) + ") <b>" + String(packet->payload->userName) + ":</b> " + String(packet->payload->message);
            // Mostrar el mensaje descifrado en el monitor
            Serial.printf("Paquete recibido de %d para %d: Mensaje descifrado: %s \n", packet->src, packet->dst, packet->payload->message);
            
            // Enviar el mensaje recibido a través de WebSocket
            JsonDocument doc; // Crear un documento JSON
            // Verificar si el mensaje es un mensaje de broadcast o unicast
            if (packet->dst == BROADCAST_ADDR) {
                doc["type"] = "community_broadcast"; 
            } else if (packet->dst == radio.getLocalAddress()) {
                doc["type"] = "unicast";
            } 
            // Agregar los datos al documento JSON
            doc["message"] = receivedMessage;
            doc["channelId"] = packet->payload->channelId;
            doc["dst"] = packet->dst;
            // Serializar el documento JSON
            String jsonMessage;
            serializeJson(doc, jsonMessage);
            ws.textAll(jsonMessage);// Enviar el mensaje a todos los clientes conectados a través de WebSocket
            // Eliminar el paquete de la cola para liberar memoria
            radio.deletePacket(packet);
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;// Manejador de la tarea de recepción de mensajes

// Función para crear una tarea para recibir mensajes
void createReceiveMessages() {
    int res = xTaskCreate( 
        processReceivedPackets,
        "Recibir tarea de aplicación",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle
    );
    if (res != pdPASS) {
        Serial.printf("Error: la creación de la tarea de recepción dio error: %d\n", res);
    }
}

// Función para generar un IV aleatorio
void generateRandomIV(uint8_t* iv, size_t ivLen) {
    for (size_t i = 0; i < ivLen; ++i) {
        iv[i] = esp_random() & 0xFF;
    }
}

// Función para cifrar/descifrar un mensaje usando AES128-CTR
void aesEncryptDecrypt(char* data, size_t dataLen, const uint8_t* key, const uint8_t* iv, bool encrypt) {
    CTR<AES128> ctr;
    ctr.setKey(key, sizeof(aesKey));
    ctr.setIV(iv, 16);
    
    if (encrypt) {
        ctr.encrypt((uint8_t*)data, (uint8_t*)data, dataLen);
    } else {
        ctr.decrypt((uint8_t*)data, (uint8_t*)data, dataLen);
    }
}

void setup() {
    Serial.begin(115200);
    // Inicializar SPIFFS
    if (!SPIFFS.begin()) {
        Serial.println("Error inicializando SPIFFS");
        return;
    }
    // Inicializar WiFi como AP
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    // Configurar servidor web para servir el archivo index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    // Configurar endpoint para obtener coordenadas GPS
    server.on("/getGPS", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument jsonDoc;
        jsonDoc["latitude"] = latitude;
        jsonDoc["longitude"] = longitude;
        String jsonString;
        serializeJson(jsonDoc, jsonString);
        request->send(200, "application/json", jsonString);
    });
    // Configurar WebSocket
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    // Iniciar servidor web
    server.begin();
    // Instalar el servicio de interrupción del GPIO
    gpio_install_isr_service(0);
    // Configurar LoRaMesher
    LoraMesher::LoraMesherConfig config;
    config.loraCs = 18;
    config.loraRst = 23;
    config.loraIrq = 26;
    config.loraIo1 = 33;
    config.module = LoraMesher::LoraModules::SX1276_MOD;

    radio.begin(config);
    createReceiveMessages();//Tarea para recibir mensajes
    //Establece el manejador de tarea para LoRaMesher
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
    // Iniciar LoRaMesher
    radio.start();
    // Configurar el GPS Serial
    gpsSerial.begin(9600, SERIAL_8N1, 16);
}

void loop() {
    if (millis() - lastGPSTime > 30000) {
        lastGPSTime = millis();
        readGPSData();
    }
}

// Manejar mensajes de WebSocket
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg; // Obtener información del mensaje
    // Verificar si el mensaje es un mensaje de texto y si es el último fragmento
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;// Agregar un carácter nulo al final del mensaje
        String msg = String((char*)data);// Convertir el mensaje a String
        // Mostrar el mensaje en el monitor serial
        Serial.printf("WebSocket message: %s\n", msg.c_str());

        JsonDocument doc;// Crear un documento JSON
        DeserializationError error = deserializeJson(doc, msg);// Deserializar el mensaje JSON
        // Verificar si hubo un error al deserializar el mensaje
        if (error) {
            Serial.println("deserializeJson() failed");
            return;
        }         
        const char* type = doc["type"];// Obtener el tipo de mensaje
        const char* message = doc["message"];// Obtener el mensaje
        const char* userName = doc["userName"];// Obtener el nombre de usuario
        uint16_t nodeId = doc["nodeId"];// Obtener el ID del nodo
        uint16_t channelId = doc["channelId"];// Obtener el ID del canal
        uint16_t dst = doc["dst"];// Obtener el ID del destino

        size_t messageLen = strlen(message);// Calcular la longitud del mensaje
        strncpy(helloPacket->message, message, messageLen);// Copiar el mensaje al paquete de datos
        helloPacket->message[messageLen] = '\0';// Agregar un carácter nulo al final del mensaje
        strncpy(helloPacket->userName, userName, sizeof(helloPacket->userName) - 1);// Copiar el nombre de usuario al paquete de datos
        helloPacket->userName[sizeof(helloPacket->userName) - 1] = '\0';// Agregar un carácter nulo al final del nombre de usuario
        helloPacket->channelId = channelId;// Agregar el ID del canal al paquete de datos

        generateRandomIV(helloPacket->iv, sizeof(helloPacket->iv));// Generar un IV aleatorio para el cifrado
        aesEncryptDecrypt(helloPacket->message, messageLen, aesKey, helloPacket->iv, true);// Cifrar el mensaje
        Serial.printf("Mensaje cifrado enviado: %s\n", helloPacket->message);// Mostrar el mensaje cifrado en el monitor serial
        // Verificar si el tipo de mensaje es un mensaje de broadcast o unicast
        if (strcmp(type, "broadcast") == 0) {
            radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);// Enviar el mensaje a través de LoRa
        } else if (strcmp(type, "unicast") == 0) {
            int dataTablePosition = 0;
            if (radio.routingTableSize() == 0) {// Verificar si la tabla de enrutamiento está vacía
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                return;
            }
            // Verificar si el destino se encuentra en la tabla de enrutamiento
            bool found = false;
            LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();
            for (int i = 0; i < radio.routingTableSize(); i++) {
                uint16_t addr = (*routingTableList)[i]->networkNode.address;
                Serial.printf("ID DE TABLA ENRUTAMIENTO: %d\n", addr);
                if (addr == dst) {
                    found = true;
                    break;
                }
            }
            // Enviar el mensaje al destino si se encuentra en la tabla de enrutamiento
            if (found) {
                radio.createPacketAndSend(dst, helloPacket, 1);// Enviar el mensaje a través de LoRa
            } else {
                Serial.println("El destino no se encuentra en la tabla de enrutamiento");
                Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------");
            }
        }
    }
}

// Evento de WebSocket
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {// Verificar el tipo de evento
        case WS_EVT_CONNECT:// Evento de conexión
            Serial.printf("WebSocket client %u connected\n", client->id());// Mostrar el ID del cliente conectado
            {
                JsonDocument doc;// Crear un documento JSON
                doc["type"] = "nodeId";
                doc["nodeId"] = radio.getLocalAddress();// Agregar el ID del nodo al documento JSON
                String jsonMessage;// Convertir el documento JSON a String
                serializeJson(doc, jsonMessage);// Serializar el documento JSON
                client->text(jsonMessage);// Enviar el mensaje al cliente conectado
            }
            break;
        case WS_EVT_DISCONNECT:// Evento de desconexión
            Serial.printf("WebSocket client %u disconnected\n", client->id());// Mostrar el ID del cliente desconectado
            break;
        case WS_EVT_DATA:// Evento de datos
            handleWebSocketMessage(arg, data, len);// Manejar el mensaje de WebSocket
            break;
        case WS_EVT_PONG:// Evento de PONG
        case WS_EVT_ERROR:// Evento de error
            break;
    }
}

// Función para leer datos del GPS
void readGPSData() {
    while (gpsSerial.available() > 0) {// Verificar si hay datos disponibles en el GPS Serial
        gps.encode(gpsSerial.read());// Procesar los datos del GPS
    }

    if (gps.location.isUpdated()) {// Verificar si la ubicación del GPS ha sido actualizada
        latitude = gps.location.lat();// Obtener la latitud
        longitude = gps.location.lng();// Obtener la longitud
    }
}



