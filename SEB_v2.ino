// Main program
/* Projeto Sistemas Embarcados 
Titulo: Secadora de Filamento / 
Autor: 
Gabriel Ditomaso 
*/

//Bibliotecas
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PID_v1.h>
#include "DHT.h"


//Mapeamento de Hardare

#define DHTPIN 15 
#define DHTTYPE DHT11
#define NTC 25 // pino 25 NTC
#define FanRPM 18  //pino leitura rpm
#define PIN_PWM 19  // pino lampada
#define PIN_FAN 26 // pino Fan
/*#define SCL 21 // Pino para LCD
#define SCI 22   // Pino para LCD
/*#define out1 32 // JOYSTICK PINS
#define out2 33  // JOYSTICK PINS
#define out1 32 // JOYSTICK PINS
#define out2 33  // JOYSTICK PINS
#define out2 33  // JOYSTICK PINS*/

#define Temp_topic "Out_topic_seb_IFSPcat_temp"
#define H_topic "Out_topic_seb_IFSPcat_h"
#define RPM_topic "Out_topic_seb_IFSPcat_rpm"
#define On_topic "In_topic_seb_IFSPcat_botao"
#define SetPoint_topic "In_topic_seb_IFSPcat_SetPoint"
#define Time_topic "In_topic_seb_IFSPcat_tempoligado"




// Setup Wifi =========================================================================
/*
const char wifi[]= "emHollyWEED";
const char senha[]= "1999bundamole";
*/
const char wifi[]= "Lets bora";
const char senha[]= "gabuzin";
// Setup MQTT =========================================================================

const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* ClientID = "MESTRE_AGL_IFSP_ESP32#397191.8+4";
int On = 0; 
String StrDado= "";
String StrDado2= "";


// Dados ===============================================================================

// variaveis 
double input, output,pd_output;
double error;
long timeNow, timeLast,inicio; // Variaveis de controle de tempo
double setpoint = 40;  // Setpoint de temperatura
double kp = 21.2536;       // Ganho proporcional
double ki = 5.0016;       // Ganho integral
double kd = 0.0;        // Ganho derivativo
int tempo,rpm;
int  count;
String dado= "", dado1= "", dado2= "";
float h,t,temp,tempo_cont;
TaskHandle_t Task_01Handle = NULL;
TaskHandle_t Task_02Handle = NULL;
TaskHandle_t Task_03Handle = NULL;
TaskHandle_t Task_04Handle = NULL;
TaskHandle_t Task_05Handle = NULL;
TaskHandle_t Task_06Handle = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;




// Objetos ============================================================================
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
DHT dht(DHTPIN, DHTTYPE);

// define o nome do client ============================================================

WiFiClient espClient;
PubSubClient client(espClient);

// configura wifi ===================================================================== 

void ConectaWifi(){
  WiFi.begin(wifi,senha);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Conectado, endereço de IP: ");
  Serial.println(WiFi.localIP());
}
// configura MQTT ===================================================================== 
 void ConectaMQTT ()
 {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(recebeDado);
  while (!client.connected()) 
  {
    Serial.println("Conectando ao MQTT...");
    if (client.connect(ClientID)) {
      Serial.println("Conexão MQTT bem sucedida");
  //    client.subscribe(In_topic);
      client.subscribe(On_topic);
      client.subscribe(SetPoint_topic);
      client.subscribe(Time_topic);
    } 
    else {
      Serial.print("Falha na conexão MQTT: ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

 void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
   //   client.subscribe(In_topic);
      client.subscribe(On_topic);
      client.subscribe(SetPoint_topic);
      client.subscribe(Time_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
  void recebeDado(char* topic, byte* payload, unsigned int length)
 {

    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");


 // Liga ou desliga ============================================================================================ 
            if(strcmp(topic,On_topic)==0) 
            
               {
                          
                if((char)payload[0] == '1')
                {
                    On= 1;
                    inicio=0;
                    // digitalWrite(PIN_PWM,HIGH);
                 
                    Serial.println("ligado");
                    }
                    
                if((char)payload[0] == '0')
                {
                    On= 0;
                   
                    // digitalWrite(PIN_PWM,LOW);
                    Serial.println("Desligado");
                    } 
               }               
 // Set Point =================================================================================================

  if(strcmp(topic,SetPoint_topic)==0) 
            
               {
                   for (int i = 0; i < length; i++) // for que armazena o vetor dos caracteres recebidos
                    {
                      //  Serial.print((char)payload[i]);
                        StrDado += String((char)payload[i]);// transformando caracteres em string
                        setpoint =StrDado.toInt();// convertendo para valor inteiro
                     
                        if(setpoint> 50) // limite para teste do prototipo
                        {
                          setpoint = 50;
                        i=0;
                        }
                    }  

                 Serial.println("setpoint: ");
                 Serial.println(setpoint); // joga variavel na serial para teste
                 StrDado= "";       //limpa string
                
                } 
 

 // Define Tempo ==============================================================================================
   if(strcmp(topic,Time_topic)==0) 
            
  {
      for (int i = 0; i < length; i++) // for que armazena o vetor dos caracteres recebidos
      {
        //  Serial.print((char)payload[i]);
          StrDado2 += String((char)payload[i]);// transformando caracteres em string
          
        tempo =StrDado2.toInt();// convertendo para valor inteiro  
        //tempo= tempo*60;
                   
      }  
        Serial.println("tempo: ");
        Serial.println(tempo); // joga variavel na serial para teste
        Serial.println(" min");
        StrDado2= "";    //limpa string
  
  } 
      
                    
  }

void countRPM()
{
  count++;
}


void setup() {
 
  Serial.begin(115200); // inicia serial
  pinMode(PIN_PWM, OUTPUT); // Definindo pinos como entrada ou saida
  pinMode(PIN_FAN, OUTPUT);
  pinMode(NTC, INPUT);
  pinMode(FanRPM,INPUT);
  attachInterrupt(digitalPinToInterrupt(FanRPM), countRPM, RISING);// leitura de pulsos emitidos pelo ventilador 
  Serial.println();
  dht.begin(); // Inicia DHT11
  ConectaWifi(); // função que configura Wifi
  ConectaMQTT();// função que configura MQTT
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Limitar a saída do PID entre 0 e 255

   xTaskCreatePinnedToCore(
      Task_PID,          // Função que representa a tarefa
      "Task_01",        // Nome da tarefa
      1000,             // Tamanho da pilha da tarefa
      NULL,             // Parâmetro da tarefa
      1,                // Prioridade da tarefa (1 é mais alta, portanto é uma função prioritaria)
      &Task_01Handle,   // Variável para armazenar o identificador da tarefa
      0);               // Núcleo da CPU para atribuir a tarefa


   xTaskCreatePinnedToCore(
      task_publicaMQTT,          // Função que representa a tarefa
      "Task_02",        // Nome da tarefa
      2000,             // Tamanho da pilha da tarefa
      NULL,             // Parâmetro da tarefa
      2,                // Prioridade da tarefa (1 é mais alta, portanto é uma função secundaria)
      &Task_02Handle,   // Variável para armazenar o identificador da tarefa
      0);               // Núcleo da CPU para atribuir a tarefa

   xTaskCreatePinnedToCore(
      Task_ler_Sensores,          // Função que representa a tarefa
      "Task_03",        // Nome da tarefa
      2000,             // Tamanho da pilha da tarefa
      NULL,             // Parâmetro da tarefa
      1,                // Prioridade da tarefa (1 é mais alta, portanto é uma função prioritaria)
      &Task_03Handle,   // Variável para armazenar o identificador da tarefa
      1);               // Núcleo da CPU para atribuir a tarefa    
   xTaskCreatePinnedToCore(
      S_VENT,          // Função que representa a tarefa
      "Task_04",        // Nome da tarefa
      3000,             // Tamanho da pilha da tarefa
      NULL,             // Parâmetro da tarefa
      2,                // Prioridade da tarefa (1 é mais alta, portanto é uma função secundaria)
      &Task_04Handle,   // Variável para armazenar o identificador da tarefa
      1);               // Núcleo da CPU para atribuir a tarefa  
  
     xTaskCreatePinnedToCore(
      Tempo_Secagem,          // Função que representa a tarefa
      "Task_05",        // Nome da tarefa
      3000,             // Tamanho da pilha da tarefa
      NULL,             // Parâmetro da tarefa
      3,                // Prioridade da tarefa (1 é mais alta, portanto é uma função secundaria)
      &Task_05Handle,   // Variável para armazenar o identificador da tarefa
      1);               // Núcleo da CPU para atribuir a tarefa  


  }
    

void loop() {

  /*    if (millis() - timeLast > 1000) 
        { 
          // atualiza valores a cada 1s
          timeLast = timeNow; // Atualiza tempo
            if (!client.connected()) 
              {
                Serial.println("reconectando com MQTT");
                reconnect();  // chama função que reconecta
              }// fim do if 
        }*/

         
    if (millis() - timeLast >= 1000) { // Intervalo de 1 segundo
        detachInterrupt(digitalPinToInterrupt(FanRPM));
        int rpm = count * 60; // Convertendo pulsos para RPM
        count = 0;
        timeLast = timeNow; // Atualiza tempo
        attachInterrupt(digitalPinToInterrupt(FanRPM), countRPM, RISING);
    }

    client.loop(); //Esta função executaas funções necessarias para conectar com o broker      
    }

   
