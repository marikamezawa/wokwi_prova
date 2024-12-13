#include <WiFi.h>
#include <HTTPClient.h>


#define led_blue 42 // Pino utilizado para controle do led azul
#define led_green 2 // Pino utilizado para controle do led verde
#define led_red 40 // Pino utilizado para controle do led vermelho
#define led_yellow 9 // Pino utilizado para controle do led amarelo

const int buttonPin = 18;  // Pino utilizado para o botão 
int buttonState = 0;  // Lê o estado do botão

const int ldrPin = 4;  // Pino utilizado para o sensor ldr 
int threshold=600;

// Conxexão wifi
  WiFi.begin("Wokwi-GUEST", ""); // Conexão à rede WiFi aberta com SSID Wokwi-GUEST

  while (WiFi.status() != WL_CONNECT_FAILED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi com sucesso!"); // Considerando que saiu do loop acima, o ESP32 agora está conectado ao WiFi (outra opção é colocar este comando dentro do if abaixo)


void setup() {

  // Configuração inicial dos pinos para controle dos leds como OUTPUTs (saídas) do ESP32
    pinMode(led_blue, OUTPUT);
    pinMode(led_green, OUTPUT);
    pinMode(led_red, OUTPUT);
    pinMode(led_yellow, OUTPUT);

    // Configuração inicial dos pinos para controle do botão e do sensor ldr como INPUT (entrada) do ESP32
    pinMode(buttonPin, INPUT); 
    pinMode(ldrPin, INPUT);

    //drPin = 4;  // the number of the pushbutton pi
    //int threshold=600;


  // função para o led amarelo piscar
    void yellow() {
      digitalWrite(led_yellow, HIGH)
      delay(1000);
      digitalWrite(led_yellow, LOW)
      delay(1000);
    }

  // função para os leds funcionar no sistema do semáforo 
    void light() {
       digitalWrite(led_green, HIGH) // led acende
       delay(3000); // delay de 3 segundo
       digitalWrite(led_green, LOW) // led apaga

       digitalWrite(led_yellow, HIGH);
       delay(2000);
       digitalWrite(led_yellow, LOW);

       digitalWrite(led_red, HIGH);
       delay(5000);
       digitalWrite(led_red, LOW);
    }

void loop() {

  // O que irá acontecer no loop
  Serial.begin(9600); // Configuração para debug por interface serial entre ESP e computador com baud rate de 9600
  digitalWrite(led_blue, LOW); //led azul desligado
  digitalWrite(led_green, LOW); //led verde desligado
  digitalWrite(led_red, LOW); //led vermelho desligado
  digitalWrite(led_yellow, LOW); //led amarelo desligado

  int ldrstatus=analogRead(ldrPin); // lendo o estado do ldr


  if(ldrstatus<=threshold){ // Se o sensor ldr detectar baixa luminosidade led amrelo pisca
    Serial.print("its dark turn on led"); // printa que está escuro e que o led acende
    Serial.println(ldrstatus); // printa o estado do sensor ldr
    Yellow(); // chama a função do led amarelo piscando a cada 1 segundo
  }

  }else{ // se não, chamada a função light 
    Serial.print("its bright turn off light");
    Serial.println(ldrstatus);
    light(); // função que tem o sistema de luz do semáforo
  }

}

  int ledstatus = digitalRead(led_red); // lê o estado do led vermelho
  int ldrstatus=analogRead(ldrPin); // lê o estado do sensor ldr

  // Verifica estado do botão
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && ldrstatus>=threshold && ledstatus == 1) {
    Serial.println("Botão pressionado!");
    digitalWrite(led_red, LOW);
    digitalWrite(led_green, HIGH);
    delay(1000);
    digitalWrite(led_red, HIGH);
    digitalWrite(led_green, LOW);

  } else {
    Serial.println("Botão não pressionado!");
  }

  if(WiFi.status() == WL_CONNECTED){ // Se o ESP32 estiver conectado à Internet
    HTTPClient http;

    String serverPath = "http://www.google.com.br/"; // Endpoint da requisição HTTP

    http.begin(serverPath.c_str());

    int httpResponseCode = http.GET(); // Código do Resultado da Requisição HTTP

    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
      }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
      }
      http.end();
    }

  else {
    Serial.println("WiFi Disconnected");
  }
}

