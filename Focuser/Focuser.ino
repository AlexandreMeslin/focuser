/*
 * Pinouts for Arduino
 *  0 Serial RX
 *  1 Serial TX
 *  2  
 *  3 Encoder CLK
 *  4 Encoder DT
 *  5 Encoder SW
 *  6 
 *  7 
 *  8 
 *  9 Stepper 0
 * 10 Stepper 1
 * 11 Stepper 2
 * 12 Stepper 3
 * 13
 * A0 
 * A1 
 * A2
 * A3
 * A4
 * A5
 * 
 * Pinouts for NodeMCU ESP8266
 * D3 GPIO0
 * TX GPIO1 Serial TX 
 * D4 GPIO2 Stepper 3
 * RX GPIO3 Serial RX
 * D2 GPIO4 Stepper 1
 * D1 GPIO5 Stepper 0
 * D6 GPIO12 Encoder DT
 * D7 GPIO13 Encoder SW
 * D5 GPIO14 Encoder CLK
 * D8 GPIO15 Stepper 2
 * D0 GPIO16
 */

//#define DEBUG
//#define NCE

/**
 * Uncomment ONE
 */
//#define ___ARDUINO___
#define ___ESP12___

/**
 * Protótipos
 */
void step(int direcao, int nSteps);
void step(int direcao);

// For virtual console
#ifdef NCE
  #define HOST "10.10.15.222"
  #define PORT (8752)
#else
  #define HOST "10.0.0.101"
  #define PORT (8752)
#endif

#ifdef ___ESP12___
  #include <ESP8266WiFi.h>

  // WiFi console
  WiFiClient client;

  // OTA
  #include <ArduinoOTA.h>
  #include <credentials.h>
  
  // WiFi
#ifdef NCE
  const char* ssid = "hsNCE";
#else  
  const char* ssid = MY_SSID;
  const char* password = MY_PASSWORD;
#endif

  #ifdef DEBUG
    #define print(x)    {Serial.print(x);   client.print(x);}
    #define println(x)  {Serial.println(x); client.println(x);}
  #else
    #define print(x)    {client.print(x);}
    #define println(x)  {client.println(x);}
  #endif
#endif

#ifdef ___ARDUINO___
  #define print(x)    {Serial.print(x);}
  #define println(x)  {Serial.println(x);}
#endif

/*
 * Serial baud rate (b/s)
 */
#ifdef ___ARDUINO___
  #define BAUDRATE 9600
#endif
#ifdef ___ESP12___
  #define BAUDRATE 115200
#endif

/**
 * Stepper
 */
const int stepsPerRevolution = 2048;  // the number of steps per revolution
#ifdef ___ARDUINO___
  #define STEPPER0 9
  #define STEPPER1 10
  #define STEPPER2 11
  #define STEPPER3 12
#endif
#ifdef ___ESP12___
  #define STEPPER0 5
  #define STEPPER1 4
  #define STEPPER2 0
  #define STEPPER3 2
#endif
struct {
  int posicao;
  int nPosicoes;
  int mControl[4][4];
} stepper = {
  0,  // start position number
  4,  // number of positions
  {
    {1,0,0,1},
    {0,0,1,1},
    {0,1,1,0},
    {1,1,0,0}
  }
};

/**
 * Rotary Encoder KY-040
 */
#ifdef ___ARDUINO___
  const int pinoCLK = 3;  // CLK
  const int pinoDT = 4;   // DT
  const int pinoSW = 5;   // SW
#endif
#ifdef ___ESP12___
  const int pinoCLK = 14;  // CLK
  const int pinoDT = 12;   // DT
  const int pinoSW = 13;   // SW
#endif
int contadorPos = 0;
int ultPosicao; 
int leituraCLK;
boolean bCW;

void setup() {
  Serial.begin(BAUDRATE);

  // Rotary Encoder
  pinMode(pinoCLK,INPUT);
  pinMode(pinoDT,INPUT);
  pinMode(pinoSW,INPUT_PULLUP);
  ultPosicao = digitalRead(pinoCLK);

  // Stepper motor
  pinMode(STEPPER0, OUTPUT);
  pinMode(STEPPER1, OUTPUT);
  pinMode(STEPPER2, OUTPUT);
  pinMode(STEPPER3, OUTPUT);

#ifdef ___ESP12___
  println("Booting");
  WiFi.mode(WIFI_STA);
#ifdef NCE
  WiFi.begin(ssid);
#else
  WiFi.begin(ssid, password);
#endif
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  println("Connected to WiFi");

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("FOCUSER_OTA");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      println("End Failed");
    }
  });
  ArduinoOTA.begin();
  println("Ready!");
  print("IP address: ");
  println(WiFi.localIP());

  // WiFi console
  if(!client.connect(HOST, PORT)) {
    print("Could not connect to WiFi console at ");
  } else {
    print("Connected to WiFi console at ");  
  }
  print(HOST);
  print(":");
  println(PORT);
  delay(1000);
#endif
} 

void loop() {
#ifdef ___ESP12___
  ArduinoOTA.handle();
#endif

  leituraCLK = digitalRead(pinoCLK);
  int leituraDT = digitalRead(pinoDT);
  if(leituraCLK != ultPosicao) {
    print(ultPosicao);
    print(" ");
    print(leituraCLK);
    print(" ");
    println(leituraDT);
    if(ultPosicao == 1)
    if (leituraDT != leituraCLK) {
      // sentido horário
      println("Sentido horário");
      contadorPos++; 
      bCW = true;
      if(digitalRead(pinoSW) == LOW) {
        step(1, 10);
      } else {
        step(1);      
      }
    } else {
      // sentido anti-horário
      println("Sentido anti-horário");
      bCW = false;
      contadorPos--;
      if(digitalRead(pinoSW) == LOW) {
        step(-1, 10);
      } else {
        step(-1);      
      }
    }
  }
  
  ultPosicao = leituraCLK;
  delay(4);
}

void step(int direcao, int nSteps) {
  for(int i=0; i<nSteps; i++) step(direcao);
}
void step(int direcao) {
  stepper.posicao = (stepper.posicao + direcao) & (stepper.nPosicoes-1);
  print("Posição: "); println(stepper.posicao);
  digitalWrite(STEPPER0, stepper.mControl[stepper.posicao][0]);
  digitalWrite(STEPPER1, stepper.mControl[stepper.posicao][1]);
  digitalWrite(STEPPER2, stepper.mControl[stepper.posicao][2]);
  digitalWrite(STEPPER3, stepper.mControl[stepper.posicao][3]);
  delay(4);
}
