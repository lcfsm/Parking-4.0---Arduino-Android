#include "Ultrasonic.h"
#include <MFRC522.h>
#include <SPI.h>

long previousMillis = 0;        // will store last time LED was updated
long intervalOn = 3000;           
int distancia;
String plaza1ocupada="0";
String estadoalarma="0";

long previousMillis2 = 0;        // will store last time LED was updated
long intervalOn2 = 10000; 

long previousMillis3 = 0;
long intervalOn3 = 1000;  

char thingSpeakAddress[] = "api.thingspeak.com";
String APIKey = "F9JDP64BPF93WAI4";
unsigned long myChannelNumber = 115835;

//#define SSID        "SNAIL2.4"
//#define PASS        "+Nu@s<9Bgj,D?N_yEg>"
//#define SSID        "MCU"
//#define PASS        "12345678" 
#define SSID        "MotoG3"
#define PASS        ""




//#define DEST_HOST   "json.internetdelascosas.es"
//#define DEST_IP     "212.227.247.230"

#define DEST_HOST   "api.thingspeak.com"
#define DEST_IP     "52.21.59.111"

#define TIMEOUT     5000 // mS
#define CONTINUE    false
#define HALT        true

// #define ECHO_COMMANDS // Un-comment to echo AT+ commands to serial monitor
int LED = 13;
boolean LEDst = false;

//always high
int CH_PD_8266 = 38;
// Print error message and loop stop.
void errorHalt(String msg)
{
  Serial.println(msg);
  Serial.println("HALT");
  while(true){};
}

boolean sepuedeenviar=false;

// Read characters from WiFi module and echo to serial until keyword occurs or timeout.
boolean echoFind(String keyword)
{
  byte current_char   = 0;
  byte keyword_length = keyword.length();
  
  // Fail if the target string has not been sent by deadline.
  long deadline = millis() + TIMEOUT;
  while(millis() < deadline)
  {
    if (Serial3.available())
    {
      char ch = Serial3.read();
      Serial.write(ch);
      if (ch == keyword[current_char])
        if (++current_char == keyword_length)
        {
          Serial.println();
          return true;
        }
    }
  }
  return false;  // Timed out
}

// Read and echo all available module output.
// (Used when we're indifferent to "OK" vs. "no change" responses or to get around firmware bugs.)
void echoFlush()
  {while(Serial3.available()) Serial.write(Serial3.read());}
  
// Echo module output until 3 newlines encountered.
// (Used when we're indifferent to "OK" vs. "no change" responses.)
void echoSkip()
{
  echoFind("\n");        // Search for nl at end of command echo
  echoFind("\n");        // Search for 2nd nl at end of response.
  echoFind("\n");        // Search for 3rd nl at end of blank line.
}

// Send a command to the module and wait for acknowledgement string
// (or flush module output if no ack specified).
// Echoes all data received to the serial monitor.
boolean echoCommand(String cmd, String ack, boolean halt_on_fail)
{
  Serial.println("Valor ack:"+ack);
  Serial3.println(cmd);
  #ifdef ECHO_COMMANDS
    Serial.print("--"); 
    Serial.println(cmd);
  #endif
  
  // If no ack response specified, skip all available module output.
  if (ack == "")
    echoSkip();
  else
    // Otherwise wait for ack.
    if (!echoFind(ack))          // timed out waiting for ack string 
      if (halt_on_fail)
        errorHalt(cmd+" failed");// Critical failure halt.
      else
        return false;            // Let the caller handle it.
  return true;                   // ack blank or ack found
}

// Connect to the specified wireless network.
boolean connectWiFi()
{
  String cmd = "AT+CWJAP=\""; cmd += SSID; cmd += "\",\""; cmd += PASS; cmd += "\"";
  if (echoCommand(cmd, "OK", CONTINUE)) // Join Access Point
  {
    Serial.println("Connected to WiFi.");
    return true;
  }
  else
  {
    Serial.println("Connection to WiFi failed.");
    return false;
  }
}

// FIN WIFI


String value1; // Variable que almacena las plazas disponibles
int numplazas=3; // Variable que inicializa las plazas del parking
int valormodificado=numplazas;

// Temporizador luces parking
boolean encenderlucesparking=false;
int temporizadorluces=0;

/*
Pins  SPI    UNO  Mega2560  Leonardo
1 (NSS) SAD (SS)   10     53        10
2       SCK        13     52        SCK1
3       MOSI       11     51        MOSI1
4       MISO       12     50        MISO1
5       IRQ        *      *         *
6       GND       GND     GND       GND
7       RST        5      ?         Reset
8      +3.3V (VCC) 3V3     3V3       3.3V
* No necesario
*/
// Valor para comprobar si el ventilador debe estar en marcha
boolean ventiladoronoff=false;
boolean servosalidalevantado=false;
unsigned long  contadorsalida=0;
boolean servoentradalevantado=false;
unsigned long  contadorentrada=0;


// DETECTOR GAS
const int gasPin = A7; //GAS sensor output pin to Arduino analog A0 pin
int valorgas;
boolean alarmaactivada=false;

// LED RGB
const int RED_PIN = 46;
const int GREEN_PIN = 47;
const int BLUE_PIN = 44;
int DISPLAY_TIME = 10;  // In milliseconds


// Barrera ultrasonidos
#define trigPin 43
#define echoPin 42
int dis = 5; // Establecemos la distancia en cm para la comprobacion
// Declaramos el sensor ultrasonido en los pines digitales elegidos
Ultrasonic ultrasonido(trigPin,echoPin); 



//altavoz
int speakerPin = 37;

// RFID
#define SAD 53
#define RST 5
MFRC522 nfc(SAD, RST);
#define ledPinAbierto    2
#define ledPinCerrado  3

// Libreria para el LCD
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
//Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity
LiquidCrystal_I2C lcd( 0x27, 2,   1,  0,  4,  5,  6,  7,           3, POSITIVE );
unsigned long tAntes = 0;
unsigned long tAhora = 0;
unsigned long tEjecucion = 1000;
// Fin LCD

// SERVO
#include <Servo.h>
Servo myservoentrada;  // create servo object to control a servo
Servo myservosalida;
int posentrada = 90;    // variable to store the servo position
int possalida = 6; 

// SENSOR INFRAROJOS
#define IR 26  
int detection = HIGH;    // no obstacle

// DETECTOR DE MOVIMIENTO
const int PIRPin= 28;
int calibrationTime = 10;   


////////////////////// ***** SETUP ****** //////////////////////////////////////////////////

void setup() {
Serial.begin(115200);         // Communication with PC monitor via USB
Serial3.begin(115200);        // Communication with ESP8266 via 5V/3.3V level shifter
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LEDst);
  pinMode(CH_PD_8266, OUTPUT);
  digitalWrite(CH_PD_8266, HIGH);

  Serial3.setTimeout(TIMEOUT);
  Serial.println("ESP8266 Connection");

  delay(2000);

  echoCommand("AT+RST", "ready", HALT);    // Reset & test if the module is ready  
  Serial.println("Module is ready.");
  echoCommand("AT+GMR", "OK", CONTINUE);   // Retrieves the firmware ID (version number) of the module. 
  echoCommand("AT+CWMODE?","OK", CONTINUE);// Get module access mode. 
  
  
  echoCommand("AT+CWMODE=1", "", HALT);    // Station mode
  echoCommand("AT+CIPMUX=1", "", HALT);    // Allow multiple connections (we'll only use the first).

  //connect to the wifi
  boolean connection_established = false;
  for(int i=0;i<100;i++)
  {
    if(connectWiFi())
    {
      connection_established = true;
      break;
    }
  }
  if (!connection_established) errorHalt("Connection failed");
  
  delay(2000);

  echoCommand("AT+CWSAP=?", "OK", CONTINUE); // Test connection
  echoCommand("AT+CIFSR", "", HALT);         // Echo IP address. (Firmware bug - should return "OK".)
  //echoCommand("AT+CIPMUX=0", "", HALT);      // Set single connection mode
  

enviardatos();

// Pin del sensor de infrarojos
pinMode(IR, INPUT);
//Dar el sensor de un cierto tiempo para calibrar
  //Serial.print("Calibrando sensores ");
    for(int i = 0; i < calibrationTime; i++){
      //Serial.print(".");
      delay(500);
      }
    //Serial.println("SENSORES ACTIVADOS");
    //delay(50); 

// Pin del sensor de movimiento
 pinMode(PIRPin, INPUT);

// Pin del ventilador
 pinMode(36,OUTPUT);

// Here we'll configure the Arduino pins we're using to drive the LED to be outputs:
 pinMode(RED_PIN, OUTPUT);
 pinMode(GREEN_PIN, OUTPUT);
 pinMode(BLUE_PIN, OUTPUT);

//  Sensor ultrasonidos
  pinMode(trigPin, OUTPUT); // Establecemos el pin trig como salida
  pinMode(echoPin, INPUT); // Establecemos el pin echo como entrada

// Asignación de las salidas digitales para el led de 1 digito
  pinMode(23, OUTPUT);  
  pinMode(22, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);

// Mostrar mensaje LCD
  lcd.begin(16,2);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("PARKING");  
  lcd.setCursor(0,1);
  lcd.print("DEUSTO"); 
      
   
display (1,1,1,1,0,0,1); // Escribe Nº3 en el display de 1 digito   


myservoentrada.attach(8);  
myservosalida.attach(9); 
myservoentrada.write(posentrada);
myservosalida.write(possalida);
  
pinMode(ledPinAbierto  , OUTPUT);   
pinMode(ledPinCerrado, OUTPUT);   
SPI.begin();

  //Serial.println("Buscando RC522");
  nfc.begin();
  byte version = nfc.getFirmwareVersion();
  if (! version) {//Entra si no encuentra el módulo.
    //Serial.print("No ha encontrado RC522");
    while(1); //detener
  }
  //Serial.print("Ha encontrado RC522");
  //Serial.print("Firmware version 0x");
  //Serial.print(version, HEX);
  //Serial.println(".");

pinMode(41, OUTPUT);       // LED ROJO
pinMode(40, OUTPUT);       // LED VERDE
  
}

#define AUTHORIZED_COUNT 2 //Para autoriazar más tarjetas ponemos el número aqui y la añadimos abajo
byte Authorized[AUTHORIZED_COUNT][6] = {
                          {0x9B, 0x1E, 0x55, 0xCC, 0x1C,}
                          ,{0xD6, 0x9F, 0x83, 0x32, 0xF8,}
                          };                                  
void printSerial(byte *serial);
boolean isSame(byte *key, byte *serial);
boolean isAuthorized(byte *serial);

//////////////////  VOID LOOP  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  unsigned long currentMillis = millis();
    if(currentMillis - previousMillis > intervalOn) {
            previousMillis = currentMillis; 

// DETECTOR HUMO
            //Serial.println(analogRead(gasPin));
            valorgas= map (analogRead(gasPin), 150, 600, 0, 10);
            //Serial.println(valorgas);
            if(valorgas>4){
              alarmaactivada=true;
              alarma();
              //Serial.print("Ventilador=true ");
              ventiladoronoff=true;
              estadoalarma="1";
            }else{
              alarmaactivada=false;
              //Serial.print("Ventilador=false ");
              ventiladoronoff==false;
              digitalWrite(36,LOW);
              estadoalarma="0";
            } 

// Para enviar cuando se dispare la alarma
unsigned long currentMillis3 = millis();
if(currentMillis3 - previousMillis3> intervalOn3) {
            previousMillis3 = currentMillis3; 
            if(estadoalarma=="1"){
              Serial.println("La alarma se ha disparado");
                 //if(sepuedeenviar==true){
                  enviardatos();
                  estadoalarma="0";
                 //} 
            }
}

// Sensor para comprobar si la plaza esta ocupada ULTRASONIC
            distancia = ultrasonido.Ranging(CM);
            if (distancia < dis){
               plaza1ocupada="1";
               digitalWrite(40, LOW);   // LED VERDE
               digitalWrite(41, HIGH);   // LED ROJO
               //Serial.print("Led roja - Distancia: ");
               //Serial.print(ultrasonido.Ranging(CM)); 
               //Serial.println(" cm");
            } else {
                plaza1ocupada="0";
                digitalWrite(40, HIGH);   // LED VERDE
                digitalWrite(41, LOW);   // LED ROJO
                //Serial.print("Led verde - Distancia: ");
                //Serial.print(ultrasonido.Ranging(CM)); 
                //Serial.println(" cm");
            }

                 
               
    }

unsigned long currentMillis2 = millis();
if(currentMillis2 - previousMillis2 > intervalOn2) {
            previousMillis2 = currentMillis2; 
            if(valormodificado!=numplazas){
                 if(sepuedeenviar==true){
                  enviardatos();
                 }
                    
                    valormodificado=numplazas;
            }
}
  
  

// SENSOR MOVIMIENTO (Encender luces parking solo si la alarma esta desactivada)
if(alarmaactivada==false){
int value= digitalRead(PIRPin);
  if (value == HIGH){
    //Serial.print("There is movement!\n");
    encenderlucesparking=true;
  }
  if (encenderlucesparking==true){
    temporizadorluces++;
    if(temporizadorluces==1){
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(BLUE_PIN, HIGH);
    }
    if(temporizadorluces==500){
      encenderlucesparking=false;
      temporizadorluces=0;
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW); 
    }
  }
}   

// SENSOR INFRAROJO - SERVO DE SALIDA
detection = digitalRead(IR);
if(numplazas>=0 && numplazas<=2){
  if(detection == LOW){
    //Serial.print("Detectado coche intentando salir!\n");
    servosalidalevantado=true;
  }
}


if(servosalidalevantado==true){
  contadorsalida++;
    if(contadorsalida==1){
      myservosalida.write(90);
    }
    if(contadorsalida==150){
      numplazas=numplazas+1;
      contadorsalida=0;
      myservosalida.write(6);
      servosalidalevantado=false;
      //Serial.print("El numero de plazas incrementado es");Serial.println(numplazas);
      plazas();
      // Mostrar mensaje LCD
      lcd.begin(16,2);
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("PARKING");  
      lcd.setCursor(0,1);
      lcd.print("DEUSTO");
      sepuedeenviar=true;
    }
}

   


/*if(ventiladoronoff==true){
  // Obtenemos el momento actual en ms
    //unsigned long currentMillis = millis();
    // Si ha pasado el tiempo establecido, ejecutamos la acción
    //if(currentMillis - previousMillis >= timeToAction) {
  //digitalWrite(36,HIGH); // se enciende el ventilador
        //previousMillis = currentMillis; 
    //} 

    //digitalWrite(36,HIGH); // se enciende el ventilador
}else{
  //digitalWrite(36,LOW); // se apaga el ventilador
}*/
 


// Lectura RFID
byte status;
  byte data[MAX_LEN];
  byte serial[5];
  boolean Abierto = false;
  digitalWrite(ledPinAbierto, Abierto);
  digitalWrite(ledPinCerrado, !Abierto);
  status = nfc.requestTag(MF1_REQIDL, data);
if(numplazas<=3 && numplazas>=1){
  if (status == MI_OK) {
    status = nfc.antiCollision(data);
    memcpy(serial, data, 5);
    if(isAuthorized(serial))
    { 
      //Serial.println("Autorizado");
      //printSerial(serial);
      //Abierto = true;
      servoentradalevantado=true;
    }
    else
    { 
      //printSerial(serial);
      //Serial.println("NO autorizado");
      /*Abierto = false;
      // Mostrar mensaje LCD
      lcd.begin(16,2);
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("No");
      lcd.setCursor(0,1);
      lcd.print("autorizado");
      delay(3000);
      // Mostrar mensaje LCD
      lcd.begin(16,2);
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("PARKING");  
      lcd.setCursor(0,1);
      lcd.print("DEUSTO");*/
      
    }
    
    nfc.haltTag();
    digitalWrite(ledPinAbierto, Abierto);
    digitalWrite(ledPinCerrado, !Abierto);
    //delay(2000);
  }
delay(20);
}

if(servoentradalevantado==true){
  contadorentrada++;
  if(contadorentrada==1){
    // Mostrar mensaje LCD
      lcd.begin(16,2);
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("Autorizado");
      myservoentrada.write(160);
  } 
  if(contadorentrada==150){
      contadorentrada=0;
      myservoentrada.write(90);
      servoentradalevantado=false;
      numplazas=numplazas-1;
      plazas();
      // Mostrar mensaje LCD
      lcd.begin(16,2);
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("PARKING");  
      lcd.setCursor(0,1);
      lcd.print("DEUSTO");
      sepuedeenviar=true;
  }
}




}// FIN void loop()

boolean isSame(byte *key, byte *serial)
{
    for (int i = 0; i < 4; i++) {
      if (key[i] != serial[i])
      { 
        return false; 
      }
    }
    
    return true;

}

boolean isAuthorized(byte *serial)
{
    for(int i = 0; i<AUTHORIZED_COUNT; i++)
    {
      if(isSame(serial, Authorized[i]))
        return true;
    }
   return false;
}

void printSerial(byte *serial)
{
        Serial.print("Serial:");
    for (int i = 0; i < 5; i++) {// aumentar a 5 para leer el número de la tarjeta completo
      Serial.print(serial[i], HEX);
      Serial.print(" ");
    }
}


// Funcion del display de 1 digito
void display (int a, int b, int c, int d, int e, int f, int g){
  digitalWrite (23,a);   //Se reciben 7 variables y se asignan
  digitalWrite (22,b);   //a cada una de las salidas
  digitalWrite (34,c);
  digitalWrite (33,d);
  digitalWrite (32,e);
  digitalWrite (24,f);
  digitalWrite (25,g);
}

//Funcion luces parking
void luzparking(){
digitalWrite(RED_PIN, HIGH); // White (turn all the LEDs on):
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, HIGH);
}


void plazas(){
  if(numplazas==0){
   display (1,1,1,1,1,1,0); //escribe 0
  }
  if(numplazas==1){
    display (0,1,1,0,0,0,0); //escribe 1
  }
  if(numplazas==2){
    display (1,1,0,1,1,0,1); //escribe 2
  }
  if(numplazas==3){
    display (1,1,1,1,0,0,1); //escribe 3
  }
}

void alarma(){
 /* if (ventiladoronoff==true){
 Serial.print("entra en el vent true ");
 digitalWrite(36,HIGH); // se enciende el ventilador
}

if (ventiladoronoff==false){
  Serial.print("entra en el vent false ");
  digitalWrite(36,LOW); // se apaga el ventilador
}*/
digitalWrite(36,HIGH);
tone(speakerPin, 294);
// Red (turn just the red LED on):
digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);
delay(1500);
noTone(speakerPin);
// Off (all LEDs off):
digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);
}




// ******** ENVIAR DATOS ********
void enviardatos() {
sepuedeenviar=false;
  
if(numplazas==0){
  value1="0";
}
if(numplazas==1){
  value1="1";
}
if(numplazas==2){
  value1="2";
}if(numplazas==3){
  value1="3";
}

String chain="GET /update?api_key=F9JDP64BPF93WAI4&field1="+value1+"&field2="+plaza1ocupada+"&field3="+estadoalarma+" HTTP/1.1\r\n";

// Establish TCP connection
  String cmd = "AT+CIPSTART=0,\"TCP\",\""; 
  cmd += DEST_IP; 
  cmd += "\",80";


 if (!echoCommand(cmd, "OK", CONTINUE)){
  return;
 }
  delay(3000);
  
  if (!echoCommand("AT+CIPSTATUS", "OK", CONTINUE)) { // Get connection status 
    return;
  }

cmd=chain;
cmd += "Host: api.thingspeak.com\r\n\r\n";
  
  // Ready the module to receive raw data
  if (!echoCommand("AT+CIPSEND=0,"+String(cmd.length()), ">", CONTINUE))
  {
    echoCommand("AT+CIPCLOSE", "", CONTINUE);
    Serial.println("Connection timeout.");
    enviardatosredundancia();
    return;
  }
  // Send the raw HTTP request
  echoCommand(cmd, "OK", CONTINUE);  // GET 


   // Loop forever echoing data received from destination server.
  //while(true)
    while (Serial1.available())
      Serial.write(Serial1.read());
      
  //errorHalt("ONCE ONLY");
}


void enviardatosredundancia(){
  Serial3.begin(115200);        // Communication with ESP8266 via 5V/3.3V level shifter
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LEDst);
  pinMode(CH_PD_8266, OUTPUT);
  digitalWrite(CH_PD_8266, HIGH);

  Serial3.setTimeout(TIMEOUT);
  Serial.println("ESP8266 Modo redundancia");

  //delay(2000);
echoCommand("AT+RST", "ready", HALT);    // Reset & test if the module is ready  
  Serial.println("Module is ready / REDUNDANCY.");
  echoCommand("AT+GMR", "OK", CONTINUE);   // Retrieves the firmware ID (version number) of the module. 
  echoCommand("AT+CWMODE?","OK", CONTINUE);// Get module access mode. 
  
  
  echoCommand("AT+CWMODE=1", "", HALT);    // Station mode
  echoCommand("AT+CIPMUX=1", "", HALT);    // Allow multiple connections (we'll only use the first).

  //connect to the wifi
  boolean connection_established = false;
  for(int i=0;i<100;i++)
  {
    if(connectWiFi())
    {
      connection_established = true;
      break;
    }
  }
  if (!connection_established) errorHalt("Connection failed 2");
  
  //delay(2000);

  echoCommand("AT+CWSAP=?", "OK", CONTINUE); // Test connection
  echoCommand("AT+CIFSR", "", HALT);         // Echo IP address. (Firmware bug - should return "OK".)
  //echoCommand("AT+CIPMUX=0", "", HALT);      // Set single connection mode

  
  if(numplazas==0){
  value1="0";
}
if(numplazas==1){
  value1="1";
}
if(numplazas==2){
  value1="2";
}if(numplazas==3){
  value1="3";
}

String chain="GET /update?api_key=F9JDP64BPF93WAI4&field1="+value1+"&field2="+plaza1ocupada+"&field3="+estadoalarma+" HTTP/1.1\r\n";


// Establish TCP connection
  String cmd = "AT+CIPSTART=0,\"TCP\",\""; 
  cmd += DEST_IP; 
  cmd += "\",80";
  
  if (!echoCommand(cmd, "OK", CONTINUE)) return;
  //delay(2000);
  // Get connection status 
  if (!echoCommand("AT+CIPSTATUS", "OK", CONTINUE)) return;



cmd=chain;
cmd += "Host: api.thingspeak.com\r\n\r\n";
  
  // Ready the module to receive raw data
  if (!echoCommand("AT+CIPSEND=0,"+String(cmd.length()), ">", CONTINUE))
  {
    echoCommand("AT+CIPCLOSE", "", CONTINUE);
    Serial.println("Connection timeout.");
    enviardatosredundancia2();
    return;
  }
  // Send the raw HTTP request
  echoCommand(cmd, "OK", CONTINUE);  // GET
}

///////////////// CAMBIAR COLORES LUCES PARKING ////////////////////////////////

/*
// Off (all LEDs off):
digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Red (turn just the red LED on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Green (turn just the green LED on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Blue (turn just the blue LED on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// Yellow (turn red and green on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Cyan (turn green and blue on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// Purple (turn red and blue on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// White (turn all the LEDs on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);*/


void enviardatosredundancia2(){
  Serial3.begin(115200);        // Communication with ESP8266 via 5V/3.3V level shifter
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LEDst);
  pinMode(CH_PD_8266, OUTPUT);
  digitalWrite(CH_PD_8266, HIGH);

  Serial3.setTimeout(TIMEOUT);
  Serial.println("ESP8266 Modo redundancia");

  //delay(2000);
echoCommand("AT+RST", "ready", HALT);    // Reset & test if the module is ready  
  Serial.println("Module is ready / REDUNDANCY.");
  echoCommand("AT+GMR", "OK", CONTINUE);   // Retrieves the firmware ID (version number) of the module. 
  echoCommand("AT+CWMODE?","OK", CONTINUE);// Get module access mode. 
  
  
  echoCommand("AT+CWMODE=1", "", HALT);    // Station mode
  echoCommand("AT+CIPMUX=1", "", HALT);    // Allow multiple connections (we'll only use the first).

  //connect to the wifi
  boolean connection_established = false;
  for(int i=0;i<100;i++)
  {
    if(connectWiFi())
    {
      connection_established = true;
      break;
    }
  }
  if (!connection_established) errorHalt("Connection failed 2");
  
  //delay(2000);

  echoCommand("AT+CWSAP=?", "OK", CONTINUE); // Test connection
  echoCommand("AT+CIFSR", "", HALT);         // Echo IP address. (Firmware bug - should return "OK".)
  //echoCommand("AT+CIPMUX=0", "", HALT);      // Set single connection mode

  
  if(numplazas==0){
  value1="0";
}
if(numplazas==1){
  value1="1";
}
if(numplazas==2){
  value1="2";
}if(numplazas==3){
  value1="3";
}

String chain="GET /update?api_key=F9JDP64BPF93WAI4&field1="+value1+"&field2="+plaza1ocupada+"&field3="+estadoalarma+" HTTP/1.1\r\n";


// Establish TCP connection
  String cmd = "AT+CIPSTART=0,\"TCP\",\""; 
  cmd += DEST_IP; 
  cmd += "\",80";
  
  if (!echoCommand(cmd, "OK", CONTINUE)) return;
  //delay(2000);
  // Get connection status 
  if (!echoCommand("AT+CIPSTATUS", "OK", CONTINUE)) return;



cmd=chain;
cmd += "Host: api.thingspeak.com\r\n\r\n";
  
  // Ready the module to receive raw data
  if (!echoCommand("AT+CIPSEND=0,"+String(cmd.length()), ">", CONTINUE))
  {
    echoCommand("AT+CIPCLOSE", "", CONTINUE);
    Serial.println("Connection timeout.");
    return;
  }
  // Send the raw HTTP request
  echoCommand(cmd, "OK", CONTINUE);  // GET
}
