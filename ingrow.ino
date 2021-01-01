#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <BlynkSimpleEsp8266.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include "Irms_Calc.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

//falta: config de temperatura, notificaciones

WiFiServer server(80);
// IPAddress IP(192,168,1,1);
// IPAddress mask = (255, 255, 255, 0);

byte ledPin = 2;
bool habia_subido = false;
bool clear_credentials = false;
const int sensorIn = A0;


#define PIN_RELE1 0 //GPIO0=D3
#define PIN_RELE2 2 //GPIO2=D4
#define PIN_RELE3 14 //GPIO14=D5
#define PIN_RELE4 12 //GPIO12=D6
#define DHTPIN 4 //GPIO4=D2
#define PIN_SENSORES_TEMP 5  // GPIO5=D1. Pin donde se conecta el bus 1-Wire del sensor

#define TEMP_MAX 30
#define DHTTYPE    DHT11     // DHT 11


uint8_t id_sensor_indoor[8] = {0x28, 0xBD, 0x75, 0x20, 0x5F, 0x14, 0x01, 0x31}; 

const long utcOffsetInSeconds = -10800;
char daysOfWeek[7][12] = {"Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado", "Domingo"};

char auth[] = "flKxAwuFeMrjeuQUKFrrayiWZgBh2Ghg"; 

OneWire oneWireObjeto(PIN_SENSORES_TEMP);
DallasTemperature sensores(&oneWireObjeto);
SimpleTimer timer;
WiFiManager wifiManager;
DHT dht(DHTPIN, DHTTYPE);
ACS712_Irms acs712;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "south-america.pool.ntp.org", utcOffsetInSeconds);

void leerYSubirDatos()
{
  //mide la temperatura con DS18b20
  float temp_indoor = sensores.getTempC(id_sensor_indoor);
  Blynk.virtualWrite(V5, temp_indoor);  //V5: DS18b20  
  ledPulso();
  sensores.requestTemperatures();

  //mide la temperatura con DHT11
  // Leemos la humedad relativa
  float dht_h = dht.readHumidity();
  // Leemos la temperatura en grados centígrados (por defecto)
  float dht_t = dht.readTemperature();

  // Comprobamos si ha habido algún error en la lectura
  if (isnan(dht_h) || isnan(dht_t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    Blynk.virtualWrite(V6, -1.0);
  }
  else {
    Blynk.virtualWrite(V6, dht_h);
    Blynk.virtualWrite(V7, dht_h);
    //Blynk.virtualWrite(V6, dht_t);
  }

  double AmpsRMS;
  int cant_muestras = 10;
  
  for (int i = 0; i < cant_muestras; i++){
    AmpsRMS =  acs712.Process();
    delay(5);
  }
  Blynk.virtualWrite(V8, (float)AmpsRMS);

  timeClient.update();
  String dia =  /*daysOfWeek[timeClient.getDay()] + ", " +*/ 
                (String)timeClient.getHours() + ":" + 
                (String)timeClient.getMinutes() + ":" + 
                (String)timeClient.getSeconds();
  //Blynk.virtualWrite(V3, dia);

}

void cambiarRele(int pin, int estado_boton){
  if (estado_boton == 0){
    digitalWrite(pin, LOW);
    Serial.println("Boton APAGADO");
  }else{
    digitalWrite(pin, HIGH);
    Serial.println("Boton ENCENDIDO");
  }
}

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin V1
BLYNK_WRITE(V1)
{
  int estado = param.asInt();
  cambiarRele(PIN_RELE1, estado);
}

BLYNK_WRITE(V2)
{
  int estado = param.asInt();
  cambiarRele(PIN_RELE2, estado);
}

BLYNK_WRITE(V3)
{
  int estado = param.asInt();
  cambiarRele(PIN_RELE3, estado);
}

BLYNK_WRITE(V4)
{
  int estado = param.asInt();
  cambiarRele(PIN_RELE4, estado);
}

BLYNK_WRITE(V0)
{/*
  // You'll get HIGH/1 at startTime and LOW/0 at stopTime.
  // this method will be triggered every day
  // until you remove widget or stop project or
  // clean stop/start fields of widget
  int estado_boton = param.asInt(); // assigning incoming value from pin V1 to a variable

  if (estado_boton == 0){ //Stop time
    digitalWrite(PIN_RELE2, HIGH);
    digitalWrite(PIN_RELE3, HIGH);
    digitalWrite(PIN_RELE4, HIGH);
    Serial.println("Reles 2, 3 y 4 APAGADOS");
  }else{//Start time
    digitalWrite(PIN_RELE2, LOW);
    digitalWrite(PIN_RELE3, LOW);
    digitalWrite(PIN_RELE4, LOW);
    Serial.println("Reles 2, 3 y 4 ENCENDIDOS");
  }

  ledPulso();
  ledPulso();
*/
}


void mostrarDatos()
{
  // Buscamos los sensores conectados
    Serial.println("Buscando dispositivos...");
    Serial.print("Encontrados: ");
    int numeroSensoresConectados = sensores.getDeviceCount();
    Serial.print(numeroSensoresConectados);
    Serial.println(" sensores");

    // Si hemos encontrado uno mostramos su dirección
    if(numeroSensoresConectados > 0){
        
        // Tipo definido como un array de 8 bytes (uint8_t [8])
        DeviceAddress sensor_heladera;
        DeviceAddress sensor_camara;
        // Obtenemos dirección
        sensores.getAddress(sensor_heladera, 1);
        sensores.getAddress(sensor_camara, 2); //no funca bien, capaz no hace lo que pienso

        // Mostamos por el monitor serie
        Serial.print("Sensores encontrados: ");

        // Recorremos los 8 bytes del identificador único
        for (uint8_t i = 0; i < 8; i++)
        {
          // Si solo tiene un dígito rellenamos con un cero a la izquierda
          if (sensor_heladera[i] < 16) Serial.print("0");

          // Mostramos los datos que van en HEXADECIMAL
          Serial.print("0x");
          Serial.print(sensor_heladera[i], HEX);
        }
        Serial.print("\nResolucion: ");
        Serial.print(sensores.getResolution(id_sensor_indoor));
        
    }
}

void ledPulso(){
  /*
  digitalWrite(2, LOW);
  digitalWrite(LED_STATUS, HIGH);
  delay(100);
  digitalWrite(2, HIGH);
  digitalWrite(LED_STATUS, LOW);
  */
}


void configurarLEDs(){
  pinMode(2, OUTPUT); //LED del esp
  pinMode(16, OUTPUT); //LED del devkit
  //pinMode(LED_STATUS, OUTPUT);
  //pinMode(LED_ONOFF, OUTPUT);
  pinMode(PIN_RELE1, OUTPUT);
  
  
  digitalWrite(2, HIGH); //LED del esp que se apague al identificador  
  digitalWrite(16, LOW); //LED devkit como testigo de alimentacion
  //digitalWrite(LED_STATUS, LOW);
  //digitalWrite(LED_ONOFF, HIGH);

  digitalWrite(PIN_RELE1, LOW);
}

void configurarReles(){
  pinMode(PIN_RELE1, OUTPUT);
  pinMode(PIN_RELE2, OUTPUT);
  pinMode(PIN_RELE3, OUTPUT);
  pinMode(PIN_RELE4, OUTPUT);
  
  digitalWrite(PIN_RELE1, HIGH); //La carga esta conectada a NC, entonces la deja prendida
  digitalWrite(PIN_RELE2, HIGH); //La carga esta conectada a NC, entonces la deja prendida
  digitalWrite(PIN_RELE3, HIGH); //La carga esta conectada a NC, entonces la deja prendida
  digitalWrite(PIN_RELE4, HIGH); //La carga esta conectada a NC, entonces la deja prendida
}

void setup() {
  Serial.begin(115200);
  
  // WiFiManager wifiManager;
  // wifiManager.resetSettings();
  wifiManager.autoConnect("InESP_Door");
  delay(1000);
  Serial.println("Iniciando InESP_Door");
  configurarReles();
  ledPulso();
  ledPulso();
  ledPulso();
  Blynk.config(auth);
  delay(1000);
  Serial.println("paso por Blynk.config().");
  sensores.begin();
  //mostrarDatos();
  dht.begin();
  ArduinoOTA.begin();
  int cant_sensores = sensores.getDeviceCount();
  Serial.println("0. Cantidad de sensores: ");
  Serial.println(cant_sensores);
  while (cant_sensores == 0){
    delay(6000);
    cant_sensores = sensores.getDeviceCount();
  }
  Serial.println("1. Cantidad de sensores: ");
  Serial.println(cant_sensores);
  sensores.requestTemperatures();  
  Serial.println("Paso por requestTemperatures");
  timer.setInterval(5000L, leerYSubirDatos); //no menos de 5s por el DHT
  Serial.println("Paso por setInterval");

  acs712.ADCSamples = 1024.0; //1024 samples
  acs712.mVperAmp = scaleFactor::ACS712_30A; // use 100 for 20A Module and 66 for 30A Module and 185 for 5A Module
  acs712.maxADCVolt = 5.0; //5 Volts
  acs712.ADCIn = A0;
  acs712.Init(); 

  timeClient.begin();
}

void loop() {
  ArduinoOTA.handle();
  Blynk.run();
  timer.run();  
  
  // while (digitalRead(BTN_CONFIG) == HIGH){
  //   clear_credentials = true;
  // }
  // if (clear_credentials) {
  //   wifiManager.resetSettings();
  //   ESP.restart();
  // }
}
