// Adafruit IO Publish & Subscribe Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"
#include "Wire.h" 
/************************ Example Starts Here *******************************/

// Dirección I2C del dispositivo (esclavo)
#define DEVICE_ADDR 0x50 // Pines D21 y D22
// D21 SDA
// D22 SCL
char command = 0;
// this int will hold the current count for our sketch
int count = 0;
int dc ; //Variable de prueba para mandar valores a push1
int servo ;

int sultra = 0; //Variable para sensor ultrasonico
int stemp = 0; //Variable sensor de temperatura
int sLuz = 0; //Variable sensor de Luz
int command1; //Commando DC push1
int command2; //Comando Servo push2

// Variables para almacenar los valores anteriores de los sensores
int sultraAnterior = 0;
int stempAnterior = 0;
int sLuzAnterior = 0;

String valorDC = ""; //Valor que recibimos del Push1
String valorSer = ""; //Valor que recibimos del Push2

// Buffer para almacenar datos recibidos
byte data_received[3]; // 3 para recibir tres valores

// Variable para almacenar el valor decimal
int decimal_value = 0;
int last_value = -1;

// Track time of last published messages and limit feed->save events to once
// every IO_LOOP_DELAY milliseconds.
//
// Because this sketch is publishing AND subscribing, we can't use a long
// delay() function call in the main loop since that would prevent io.run()
// from being called often enough to receive all incoming messages.
//
// Instead, we can use the millis() function to get the current time in
// milliseconds and avoid publishing until IO_LOOP_DELAY milliseconds have
// passed.
#define IO_LOOP_DELAY 5000
//#define IO_LOOP_DELAY 15000
unsigned long lastUpdate = 0;

// set up the 'counter' feed
AdafruitIO_Feed *push1 = io.feed("push1"); //DC motor manual
AdafruitIO_Feed *push2 = io.feed("push2"); //Servo motor manual
AdafruitIO_Feed *temp = io.feed("temp");  //Sensor de Temperatura
AdafruitIO_Feed *ultra = io.feed("ultra"); //Sensor ultrasonico
AdafruitIO_Feed *luz = io.feed("luz"); //Sensor de Luz


void handleMessage(AdafruitIO_Data *data);
void handleMessage1(AdafruitIO_Data *data);

void requestEvent();

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the count feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  push1->onMessage(handleMessage);
  push2->onMessage(handleMessage1);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  push1->get();
  push2->get(); 
  ultra->get();
  temp->get();
  luz->get();

  // Inicialización del bus I2C como esclavo con dirección DEVICE_ADDR
  Wire.begin(DEVICE_ADDR);

  // Asignar funciones para manejar eventos de recepción y solicitud
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void receiveEvent(int byteCount) {
  static int index = 0; // Índice para rastrear cuál sensor se está recibiendo
  //byte data_received;   // Variable temporal para almacenar el byte recibido

  while (Wire.available()) {
    // Leer un byte de datos
    byte data_received = Wire.read();
//Mandar caracter para saber que es sENsor 1, 1. 2. 3. A. B 
    // Almacenar el valor en la variable correspondiente
    if (index == 0) {
      sLuz = data_received;
    } else if (index == 1) {
      stemp = data_received;
    } else if (index == 2) {
      // Limitar el valor a 20 si es mayor
      sultra = data_received > 20 ? 20 : data_received;
    } /*else if(index == 3){
      command1 = data_received;
    }else if(index == 4){
      command2 = data_received;
    }*/

    // Imprimir el valor recibido para depuración
    Serial.print("Sensor ");
    Serial.println(index + 1);
    /*Serial.print("Command1");
    Serial.println(command1); */
    /*Serial.print(" -> Valor decimal: ");
    Serial.println(data_received);*/

    // Avanzar al siguiente sensor
    index++;

    // Reiniciar el índice si ya se han recibido los tres valores
    if (index >= 3) {
      index = 0;
    }
  }
}

// Función para manejar la solicitud de datos del maestro
void requestEvent() {
  byte response = 0;
  static int index2 = 0;
  int valorDC_int = valorDC.toInt();
  int valorSer_int = valorSer.toInt();
  if (index2 == 0) {
    response = valorDC_int; // Enviar valor de sensor ultrasonico
  } else if (index2 == 1) {
    response = valorSer_int; // Enviar valor de sensor de temperatura
  }/* else {
    response = 0;
  }*/
  Wire.write(response);
  index2++;
  if(index2 >= 2){
    index2 = 0;
  }
  /*Serial.print("Enviando valor: ");
  Serial.println(response);*/
}

void loop() {
  io.run();
   
  if (millis() > (lastUpdate + IO_LOOP_DELAY)) {
    if (sultra != sultraAnterior) {
      Serial.print("sending ultra -> ");
      Serial.println(sultra);
      ultra->save(sultra);
      sultraAnterior = sultra;
    }

    if (stemp != stempAnterior) {
      Serial.print("sending temp -> ");
      Serial.println(stemp);
      temp->save(stemp);
      stempAnterior = stemp;
    }

    if (sLuz != sLuzAnterior) {
      Serial.print("sending luz -> ");
      Serial.println(sLuz);
      luz->save(sLuz);
      sLuzAnterior = sLuz;
    }

    lastUpdate = millis();
    
  }
}

// this function is called whenever a 'counter' message
// is received from Adafruit IO. it was attached to
// the counter feed in the setup() function above.
void handleMessage(AdafruitIO_Data *data) {

  valorDC = data->value();
  Serial.print("received DC <- ");
  Serial.println(valorDC);
  //Serial.println(data->value());

}

void handleMessage1(AdafruitIO_Data *data) {
  valorSer = data->value();
  Serial.print("received Servo <- ");
  Serial.println(valorSer);
  //Serial.println(data->value());

}
