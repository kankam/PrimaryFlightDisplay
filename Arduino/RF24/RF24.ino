/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

bool led;
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00011";
void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("start listening");
  pinMode(9, OUTPUT);
  led =0;
}
void loop() {
  led = !led;
  if (radio.available()) {

    char text[50] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }  
 // else {Serial.println("nope");}
 digitalWrite(9, led);
}
