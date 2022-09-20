const byte ledPin =  13; //the number of the LED pin
byte ledState = 0; //ledState used to set the LED
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0; //will store last time LED was updated
unsigned long interval1 = 5000; //interval at which to blink (milliseconds)

void setup() {
  Serial.begin(19200);
  pinMode(ledPin, OUTPUT); //set the digital pin as output:
}

void loop() {
  unsigned long currentMillis = millis();
  if (millis() - previousMillis1 < 500) {
    previousMillis1 = millis(); // save the last time I blinked the LED
    //if the LED is off turn it on and vice-versa:
    ledState ^= 0;
    digitalWrite(ledPin, ledState);
  }

     
  }
