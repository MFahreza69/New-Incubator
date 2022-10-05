//tes
#include "xbaby.h"
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include "src/library/SHT15/SHT1X.h"
#include "src/library/SimpleTimer/SimpleTimer.h"
#include <SoftwareSerial.h>

/*
    Define pins
*/
/*                 | Select Pin  | Atmega2560   |   Atmega64 |*/
#define shtData         16          /*3*/            /*24*/
#define shtClock        17          /*2*/            /*25*/
#define oxygenPin       A2          /*A5*/           /*A2*/
#define skinTemp1       A0
#define skinTemp2       A1
#define waterPin        14           /*4*/            /*14*/
#define warmerPin       13      /* default 5*/ /*auxiliary 13*/
#define heaterPin       4            /*11*/           /*4*/
#define fanPin          3           /*12*/           /*3*/
// #define valvePin        15
#define pinBuzzer       12
// #define O2Press         A3
#define pinButton       A3 /* pinButton with ADC */
#define pinOnOff        A4          /*A10*/          /*A4*/
#define voltage5V       A5          /*A10*/          /*A5*/
#define pinLamp         A7          /*38*/           /*A7*/
#define pinRst          33          /**/            /*33*/

/*
    Define variables
*/
/*sensor pin*/
float babySkinTemp0         = 0;
float babySkinTemp1         = 0;
float chamberTemp0          = 0;
float humidityMid           = 0;
uint8_t outWarmer;
uint8_t statHigh            = 0;
uint8_t callBtn             = 1;
unsigned long lastTime0     = 0;
unsigned long lastTime1     = 0;
unsigned long lastTime2     = 0;
/* power Button */
int numberA                 = 0;
int numberB                 = 0;
/* lockButton */
int lock                    = 1;
unsigned long last_time     = 0;
unsigned long last_time1    = 0;
unsigned long tcal          = 0;

/*variable data*/
int a = 0;
unsigned long b;
uint8_t G                   = 0;
uint8_t B                   = 0;
uint8_t fanPwm              = 0;
uint8_t heaterPwm           = 0;
uint8_t warmerPwm           = 0;
uint8_t valvePwm            = 0;
uint8_t skinMode            = 0; //skinmode =1 = BabyMode || skinmode =2 = Airmode
uint8_t menuMode            = 0;
uint8_t humiMode            = 0;
uint8_t pos                 = 0;
uint8_t last_pos            = 0;
uint8_t highTemp            = 0;
uint8_t alarmValue;
uint8_t sunyiValue          = 0;
uint8_t silent              = 0;
uint8_t debugging           = 1;
uint8_t error0              = 0; 
uint8_t error1              = 0; 
uint8_t error2              = 0; 
uint8_t error3              = 0; 
uint8_t error4              = 0; 
uint8_t error5              = 0; 
uint8_t error6              = 0; 
// uint8_t last_sunyi_value    = 0;
uint8_t callStatt           = 0;
uint8_t setFiO2             = 20;
float displaySetTemp        = 27;
float displayBabyTemp0      = 0;
float displayBabyTemp1      = 0;
uint8_t displayOxygen       = 0;
float lastDisplay           = 0;
float lastHumidity          = 0;
float lastFiO2              = 0;
float setTemp               = 27;
float setTemp1              = 27;
uint8_t setHumidity         = 60;
//default A= -0.1854, B= 58.551
float setValue0A            = 0.022/*0.3027*/;
float setValue0B            = 21.023/*-214.88*/;
float setValue1A            = 0.022/*0.1154*/;
float setValue1B            = 21.023/*-32.134*/;
float errorData             = 0;
float errorHumidity         = 0;
unsigned long lastTime3     = 0;
uint8_t loopAlarm           = 0;
uint8_t lastError0          = 0;
uint8_t lastError1          = 0;
uint8_t lastError2          = 0;
uint8_t lastError3          = 0;
uint8_t lastError4          = 0;
uint8_t lastError5          = 0;
uint8_t lastError6          = 0;
uint8_t suhu                = 0;
uint8_t sensor              = 0;
// uint8_t O2                  = 0;
uint8_t heatedPower         = 0;
//error variable
uint8_t powerIn             = 0;
uint8_t waterIn             = 0;       

//Alarm variable data
float deviationAir = 0;
float deviationSkin0 = 0;
float deviationSkin1 = 0;
uint8_t steadytime0 = 0;
uint8_t steadytime1 = 0;
float errorAir;
float errorSkin0;
float errorSkin1;

//timer Variable data
uint8_t timeMode = 0;

static char userInput[255];
static unsigned char x;
static unsigned char x2;
int sampleData[20];
int sampleData1[20];
int readData[21];
char in = 1;
int g = 0;
float h;
int i = 0;


double setPoint1; 
double input;
double outputHeater;
double outputFan;

PID myPID(&input, &outputHeater, &setPoint1, 2, 5 ,1 ,DIRECT);
PID myPID2(&input, &outputFan, &setPoint1, 2, 5, 1, DIRECT);

Xbaby Xinfant;
RTC_DS3231 rtc;
SHT1x sht15(shtData, shtClock);
SoftwareSerial mySerial(6, 24); // RX, TX // Only Use RX
SimpleTimer timer0;
SimpleTimer timer1;
SimpleTimer timer3;

void pin_setup(){
      /* pin led */
  pinMode(26, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(warmerPin, OUTPUT);
  pinMode(pinRst, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinLamp, OUTPUT);
  pinMode(voltage5V, INPUT);
  pinMode(waterPin, INPUT);
}

void setup(){
    mySerial.begin(9600);
    Serial1.begin(9600);
    // rtc.begin();
    // Serial.begin(9600);
    // setup_fast_pwm();
    pin_setup();
    timer0.setInterval(200, generate_json); 
    timer3.setInterval(500, PID);
    analogWrite(warmerPin, 0);
    myPID.SetOutputLimits(0,255);
    myPID.SetMode(AUTOMATIC);
    myPID2.SetOutputLimits(70, 255);
    myPID2.SetMode(AUTOMATIC);
}
////////////////////////////////////////////////////////////////////////////////////////

void loop(){
    run_program();
}

void run_program(){
    timer0.run();
    timer3.run();
    communication_serial();
    digitalWrite(pinLamp, HIGH);
    digitalWrite(26, LOW); 
    read_skin_temperature();
    read_temperature();
    // run_warmer();
    // run_control();
    read_error();
    alarm();

    // pewaktu();
    // Serial.println(analogRead(skinTemp1));
    // doremi();
}

/*Communication Data*/////////////////////////////////////////////////////////////////////////////////////////
//Read data com input using mySerial (RX pin 6)
void communication_serial(){
    while(mySerial.available()>0){
        userInput[x2] = mySerial.read();
        x2++;
        if(userInput[x2-1] == '\n'){
            // Serial.println(userInput); // check the data if it was received or not
            StaticJsonDocument<255>in;
            DeserializationError error = deserializeJson(in, userInput);
            if(!error){
                setTemp = in["data1"]["sn"][0];
                setHumidity = in["data1"]["sn"][1];
                skinMode = in["data1"]["mode"][0];
                humiMode = in["data1"]["mode"][1];
                highTemp = in["data1"]["mode"][2];
                alarmValue = in["data1"]["mode"][3];
                sunyiValue = in["data1"]["mode"][4];
                timeMode = in["data1"]["mode"][5];
            }
            x2 = 0;
        }
    }                
}

//Send data using Serial 1 (TX pin 21/PD3) but send data json
void generate_json(){
//   DateTime now = rtc.now();
  StaticJsonDocument<512> outDbg;
  StaticJsonDocument<512> outBtn;
        outDbg["suhu"][0] = chamberTemp0;
        outDbg["suhu"][1] = babySkinTemp0;
        outDbg["suhu"][2] = babySkinTemp1;
        outDbg["suhu"][3] = humidityMid;
        // outDbg["pow"][0] = outputFan;
        outDbg["pow"][0] = outputHeater;
        // outDbg["tim"][0] = now.hour();
        // outDbg["tim"][1] = now.minute();
        // outDbg["tim"][2] = now.second();
        outDbg["err"][0] = error0; //probe missing
        outDbg["err"][1] = error1; // alarm deviation airway
        outDbg["err"][2] = error2; // alarm deviation skin
        outDbg["err"][3] = error3;
        outDbg["err"][4] = error4;
        outDbg["err"][5] = error5;
        outDbg["err"][6] = powerIn;
        serializeJson(outDbg, Serial1);
        Serial1.println();
        // serializeJson(outDbg, Serial);
        // Serial.println();    
     
}
/*END Communication Data*/////////////////////////////////////////////////////////////////////////////////////////


/*Read Skin, Airway, HUMidi*/
void read_temperature(){
    float read_sht_temperature = 0;
    float read_sht_humidity = 0;
    read_sht_temperature = sht15.readTemperatureC();
    read_sht_humidity = sht15.readHumidity();
    if(read_sht_temperature < 1){
        chamberTemp0 = 0;
    }
    if(read_sht_temperature > 1){
        chamberTemp0 = read_sht_temperature;
    }
    if(read_sht_humidity < 1){
        humidityMid = 0;
    }
    if(read_sht_humidity > 1){
        humidityMid = read_sht_humidity;
    }
}
/*end read sensor*/

void read_skin_temperature(){
    float read_skin_temperature0 = Xinfant.get_value_baby_skin(setValue0A, setValue0B, skinTemp1);
    float read_skin_temperature1 = Xinfant.get_value_baby_skin(setValue1A, setValue1B, skinTemp2);
    if(read_skin_temperature0 > 21){
        babySkinTemp0 = read_skin_temperature0;
        if(millis() - tcal > 1000){
            displayBabyTemp0 = babySkinTemp0;
        }
    }
    if(read_skin_temperature0 < 22){
        babySkinTemp0 = 0;
        displayBabyTemp0 = babySkinTemp0;
    }
    if(read_skin_temperature1 > 21){
        babySkinTemp1 = read_skin_temperature1;
        if(millis() - tcal > 1000){
            displayBabyTemp1 = babySkinTemp1;
        }
    }
    if(read_skin_temperature1 < 22){
        babySkinTemp1 = 0;
        displayBabyTemp1 = babySkinTemp1;
    }
}

/*Control*/
// void run_control(){
//     fanPwm = Xinfant.get_value_fan(setTemp, skinMode, chamberTemp0, babySkinTemp0, babySkinTemp1, setHumidity, humidityMid);
//     heaterPwm = Xinfant.get_value_heater(setTemp, skinMode , highTemp, chamberTemp0, babySkinTemp0, babySkinTemp1);
//     set_pwm(fanPwm, heaterPwm);
//         if(skinMode == 0 && humiMode == 0){
//         set_pwm(0, 0);
//         analogWrite(warmerPin, 0);
//     }
// }

// void setup_fast_pwm(){
//     TCCR3A = (1<<COM3B1)|(1<<COM3B0)|(1<<COM3A1)|(0<<COM3A0)|(1<<WGM32)|(1<<WGM31)|(1<<WGM30);
//     TCCR3B = (1<<CS32)|(0<<CS31) | (1<<CS30);
//     OCR3A = 0;
//     OCR3B = 1023;
// }

void set_pwm(int fan, int heater){
    // OCR3A = heater;
    // OCR3A = fan;
    analogWrite(heaterPin, heater);
    analogWrite(fanPin, fan);
}

void run_warmer(){
     errorHumidity = (setHumidity * 10) - (humidityMid * 10);
     if(humiMode == 1){
        if(humidityMid < 0){
            analogWrite(warmerPin, 0);
            warmerPwm = 0;
        }else {
            if(errorHumidity < -0.5 && errorHumidity > -35){
                analogWrite(warmerPin, 0);
                warmerPwm = 0;
            }  
            if(errorHumidity < 950 && errorHumidity >= 307){
                analogWrite(warmerPin, 255); //255
                warmerPwm = 255;
            }
            if(errorHumidity < 307 && errorHumidity>= 105){
                analogWrite(warmerPin, 215); //215
                warmerPwm = 215;
            }
            if(errorHumidity < 105 && errorHumidity >= 70 ){
                analogWrite(warmerPin, 200); //200
                warmerPwm = 200;
            }
            if(errorHumidity < 70 && errorHumidity >= 20){
                analogWrite(warmerPin, 190); //190
                warmerPwm = 190;
            }
            if(errorHumidity < 20 && errorHumidity >= 5){
                analogWrite(warmerPin, 180); //180
                warmerPwm = 180;
            }
            if(errorHumidity < 5 && errorHumidity >= -0.5){
                analogWrite(warmerPin, 180); //180
                warmerPwm = 180; //100
            }
        }
   }
   if(humiMode == 0){
     analogWrite(warmerPin, 0);
     warmerPwm = 0;
   }
}

/*End Control*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void pewaktu(){
//     if(timeMode == 0){
//         rtc.adjust(DateTime(0000,0,00,00,00,1)); 
//     }    
// }

/*Error Session*/

void read_error(){
    errorAir = (setTemp * 10) - (chamberTemp0 * 10);
    errorSkin0 = (setTemp * 10) - (babySkinTemp0 * 10);
    errorSkin1 = (setTemp * 10) - (babySkinTemp1 * 10);
    /*probe sensor missing*///////////////////////
    if(babySkinTemp0 == 0 || chamberTemp0 == 0){
        error0 = 1;
    }
    if(babySkinTemp0 != 0 && chamberTemp0 != 0){
        error0 = 0;
    }
    /*end probe sensor missing*/////////////////////


    /*Temp Deviation Alarm*////////////////////////
    if(skinMode == 1){
        steadytime1 = 0;
        error2 = 0;
        if(errorSkin0 < 1 && errorSkin0 >= -1 || errorSkin1 < 1 && errorSkin1 >=-1){
            steadytime0++;
            error1 = 0;
            if(steadytime0 >= 60){
                steadytime0 = 60;
            }
        }
        if(steadytime0 == 60){
            if(babySkinTemp0 >= setTemp+1 || babySkinTemp0 <= setTemp-1 || babySkinTemp1 >= setTemp +1 || babySkinTemp1 <= setTemp-1){
                error1 = 1;
            }
            else{
                error1 = 0;
            }
        }  
    }
    if(skinMode == 2){
        steadytime0 = 0;
        error1 = 0;
        if(errorAir < 1 && errorAir >= -1){
            steadytime1++;
            error2 = 0;
            if(steadytime1 >= 60){
                steadytime1 = 60;
            }
        }
        if(steadytime1 == 60){
            if(chamberTemp0 >= setTemp+1 || chamberTemp0 <= setTemp-1){
                error2 = 1;
            }
            else{
                error2 = 0;
            }
        }
    }
    if(skinMode == 0){
        steadytime0 = 0;
        steadytime1 = 0;
    } 
       
   /*End Temp Deviation Alarm*////////////////////////


   /*High Temperature Alarm*//////////////////////////
    if(highTemp == 0 && chamberTemp0 >= 38 || babySkinTemp0 >= 38 || babySkinTemp1 >= 38){
        error3 = 1;
    }else{
        error3 = 0;
    }
    if(highTemp == 1 && chamberTemp0 >= 39.5 || babySkinTemp0 >= 38.5 || babySkinTemp1 >= 38.5){
        error4 = 1;
    }else{
        error4 = 0;
    }
    /*end High Temperature Alarm*/////////////////////////
    
   
    /*Power Failure*/
    powerIn = digitalRead(voltage5V);
    if(powerIn == 1){
        error5 = 0;
    }
    if(powerIn == 0){
        error5 = 1;
    }

    /*Water at low Level*/
    waterIn = digitalRead(waterPin);
    if(waterIn == 1){
        error6 = 1;
    }
    if(waterIn == 0){
        error6 = 0;
    }

}
/*End read Error data or missing sensor*/////////////////////////////////////////////////////////////////////////////////////

/*alarm status*/
void alarm(){
    // Serial.println(alarmValue);
    if(sunyiValue == 1 && alarmValue == 0){
        if(millis() - lastTime3 > 500 && loopAlarm == 0){
            lastTime3 = millis();
            loopAlarm = 1;
            tone(pinBuzzer, 1900);
            // Serial.println("Alarm!");
        }
        if(millis() - lastTime3 > 500 && loopAlarm == 1){
            lastTime3 = millis();
            loopAlarm = 0;
            noTone(pinBuzzer);
        }
    }
    if(sunyiValue == 1 && alarmValue == 1){
        // Serial.println("Alarm!!!!");
    }
    else{
        noTone(pinBuzzer);
    }
}
/*End alarm status*/

// /*tone buzzer function*/
void PID(){
setPoint1 = setTemp; 
if(skinMode == 2){
    input = chamberTemp0;
    myPID.Compute();
    myPID2.Compute();
    set_pwm(outputFan, outputHeater);
}
else{
    set_pwm(outputFan, outputHeater);
    outputHeater = 0;
    outputFan = 0;
    }


}
