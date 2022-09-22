
//tes

#include "xbaby.h"
#include <Servo.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include "src/library/O2control/XMV20_O2Sensor.h"
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
#define servoPin        14           /*4*/            /*14*/
#define warmerPin       5 /* default 5*/ /*13*/
#define heaterPin       4            /*11*/           /*4*/
#define fanPin          3           /*12*/           /*3*/
#define valvePin        15
#define pinBuzzer       12
#define O2Press         A3
#define pinButton       A3 /* pinButton with ADC */
#define pinOnOff        A4          /*A10*/          /*A4*/
#define voltage5V       A5          /*A10*/          /*A5*/
#define pinLamp         A7          /*38*/           /*A7*/
#define pinRst          33          /**/            /*33*/

/*buzzer tone*/
#define DO  262
#define RE  294
#define MI  330
#define FA  349
#define SOL 395
#define LA  440
#define SI  494
#define DOO 523
/*
    Define variables
*/
/*sensor pin*/
float babySkinTemp0         = 0;
float babySkinTemp1         = 0;
float chamberTemp0          = 0;
float humidityMid           = 0;
float O2Pressure            = 0;
uint8_t fio2Mid             = 0;
uint8_t spo2Mid             = 0;
uint8_t bpmMid              = 0;
uint8_t oxygen              = 0;
uint8_t outWarmer;

uint8_t valuePower0         = 0; //Masuk ke Menu Value
uint8_t valuePower1         = 0; //Masuk ke pengaturan nilai (temp,humi,O2)
uint8_t valuePower2         = 0; //Untuk ON/OFF
uint8_t statHigh            = 0;
uint8_t callBtn             = 1;
unsigned long lastTime0     = 0;
unsigned long lastTime1     = 0;
unsigned long lastTime2     = 0;
/* power Button */
int numberA                 = 0;
int numberB                 = 0;
/* lockButton */
// int timeBtn0                = 0;
// int timeBtn1                = 0;
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
uint8_t alarmValue          = 0;
uint8_t sunyiValue          = 1;
uint8_t debugging           = 1;
uint8_t error0              = 0; 
uint8_t error1              = 0; 
uint8_t error2              = 0; 
uint8_t error3              = 0; 
uint8_t error4              = 0; 
uint8_t error5              = 0; 
uint8_t error6              = 0; 
uint8_t error7              = 0; 
uint8_t error8              = 0; 
uint8_t last_sunyi_value    = 0;
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
int autoMoveModeBaby        = 0;
int autoMoveModeAir         = 0;
uint8_t lastError0          = 0;
uint8_t lastError1          = 0;
uint8_t lastError2          = 0;
uint8_t lastError3          = 0;
uint8_t lastError4          = 0;
uint8_t lastError5          = 0;
uint8_t lastError6          = 0;
uint8_t suhu                = 0;
uint8_t sensor              = 0;
uint8_t O2                  = 0;
uint8_t heatedPower         = 0;

//Alarm variable data
float deviationAir = 0;
float deviationSkin0 = 0;
float deviationSkin1 = 0;

static char userInput[255];
static char userInput1[255];
static char userInput2[255];
static unsigned char x;
static unsigned char x1;
static unsigned char x2;
int sampleData[20];
int sampleData1[20];
int readData[21];
char in = 1;
int g = 0;
float h;
int i = 0;
float j;
int k = 0;
float l;
int m = 0;
float n;
int o = 0;
float p;
int q = 0;

Xbaby Xinfant;
O2Sensor O2SensorDev(oxygenPin);
SHT1x sht15(shtData, shtClock);
SoftwareSerial mySerial(6, 24); // RX, TX // Only Use RX
SimpleTimer timer0;
SimpleTimer timer1;

void pin_setup(){
  pinMode(pinOnOff, INPUT);
      /* pin led */
  pinMode(26, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(warmerPin, OUTPUT);
  pinMode(voltage5V, OUTPUT);
  pinMode(pinRst, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinLamp, OUTPUT);
}

void setup(){
    mySerial.begin(9600);
    Serial.begin(9600);
    Serial1.begin(9600);
    // setup_fast_pwm();
    pin_setup();
    timer0.setInterval(1500, generate_json);
    analogWrite(voltage5V, 255);
    analogWrite(warmerPin, 0);
}
////////////////////////////////////////////////////////////////////////////////////////

void loop(){
    run_program();
    
}

void run_program(){
    on_power();
    timer0.run();
  if(valuePower2 == 1){
    communication_serial();
    digitalWrite(pinLamp, HIGH);
    digitalWrite(26, LOW); 
    read_skin_temperature();
    read_temperature();
    run_warmer();
    run_control();
    // doremi();
    
    // if(skinMode == 0 && humiMode == 0){
    //     set_pwm(0, 0);
    //     analogWrite(warmerPin, 0);
    // }
    // read_error(5);
  }
  if(valuePower2 == 0){
    // on_power();
    noTone(pinBuzzer);
    chamberTemp0 = 0;
    babySkinTemp0 = 0;
    babySkinTemp1 = 0;
    humidityMid = 0;
    skinMode = 0;
    humiMode = 0;
    set_pwm(0, 0);
    digitalWrite(pinLamp, LOW);
    digitalWrite(26, HIGH);
    analogWrite(warmerPin, 0);
    }
}

/*Communication Data*/////////////////////////////////////////////////////////////////////////////////////////
//Read data com input using mySerial (RX pin 6)
void communication_serial(){
    while(mySerial.available()>0){
        userInput[x2] = mySerial.read();
        x2++;
        if(userInput[x2-1] == '\n'){
            Serial.println(userInput); // check the data if it was received or not
            StaticJsonDocument<255>in;
            DeserializationError error = deserializeJson(in, userInput);
            if(!error){
                setTemp = in["data1"]["sn"][0];
                setHumidity = in["data1"]["sn"][1];
                skinMode = in["data1"]["mode"][0];
                humiMode = in["data1"]["mode"][1];
                highTemp = in["data1"]["mode"][2];
            }
            x2 = 0;
        }
    }                
}

//Send data using Serial 1 (TX pin 21/PD3) but send data json in string
void generate_json(){
  StaticJsonDocument<512> outDbg;
  StaticJsonDocument<512> outBtn;
  if(valuePower2 == 1){
        outDbg["suhu"][0] = chamberTemp0;
        outDbg["suhu"][1] = babySkinTemp0;
        outDbg["suhu"][2] = babySkinTemp1;
        outDbg["suhu"][3] = humidityMid;
        outDbg["btn"][0] = valuePower2;
        outDbg["pow"][0] = heaterPwm;
        outDbg["err"][0] = fanPwm;
        outDbg["err"][1] = heaterPwm;
        outDbg["err"][2] = warmerPwm;
        outDbg["err"][3] = error3;
        outDbg["err"][4] = error4;
        outDbg["err"][5] = error5;
        outDbg["err"][6] = error6;
        
        // serializeJson(outDbg, Serial);
        // Serial.println();
        serializeJson(outDbg, Serial1);
        Serial1.println();
    }
  if(valuePower2 == 0){
        outBtn["btn"][0] = valuePower2;
        outBtn["btn"][1] = sunyiValue;
        serializeJson(outBtn, Serial1);
        Serial1.println();
  }    
}
/*END Communication Data*/////////////////////////////////////////////////////////////////////////////////////////


/*Button ON/OFF*/
void on_power(){
    int onOff = analogRead(pinOnOff);
    if(onOff >= 1020 && onOff <= 1030){ //tombol on/off ditekan lama sampai nilai analog mencapai range 1020-1030
        numberA++;
    }
    if(onOff >= 0 && onOff <= 500){ //tombol on/off tidak ditekan lama nilai numberA tidak akan bertambah
        numberA = 0;
        numberB = 0;
    }
    /* on */
    if(numberA >= 200 && valuePower2 == 0){ //kondisi awal valuepower2=0 dan nilai numberA naik setelah pin on/off mencapai nilai range
        if(numberB == 0){ //kondisi awal numberB=0
            valuePower2 = 1; //indikator sistem ON
            numberB = 1;
        }
    }
    /* off*/
    if(numberA >= 1 && valuePower2 == 1){ //tombol on/off ditekan kembali sampai nilaiA = 8 dan kondisi valuePower2=1
        if(numberB == 0){
            valuePower2 = 0;
            numberB = 1;
        }
    }
}
/*end button ON/OFF*/

/*Read Skin, Airway, HUMidi*/
void read_temperature(){
    float read_sht_temperature = 0;
    float read_sht_humidity = 0;
    read_sht_temperature = sht15.readTemperatureC();
    read_sht_humidity = sht15.readHumidity();
    if(read_sht_temperature <= 0){
        chamberTemp0 = 0;
    }
    if(read_sht_temperature != 0 || read_sht_temperature > 0){
        chamberTemp0 = read_sht_temperature;
    }
    if(read_sht_humidity <= 0){
        humidityMid = 0;
    }
    if(read_sht_humidity != 0 || read_sht_humidity > 0){
        humidityMid = read_sht_humidity;
    }
    // Serial1.print("Pembacaan Temperatur = ");
    // Serial1.print(chamberTemp0);
    // Serial1.print("      Pembacaan Kelembaban = ");
    // Serial1.println(humidityMid);
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
    //  Serial.print("Skin Temp 1 = ");
    //  Serial.print(displayBabyTemp1);
    //  Serial.print("      analog Read sensor = ");
    //  Serial.println(analogRead(A0));

}


/*Control*/
void run_control(){
    fanPwm = Xinfant.get_value_fan(setTemp, skinMode, chamberTemp0, babySkinTemp0, babySkinTemp1, setHumidity, humidityMid);
    heaterPwm = Xinfant.get_value_heater(setTemp, skinMode , highTemp, chamberTemp0, babySkinTemp0, babySkinTemp1);
    set_pwm(fanPwm, heaterPwm);
        if(skinMode == 0 && humiMode == 0){
        set_pwm(0, 0);
        analogWrite(warmerPin, 0);
    }
    // if(heaterPwm < 10 || fanPwm < 10){
    //     set_pwm(0, 0);
    // }
}

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
            if(errorHumidity < -1.5 && errorHumidity > -35){
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
            if(errorHumidity < 70 && errorHumidity > 20){
                analogWrite(warmerPin, 190); //190
                warmerPwm = 190;
            }
            if(errorHumidity < 20 && errorHumidity > 5){
                analogWrite(warmerPin, 180); //180
                warmerPwm = 180;
            }
            if(errorHumidity < 5 && errorHumidity > 1){
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

/*alarm status*/
// void alarm(){
//     if(sunyiValue == 1){
//         if(millis() - lastTime3 > 500 && loopAlarm == 0){
//             lastTime3 = millis();
//             loopAlarm = 1;
//             tone(pinBuzzer, 494);
//         }
//         if(millis() - lastTime3 > 500 && loopAlarm == 1){
//             lastTime3 = millis();
//             loopAlarm = 0;
//             noTone(pinBuzzer);
//         }
//     }else{
//         noTone(pinBuzzer);
//     }
// }
/*End alarm status*/


/*Error Session*/

/*cek sensor condition*/

void read_error(){
lastError0 = error0; // probe missing
    lastError1 = error1; // deviation air
    lastError2 = error2; // deviation skin
    lastError3 = error3;
    lastError4 = error4;
    lastError5 = error5;
    lastError6 = error6;
    float errorAir = (setTemp * 10) - (chamberTemp0 * 10);
    float errorSkin0 = (setTemp * 10) - (babySkinTemp0 * 10);
    float errorSkin1 = (setTemp * 10) - (babySkinTemp1 * 10);

    /*probe sensor missing*///////////////////////
    read_temperature();
    read_skin_temperature();
    if(babySkinTemp0 == 0 || babySkinTemp1 == 0 || chamberTemp0 == 0){
        error0 = 1;
    }
    if(babySkinTemp0 != 0 && babySkinTemp1 !=0 && chamberTemp0 != 0){
        error0 = 0;
    }
    /*end probe sensor missing*/////////////////////


    /*Temp Deviation Alarm*////////////////////////
    if(skinMode == 1){
        // error1 = 0;
        if(errorSkin0 < 2.5 && errorSkin0 >= -2.5 || errorSkin1 < 2.5 && errorSkin1 >= -2.5 ){
            // risetime = 2;
            // deviationSkin0 = 0;
            // deviationSkin1 = 0;
            // if(risetime == 2 && babySkinTemp0 >= setTemp + 1 || babySkinTemp0 <= setTemp +1 || babySkinTemp1 <= setTemp+1 || babySkinTemp1 >= setTemp+1){
            //     deviationSkin0 = 1;
            // }
            // // if(errorSkin0 > -9 && errorSkin0 < 9 && risetime == 2){
            // //     deviationSkin0 = 0;
            // // }
            // // if(errorSkin1 < -9 || errorSkin1 > 9 && risetime == 2){
            //     // deviationSkin1 = 1;
            // // }
            // // if(errorSkin1 > -9 && errorSkin1 < 9 && risetime == 2){
            //     // deviationSkin1 = 0;
            // // }
            // if(deviationSkin0 == 1 || deviationSkin1 == 1){
            //     error2 = 1;
            //     risetime = 0;
            // }if(deviationSkin0 == 0 || deviationSkin1 == 0){
            //     error2 = 0;
            // }   
        } 
    }
    //// BAY DIDIEU WKWK //////////
    if(errorAir < 3 && errorAir >= -3){
        // error2 = 0;
        risetime++;
        deviationAir = 0;
    }   
    if(risetime >= 10000 && chamberTemp0 >= setTemp+3 || chamberTemp0 <= setTemp+3){
        deviationAir = 1;
        risetime = 10000;
    }
    if(deviationAir == 1){
        // error2 = 1;
        risetime = 0;
    }
    if(deviationAir == 0){
        // error2 = 0;
    }
   /*End Temp Deviation Alarm*////////////////////////
    
   /*High Temperature Alarm*//////////////////////////
    if(highTemp == 0 && chamberTemp0 >= 38 || babySkinTemp0 >= 38 || babySkinTemp1 >= 38){
        error3 = 1;
    }else{
        error3 = 0;
    }
    if(highTemp == 1 && chamberTemp0 >= 39.5 || babySkinTemp0 >= 38 || babySkinTemp1 >= 38){
        sunyiValue = 1;
        error4 = 1;
    }else{
        sunyiValue = 2;
        error4 = 0;
    }
    /*end High Temperature Alarm*/////////////////////////
    
   

    if(error0 != lastError0){
        if(error0 != 0)
            sunyiValue = 1;
        if(error0 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error1 != lastError1){
        if(error1 != 0)
            sunyiValue = 1;
        if(error1 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error2 != lastError2){
        if(error2 != 0)
            sunyiValue = 1;
        if(error2 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error3 != lastError3){
        if(error3 != 0)
            sunyiValue = 1;
        if(error3 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error4 != lastError4){
        if(error4 != 0)
            sunyiValue = 1;
        if(error4 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error5 != lastError5){
        if(error5 != 0)
            sunyiValue = 1;
        if(error5 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }
    if(error6 != lastError6){
        if(error6 != 0)
            sunyiValue = 1;
        if(error6 == 0 && last_sunyi_value == 2)
            sunyiValue = 2;
    }

}

    /*high temp*/

    /*end high temp*/


    /*auto alarm active*/
//     if(sunyiValue == 2 && error0 == 1 || error1 == 1 || error2 == 1 || error3 == 1 || error4 == 1 || error5 == 1 || error6 == 1){
//         if(millis() - last_time1 > long_time*60000){
//             last_time1 = millis();
//             sunyiValue = 1;
//         }
//     }else{
//         last_time1 = 0;
//     }
//     /*end auto alarm active*/
}
/*End read Error data or missing sensor*/////////////////////////////////////////////////////////////////////////////////////

/*alarm status*/
void alarm(){
    if(sunyiValue == 1){
        if(millis() - lastTime3 > 500 && loopAlarm == 0){
            lastTime3 = millis();
            loopAlarm = 1;
            tone(pinBuzzer, 494);
        }
        if(millis() - lastTime3 > 500 && loopAlarm == 1){
            lastTime3 = millis();
            loopAlarm = 0;
            noTone(pinBuzzer);
        }
    }else{
        noTone(pinBuzzer);
    }
}
/*End alarm status*/

/*tone buzzer function*/
void doremi(){
    if(millis() - h > 500 && g == 0){
        tone(pinBuzzer, DO);
        h = millis();
        g = 1;
    }
    if(millis() - h > 500 && g == 1){
        tone(pinBuzzer, RE);
        h = millis();
        g = 0;
    }
}
