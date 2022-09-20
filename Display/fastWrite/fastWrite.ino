#include <ArduinoJson.h>
#include "src/library/SimpleTimer/SimpleTimer.h"
#include "LedControl.h"

/*define*/
#define leftAir            23 //1
#define rightAir           24 //2
#define setAir             22 //3
#define leftHumi           25 //4
#define rightHumi          26 //5
#define setHumi            27 //6
#define leftSkin           28 //7
#define rightSkin          29 //8
#define setSkin            30 //9
#define setTimer           32 //10
#define bypassPin          47 //11... asalnya 31
#define set_alarm          32 //12
#define led_airway         33 //13
#define led_skin           34 //14
#define led_humi           35 //15
#define led_bypass         48//16
#define led_alarm          49//17
#define err0               50//18
#define err1               51//19
#define err2               52//20
#define err3               53//21
#define err4               54//22
#define err5               55//23
#define err6               56//24
#define err7               57//25
#define lock_btn           46 //26
#define led_lock           31 //27
#define alarmResetpin      48 //28

/*seven segment display define*/
//Pin 7 = DIN
//PIN 6 = ClK
//Pin 5 = CS/LOAD
//LedControl (DIN, CLK, CS/Load, Number of IC used)
LedControl lc = LedControl(7, 6, 5 ,3); //32
// 6 pin for 24 Seven segment
/*End Define*//////////////////////////////////////////////

int graphpin[] = {36, 37, 38, 39, 40, 41, 42, 43, 44, 45};

//display setpoint
float displaysetTemp = 27.0;
float displaysetSkin = 34.0;
float displaysetHumi = 60;
float displaysetTimer = 0;

//data from sensor
float chamberTemp0 = 0;
float humidityMid = 0;
float skinTemp1 = 0;
float skinTemp2 = 0; 

//next data dr sensor d inisialisasi ke
float displayTemp = 0;
float displayHumi = 0;
float displaySkin = 0;
float displaySkin2 = 0;

//data setpoint sended to incubator
float sendTemp = 0;
float sendHumi = 0;
float sendSkin = 0;

//data Mode sended to incubator
int skinMode = 0;
int highTemp = 0;
int humiMode = 0;
int lockMode = 0;
int sunyiMode = 0;

//data error received from incubator
int error0 = 0;
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;
int error7 = 0;
int error8 = 0;

//valuePower
int valuePower = 1;
static char inputData[256];
int x;
int a = 0;
float b;
int c = 0;
float d;
int e = 0;
float f;
int g = 0;
float h;
int i = 0;
float j;
uint8_t k = 0;
uint8_t l = 0;
int debugging = 1;
uint8_t segmentBlank = 0;


//data button
uint8_t lastPower0;   //tombol kiri airtemp      
uint8_t lastPower1;   //tombol kanan airtemp      
uint8_t lastPower2;   //tombol kiri skintemp1      
uint8_t lastPower3;   //tombol kanan skintemp1
uint8_t lastPower4;   //tombol kiri humi      
uint8_t lastPower5;   //tombol kanan humi   
uint8_t lastPower6;   //tombol setAirTemp     
uint8_t lastPower7;   //tombol setSkinTemp    
uint8_t lastPower8;   //tombol setHumidity
uint8_t lastPower9;   //tombol hightemp  
uint8_t lastPower10;  //Lock
uint8_t lastPower11;  //Alarm

uint8_t currentPower0;   //read tombol kiri airtemp   
uint8_t currentPower1;   //read tombol kanan airtemp   
uint8_t currentPower2;   //read tombol kiri set skin temp1  
uint8_t currentPower3;   //read tombol kanan set skin temp1
uint8_t currentPower4;   //read tombol kiri set humi 
uint8_t currentPower5;   //read tombol kanan set humi
uint8_t currentPower6;   //read setAirTemp   
uint8_t currentPower7;   //read setSkinTemp   
uint8_t currentPower8;   //read setHumidy
uint8_t currentPower9;   //read hightemp
uint8_t currentPower10;  //Lock
uint8_t currentPower11;  //Alarm
uint8_t onOff;

uint8_t setHigh = 0;
uint8_t setAirway = 0;
uint8_t setSkinMode = 0;
uint8_t setHumiMode = 0;
uint8_t setLockMode = 0;
uint8_t setAlarmMode = 0;

//initialize digit
int digit1 = 0;
int digit2 = 0;
int digit3 = 0;
int digit4 = 0;
int digit5 = 0;
int digit6 = 0;
int digit7 = 0;
int digit8 = 0;
int digit9 = 0;
int digit10 = 0;
int digit11 = 0;
int digit12 = 0;
int digit13 = 0;
int digit14 = 0;
int digit15 = 0;
int digit16 = 0;
int digit17 = 0;
int digit18 = 0;
int digit19 = 0;
int digit20 = 0;
int digit21 = 0;
int digit22 = 0;
int digit23 = 0;
int digit24 = 0;
int digit25 = 0;
uint8_t heaterPwm = 0;
uint8_t heatedPower = 0;
String pin;

SimpleTimer timer0;
SimpleTimer timer1;

//array to store digit
// const int digit[] = {pin_digit1, pin_digit2, pin_digit3, pin_digit4, pin_digit5, pin_digit6};

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  timer0.setInterval(1000, generate_json);
  lc.setIntensity(0, 2);
  lc.setIntensity(1, 2);
  lc.setIntensity(2, 2);
  lc.clearDisplay(0);
  lc.clearDisplay(1);
  lc.clearDisplay(2);
  
  /*button Pin*/
  pinMode(leftAir, INPUT);
  pinMode(rightAir, INPUT);
  pinMode(setAir, INPUT);
  pinMode(leftSkin, INPUT);
  pinMode(rightSkin, INPUT);
  pinMode(setSkin, INPUT);
  pinMode(leftHumi, INPUT);
  pinMode(rightHumi, INPUT);
  pinMode(setHumi, INPUT);
  pinMode(bypassPin, INPUT);
  pinMode(setTimer, INPUT);
  pinMode(set_alarm, INPUT);
  
  /*LED Pin*/
  pinMode(led_airway, OUTPUT);
  pinMode(led_skin, OUTPUT);
  pinMode(led_humi, OUTPUT);
  pinMode(led_bypass, OUTPUT);
  pinMode(led_alarm, OUTPUT);
  pinMode(led_lock, OUTPUT);

  /*led graph pin setup*/
  for(int thisled = 0; thisled < 10; thisled++){
    pinMode(graphpin[thisled], OUTPUT);
  }
  // pinMode(hp1, OUTPUT);
  // pinMode(hp2, OUTPUT);
  // pinMode(hp3, OUTPUT);
  // pinMode(hp4, OUTPUT);
  // pinMode(hp5, OUTPUT);
  // pinMode(hp6, OUTPUT);
  // pinMode(hp7, OUTPUT);
  // pinMode(hp8, OUTPUT);
  // pinMode(hp9, OUTPUT);
  // pinMode(hp10, OUTPUT);
}

void loop() {
   run_program();
}

void run_program(){
  getData();
  if(onOff == 1){
    timer0.run();//generate_json();
    btn_menu();
    set_btn();
    display_digit();
  }
  if(onOff == 0){
    digit_kosong();
  } 
}

/* Send and Receive Data Session*///// 
void generate_json(){
    StaticJsonDocument<255> out1;
    JsonObject DataButton = out1.createNestedObject("data1");
    DataButton["sn"][0] = sendTemp;
    DataButton["sn"][1] = sendSkin;
    DataButton["sn"][1] = sendHumi;
    DataButton["mode"][0] = skinMode;
    DataButton["mode"][1] = humiMode;
    DataButton["mode"][2] = highTemp;
    DataButton["mode"][3] = lockMode;
    serializeJson(out1, Serial1);
    Serial1.println();
//     serializeJson(out1, Serial);
//     Serial.println();    
}

void getData(){
  while(Serial1.available()>0){
    inputData[x] = Serial1.read();
    x++;
    if(inputData[x-1] == '\n'){
      Serial.println(inputData);
      StaticJsonDocument<512>in;
      DeserializationError error = deserializeJson(in, inputData);
      x = 0;
      
      if(!error){
       chamberTemp0 = in["suhu"][0];
       skinTemp1 =    in["suhu"][1];
       skinTemp2 =    in["suhu"][2];
       humidityMid =  in["suhu"][3];
       onOff =        in["btn"][0];
       heaterPwm =    in["pow"][0];
      //  error0 = in["err"][0];
      //  error1 = in["err"][1];
      //  error2 = in["err"][2];
      //  error3 = in["err"][3];
      //  error4 = in["err"][4];
      //  error5 = in["err"][5];
      //  error6 = in["err"][6];
      //  error7 = in["err"][7];
       return;
      }
      // x = 0;
 //      chamberTemp0 = 0;
 //      humidityMid =0;
 //      skinTemp1 = 0;
 //      skinTemp2 = 0;
 //      error0 = 0;
 //      error1 = 0;
 //      error2 = 0;
 //      error3 = 0;
 //      error4 = 0;
 //      error5 = 0;
 //      error6 = 0;
 //      error7 = 0;      
    }
  }
}
/* END Send and Receive Data Session*///// 


/*increment-Decrement function*/
void btn_menu(){
  /*kontrol set Humi*/
  if(setHumiMode == 1 && lockMode == 0){
   lastPower4 = currentPower4;
   currentPower4 = digitalRead(rightHumi);
   if(lastPower4 == HIGH && currentPower4 == LOW){
      displaysetHumi = displaysetHumi + 1;
      if(displaysetHumi > 95){
        displaysetHumi = 95;
      }
   }
   lastPower5 = currentPower5;
   currentPower5 = digitalRead(leftHumi);
   if(lastPower5 == HIGH && currentPower5 == LOW){
      displaysetHumi = displaysetHumi - 1;
      if(displaysetHumi < 30){
        displaysetHumi = 30;
      }
   }
  }

  /*Kontrol set airtemp*/
   if(setAirway == 1 && lockMode == 0){
        lastPower0 = currentPower0;
        currentPower0 = digitalRead(leftAir); //fungsi btn kiri set airtemp
        if(lastPower0 == HIGH && currentPower0 == LOW){
//            timeBtn0 = 0;
            displaysetTemp = displaysetTemp - 0.1;
           if(highTemp == 1 ){
              if(displaysetTemp <= 37){
               displaysetTemp = 37;
              }
           }
           if(highTemp == 0){
               if(displaysetTemp <= 20){
                   displaysetTemp = 20;
               }
           }
        }
        lastPower1 = currentPower1;
        currentPower1 = digitalRead(rightAir);
        if(lastPower1 == HIGH && currentPower1 == LOW){
            displaysetTemp = displaysetTemp + 0.1;
           if(highTemp == 1){
               if(displaysetTemp >= 39){
                   displaysetTemp = 39;
               }
           }
           if(highTemp == 0){
                   if(displaysetTemp >= 37){
                       displaysetTemp = 37;
                   }
           }
         }
      }

    if(setSkinMode == 1 && lockMode == 0){
      // segmentBlank = 1;
      /*kontrol Skin Temp*/
        lastPower2 = currentPower2;
        currentPower2 = digitalRead(rightSkin);
        if(lastPower2 == HIGH && currentPower2 == LOW){
//            timeBtn0 = 0;
            displaysetSkin = displaysetSkin + 0.1;
           if(highTemp == 1){
               if(displaysetSkin >= 38){
                   displaysetSkin = 38;
               }
           }
           if(highTemp == 0){
                   if(displaysetSkin >= 37){
                       displaysetSkin = 37;
                   }
               }
            }

        lastPower3 = currentPower3;
        currentPower3 = digitalRead(leftSkin);
        if(lastPower3 == HIGH && currentPower3 == LOW){
            displaysetSkin = displaysetSkin - 0.1;
           if(highTemp == 1){
               if(displaysetSkin <= 37){
                   displaysetSkin = 37;
               }
           }
           if(highTemp == 0){
                   if(displaysetSkin <= 34){
                       displaysetSkin = 34;
                   }
               }
         }       
    }
}


/*Mode Set Function*/
void set_btn(){
/*Set Lock Mode*/
  lastPower10 = currentPower10;
    currentPower10 = digitalRead(lock_btn);
    if(lastPower10 == HIGH && currentPower10 == LOW){
      setLockMode = setLockMode + 1;
      if(setLockMode == 1){
        lockMode = 1;
        digitalWrite(led_lock, HIGH);
      }

      if(setLockMode == 2){
        lockMode = 0;
        setLockMode = 0;
        digitalWrite(led_lock, LOW);
        if(setLockMode > 1){
          setLockMode = 0;
        }
      } 
   }

    if(lockMode == 0 && skinMode <= 0){

      lastPower9 = currentPower9;
      currentPower9 = digitalRead(bypassPin);
      if(lastPower9 == HIGH && currentPower9 == LOW){
          setHigh = setHigh + 1 ;
          if(setHigh == 1){
             highTemp = 1;
              digitalWrite(led_bypass, HIGH);
              displaysetTemp = 37;
              displaysetSkin = 37;
          }
          if(setHigh == 2){
              highTemp = 0;
              setHigh = 0;
              digitalWrite(led_bypass, LOW);
              displaysetTemp = 27;
            displaysetSkin = 34;
          }
      }
    } 


  if(lockMode == 0){
    lastPower6 = currentPower6;
    currentPower6 = digitalRead(setAir);
    if(lastPower6 == HIGH && currentPower6 == LOW){
      setAirway = setAirway + 1;
      if(setAirway == 1){
        // skinMode = 2;
        setSkinMode = 0;
        segmentBlank = 2;
        digitalWrite(led_skin, LOW);
        if(setSkinMode > 0){
        setSkinMode = 0;
        }
      }
      /*Send data set airway to incubator*/
      if(setAirway == 2){ 
        sendTemp = displaysetTemp;
        setAirway = 2;
        skinMode = 2;
        segmentBlank = 2;
        setSkinMode = 0;
        digitalWrite(led_airway, HIGH);
        digitalWrite(led_skin, LOW);
        if(setSkinMode > 0){
          setSkinMode = 0;
        }
      }
      if(setAirway == 3){
        setAirway = 0;
        setSkinMode = 0;
        skinMode = 0;
        segmentBlank = 0;
        digitalWrite(led_airway, LOW);
        if(setSkinMode > 0){
          setSkinMode = 0;
        }
    }
    } 

    lastPower7 = currentPower7;
    currentPower7 = digitalRead(setSkin);
    if(lastPower7 == HIGH && currentPower7 == LOW){
      setSkinMode = setSkinMode + 1;
      if(setSkinMode == 1){
        // skinMode = 1;
        setAirway = 0;
        segmentBlank = 1;
        digitalWrite(led_airway, LOW);
        if(setAirway > 0){
        setAirway = 0;
        } 
      }
      /*Send data set skin to incubator*/
      if(setSkinMode == 2){
        sendTemp = displaysetSkin;
        setSkinMode = 2;
        setAirway = 0;
        skinMode = 1;
        segmentBlank = 1;
        digitalWrite(led_skin, HIGH);
        digitalWrite(led_airway, LOW);
        if(setAirway > 0){
          setAirway = 0;
        }
      }
      if(setSkinMode == 3){
        setSkinMode = 0;
        setAirway = 0;
        skinMode = 0;
        segmentBlank = 0;
        digitalWrite(led_skin, LOW);
        if(setAirway > 0){
          setAirway = 0;
        }
      } 
    }

   /*Set Humi Mode*/
    lastPower8 = currentPower8;
    currentPower8 = digitalRead(setHumi);
    if(lastPower8 == HIGH && currentPower8 == LOW){
      setHumiMode = setHumiMode + 1;
      if(setHumiMode == 1){
        // humiMode = 1;
      }
            /*Send data set humidity to incubator*/
      if(setHumiMode == 2){
        sendHumi = displaysetHumi;
        digitalWrite(led_humi, HIGH);
        humiMode = 1;
      } 
      if(setHumiMode == 3){
        setHumiMode = 0;
        humiMode = 0;
        digitalWrite(led_humi, LOW);
      }
    }
 }



 /*Set Alarm Mode*/
  lastPower11 = currentPower11;
    currentPower11 = digitalRead(set_alarm);
    if(lastPower11 == HIGH && currentPower11 == LOW){
      setAlarmMode = setAlarmMode + 1;
      if(setAlarmMode == 1){
        sunyiMode = 1;
        digitalWrite(led_alarm, HIGH);
      }

      if(setAlarmMode == 2){
        sunyiMode = 0;
        setAlarmMode = 0;
        digitalWrite(led_alarm, LOW);
        if(setAlarmMode > 1){
          setAlarmMode = 0;
        }
      } 
   }   
  

      /*LED Blinking while setpoint changed*/
        if(setSkinMode == 0 && setAirway == 1){
          if (millis() - b > 500 && a == 0) {
            digitalWrite(led_airway, HIGH);
            b = millis();
            a = 1;
          }
          if (millis() - b > 500 && a == 1) {
            digitalWrite(led_airway, LOW);
            b = millis();
            a = 0;
          }   
        } 

        if(setSkinMode == 1 && setAirway == 0){
          if (millis() - d > 500 && c == 0) {
            digitalWrite(led_skin, HIGH);
            d = millis();
            c = 1;
          }
          if (millis() - d > 500 && c == 1) {
            digitalWrite(led_skin, LOW);
            d = millis();
            c = 0;
          }   
        } 

        if(setHumiMode == 1 && humiMode == 0){
          if (millis() - f > 500 && e == 0) {
            digitalWrite(led_humi, HIGH);
            f = millis();
            e = 1;
          }
          if (millis() - f > 500 && e == 1) {
            digitalWrite(led_humi, LOW);
            f = millis();
            e = 0;
          }   
        }
 }
 
    
 


 void read_error(){
  if(error0 == 1){
    digitalWrite(err0, HIGH);
  }
  if(error0 == 0){
    digitalWrite(err0, LOW);
  }
  if(error1 == 1){
    digitalWrite(err1, HIGH);
  }
  if(error1 == 0){
    digitalWrite(err1, LOW);
  }
  if(error2 == 1){
    digitalWrite(err2, HIGH);
  }
  if(error2 == 0){
    digitalWrite(err2, LOW);
  }
  if(error3 == 1){
    digitalWrite(err3, HIGH);
  }
  if(error3 == 0){
    digitalWrite(err3, LOW);
  }
  if(error4 == 1){
    digitalWrite(err4, HIGH);
  }
  if(error4 == 0){
    digitalWrite(err4, LOW);
  }
  if(error5 == 1){
    digitalWrite(err5, HIGH);
  }
  if(error5 == 0){
    digitalWrite(err5, LOW);
  }
  if(error6 == 1){
    digitalWrite(err6, HIGH);
  }
  if(error6 == 0){
    digitalWrite(err6, LOW);
  }
  if(error7 == 1){
    digitalWrite(err7, HIGH);
  }
  if(error7 == 0){
    digitalWrite(err7, LOW);
  }
}

void display_digit(){
  lc.shutdown(0, false);
  lc.shutdown(1, false);
  lc.shutdown(2, false);
  nilaidigit();

   /*Display Airway Temp*/
  lc.setDigit(2, 0, digit4, false);
  lc.setDigit(2, 1, digit5, true);
  lc.setDigit(2, 2, digit6, false);

  // lc.setDigit(2, 3, digit1, false);
  // lc.setDigit(2, 4, digit2, true);
  // lc.setDigit(2, 5, digit3, false);

  // /*Display Skin Temp*/
  //  lc.setDigit(1, 7, digit7, false);
  //  lc.setDigit(1, 6, digit8, true);
  //  lc.setDigit(1, 1, digit9, false);

   lc.setDigit(1, 3, digit10, false);
   lc.setDigit(1, 4, digit11, true);
   lc.setDigit(1, 5, digit12, false);
   
   /*Display Humidity*/
   lc.setDigit(0, 0, digit16, false);
   lc.setDigit(0, 1, digit17, false);
   lc.setDigit(0, 3, digit18, false);
   lc.setDigit(0, 2, digit19, false);

     /*Display Timer*/
    // lc.setDigit(2, 4, digit20, false);
    // lc.setDigit(2, 5, digit21, false);
    // lc.setDigit(2, 6, digit22, false);
    // lc.setDigit(2, 7, digit23, false);

  /*Display Led Bar Graph*/
    heatedPower = map(heaterPwm, 0, 255, 0, 10);
    for(int thisled = 0; thisled < 10; thisled++){
      if(thisled < heatedPower){
        digitalWrite(graphpin[thisled], HIGH);
      }
      else{
        digitalWrite(graphpin[thisled], LOW);
      }
    }
    // for(k = 0; k < 10; k++){
    //   digitalWrite(pin.toInt(), HIGH);
    // }
    // for(k = 10; k > heatedPower; k--){
    //   digitalWrite(pin.toInt(), LOW);
    // }
if(segmentBlank == 0){
  lc.setDigit(2, 3, digit1, false);
  lc.setDigit(2, 4, digit2, true);
  lc.setDigit(2, 5, digit3, false);
  lc.setDigit(1, 7, digit7, false);
  lc.setDigit(1, 6, digit8, true);
  lc.setDigit(1, 1, digit9, false);
  }   

  if(segmentBlank == 1){
    /*not showing airtemp number when skinmode =1*/
    lc.setChar(2, 3, '-', false);
    lc.setChar(2, 4, '-', false);
    lc.setChar(2, 5, '-', false);
    lc.setDigit(1, 7, digit7, false);
    lc.setDigit(1, 6, digit8, true);
    lc.setDigit(1, 1, digit9, false);
    /*end*/
  }

  if(segmentBlank == 2){
    /*not showing skintemp number when skinmode =2*/ 
    lc.setChar(1, 7, '-', false);
    lc.setChar(1, 6, '-', false);
    lc.setChar(1, 1, '-', false);
    lc.setDigit(2, 3, digit1, false);
    lc.setDigit(2, 4, digit2, true);
    lc.setDigit(2, 5, digit3, false);
  }
}

void digit_kosong(){
  lc.shutdown(0, true);
  lc.shutdown(1, true);
  lc.shutdown(2, true);
  nilaidigit();
    /*Display Airway Temp*/
  lc.setChar(2, 'blank', digit4, false);
  lc.setChar(2, 'blank', digit5, true);
  lc.setChar(2, 'blank', digit6, false);

  lc.setChar(2, 'blank', digit1, false);
  lc.setChar(2, 'blank', digit2, true);
  lc.setChar(2, 'blank', digit3, false);

  /*Display Skin Temp*/
   lc.setChar(1, 'blank', digit7, false);
   lc.setChar(1, 'blank', digit8, true);
   lc.setChar(1, 'blank', digit9, false);

   lc.setChar(1, 'blank', digit10, false);
   lc.setChar(1, 'blank', digit11, true);
   lc.setChar(1, 'blank', digit12, false);
   
   /*Display Humidity*/
   lc.setChar(0, 'blank', digit16, false);
   lc.setChar(0, 'blank', digit17, false);
   lc.setChar(0, 'blank', digit18, false);
   lc.setChar(0, 'blank', digit19, false);
}

void nilaidigit() {
  float displayTemp = chamberTemp0;
  float displaySkin = skinTemp1;
  float displaySkin2 = skinTemp2;
  float displayHumi = humidityMid;

  /*airway*/
  digit1 = displaysetTemp / 10;
  digit2 = displaysetTemp - (digit1 * 10);
  digit3 = (displaysetTemp * 10 - (digit1 * 100)) - (digit2 * 10);
  digit4 = displayTemp / 10;
  digit5 = displayTemp - (digit4 * 10);
  digit6 = ((displayTemp * 10) - (digit4 * 100)) - (digit5 * 10);
  
  /*skin*/
  digit7 = displaysetSkin / 10;
  digit8 = displaysetSkin - (digit7 * 10);
  digit9 = (displaysetSkin * 10 - (digit7 * 100)) - (digit8 * 10);
  digit10 = displaySkin / 10;
  digit11 = displaySkin - (digit10 * 10);
  digit12 = ((displaySkin * 10) - (digit10 * 100)) - (digit11 * 10);
  digit13 = displaySkin2 / 10;
  digit14 = displaySkin2 - (digit13 * 10);
  digit15 = (displaySkin2 * 10 - (digit13 * 100)) - (digit14 * 10);
  
  /*humidity*/
  digit16 = displayHumi / 10;
  digit17 = displayHumi - (digit16 * 10);
  digit18 = displaysetHumi / 10;
  digit19 = displaysetHumi - (digit18 * 10);

  /*timer*/
  digit20 = displaysetTimer / 1000;
  digit21 = ((displaysetTimer - (digit20 * 1000)) / 100);
  digit22 = (((displaysetTimer - (digit20 * 1000)) - (digit21 * 100)) / 10);
  digit23 = ((((displaysetTimer - (digit20 * 1000)) - digit21 * 100) - digit22 * 10));
  
}

