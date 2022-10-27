#define dataOut 10
#define clkIn 9
#define sensor A0
#define sensor2 A1


int a;
//int c;
int v;
int dat;
int dat2;
char plan[40];
unsigned long low;
int lowvar;
unsigned long x =  0;
uint8_t y;
int b = 0;
unsigned long d;
uint8_t i;
int dataSensor1;
int dataSensor2;
uint8_t current = 0;
uint8_t last = 0;
int dataArray[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
int dataArray2[20] = {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0};
String dataArray3[20];

void setup(){
  Serial.begin(9600);
    pinMode(dataOut, OUTPUT);
    pinMode(clkIn, INPUT);
    pinMode(sensor, INPUT);
    pinMode(sensor2, INPUT);
    b = 0;
}

void loop(){
  send_data();
// dataSensor2 = analogRead(sensor);
// dataSensor2 = 923;
// dataSensor1 = analogRead(sensor2);
// dataSensor1 = 923;
}

void convert_tobinary(int input, int input2){
  dat = 0;
  for(a=512; a>=1; a=a/2){
    if((input-a)>=0){
      dataArray3[dat]= '1';
      input-=a; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
    for(int a2=512; a2>=1; a2=a2/2){
    if((input2-a2)>=0){
      dataArray3[dat]= '1';
      input2-=a2; 
    }
    else{
      dataArray3[dat]='0';
    } 
    dat++;  
  }
}


void send_data(){
  last = current;
  current = digitalRead(clkIn);
  if(last == 0 && current == 1){
//     last = 0;
     lowvar = 0;
     digitalWrite(dataOut, dataArray3[b].toInt());
//     digitalWrite(dataOut, b);
     Serial.print(dataArray3[b]);
     b++;
   }
  if(b > 20){
     b = 0;
     lowvar = 1;
     current = 0;
     last = 0;
     Serial.print(dataSensor2);
     Serial.print("-");
     Serial.print(dataSensor1);
     Serial.println(",");
     dataSensor2 = analogRead(sensor);
     dataSensor1 = analogRead(sensor2);
     convert_tobinary(dataSensor2, dataSensor1);
   }               
}
