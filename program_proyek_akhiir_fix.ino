 #include<KRwifi.h>
 char* ssid         = "vivo-2007";
 char* pass         = "12345678";
 char* server       = "api.thingspeak.com";
 char* APIKey       = "T9JWGGL81HXRYVJU";
 char* mychannelNumber = "2228069";
//Millis
uint32_t periodeKirim   = 20000;
uint32_t millisKirim;


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
//byte degre = B11011111;


#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//------------------------------Ammonia
#define RL 47 //nilai RL =10 pada sensor
#define m -0.47862 //hasil perhitungan gradien
#define b 0.9571 //hasil perhitungan perpotongan
#define Ro 33 //hasil pengukuran RO
#define MQ_sensor A0 //definisi variabel
const int numReadings = 5;//nilai penambilan sample pembacaan sebesar 5 kali
float readings[numReadings];
int readIndex = 0;
float total = 0;
float average = 0;

//-------------------------------TDS
#define TdsSensorPin A2
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

//-------------------------------pH
#define SensorPin A1            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.72            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40     //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

float pHValue = 0, sensorTemp, sensorAmonia, averageVoltage = 0 ,tdsValue = 0, do2, tbd;

void setup() {
  pinMode(LED,OUTPUT);
  Serial.begin(9600);
  Serial1.begin(115200);         
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("++++++++++++++++++++");
  lcd.setCursor(0, 1);
  lcd.print("+    BISMILLAH     +");
  lcd.setCursor(0, 2);
  lcd.print("+     SIMPONIC     +");
  lcd.setCursor(0, 3);
  lcd.print("++++++++++++++++++++");
  delay(4000);
  lcd.clear();
  
  sensors.begin(); //sensor suhu
  pinMode(TdsSensorPin,INPUT); //sensor TDS
  setWifi(ssid, pass);
  millisKirim = millis();

  //------------------------------LCD

}

void loop() {
  //pH();
  Ammonia();
  TDS();
  
//------------------------------Sensor suhu
  sensors.requestTemperatures();
  int tempC = sensors.getTempCByIndex(0);
  sensorTemp = (tempC);
  //-----------------------------------------
  int do2 = 8;
  int tbd = 13;
  
//-----------------------------------------
  lcd.setCursor(0, 0);
  lcd.print("----- SIMPONIC -----");
//----------------------  
  lcd.setCursor(0, 1);
  lcd.print(" SH:");
  lcd.print(tempC);

  //------------------
  lcd.setCursor(11, 1);
  lcd.print("Am :");
  lcd.print(average);
  lcd.print("");
  //------------------
  lcd.setCursor(0, 2);
  lcd.print(" Ph:");
  lcd.print(pHValue,2);
  
  //------------------
  lcd.setCursor(11, 2);
  lcd.print("TDS:");
  lcd.print(tdsValue,0);
  //------------------
    lcd.setCursor(0, 3);
    lcd.print("--------------------");
  
  if (millisKirim < millis()) {
    millisKirim = millis() + periodeKirim;
    //sensorTemp++;
    //average++;
    
    String konten;
    konten = String() + "field1=" + sensorTemp +  "&field2=" + average +  "&field3=" + tdsValue + "&field5=" + do2;
    httpPOST(server, APIKey, konten, 50, 80);
    Serial.print("Respon: ");
    Serial.println(getData);

 //--------------------------- Data Sensor Serial Monitor
    Serial.print(sensorTemp);
    Serial.println(" Celcius");
    Serial.print(average);
    Serial.println(" ppm");
    Serial.print(tdsValue,0);
  //Serial.println(" ppm");
 //-----------------------------------------
  }
  statusPengiriman();

 
 //---------------------------pH
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    //Serial.print("Voltage:");
        //Serial.print(voltage,2);
        //Serial.print("    pH value: ");
        Serial.println(pHValue,2);
        digitalWrite(LED,digitalRead(LED)^1);
        printTime=millis();
        //-----------------
        lcd.setCursor(0, 2);
        lcd.print(" Ph:");
        lcd.print(pHValue,2);
        String konten;
        konten = String() + "&field4=" + pHValue,2;
        httpPOST(server, APIKey, konten, 50, 80);
  }
}
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
//----------------------------------------------
  
}

void Temp() {
  sensors.requestTemperatures();
  int tempC = sensors.getTempCByIndex(0);
}

void Ammonia() {
  float VRL;
  float RS;
  float ratio;

  VRL = analogRead(MQ_sensor) * (5 / 1023.0); //konversi analog ke tegangan
  RS = ((5.0 * RL) / VRL) - RL ; //rumus untuk RS
  ratio = RS / Ro; // rumus mencari ratio
  float ppm = pow(10, ((log10(ratio) - b) / m)); //rumus mencari ppm

  total = total - readings[readIndex];

  readings[readIndex] = ppm;

  total = total + readings[readIndex];

  readIndex = readIndex + 1;


  if (readIndex >= numReadings) {

    readIndex = 0;
  }

  average = total / numReadings;
}

void TDS(){
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(sensorTemp-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
