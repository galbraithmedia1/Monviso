#include <WiFi.h>
#include <FirebaseESP32.h>
#include "DHT.h"
#include <Adafruit_BMP085.h>
#include <MQUnifiedsensor.h>
 
#define FIREBASE_HOST "***"
#define FIREBASE_AUTH "***"




const char* WIFI_SSID  = "***";
const char* WIFI_PASSWORD = "***";



#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define         Pin                     (36)  //IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-135") //MQ3 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (5) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (10) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ135CleanAir        (3.6) // Ratio of your sensor, for this example an MQ-3

MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
Adafruit_BMP085 bmp;

#define DHTPIN 2   
#define DHTTYPE DHT22  
DHT dht(DHTPIN, DHTTYPE);

FirebaseData firebaseData;
FirebaseJson json1;
FirebaseJson json2;
FirebaseJson json3;

float humi = 0;
float temp = 0;

float bmpTemp = 0;
float Altitude = 0;
float Pressure = 0;

float CO = 0;
float Alcohol = 0;
float CO2 = 0;
float Toluen = 0;
float NH4 = 0;
float Aceton = 0;

void setup() {
 Serial.begin(9600); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  
  MQ135.init(); 
  
   dht.begin();
   
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");

//MQ135.setR0(calcR0/10);
//  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-135 ****");
  Serial.println("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |");  

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
 
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
 
 
  //String path = "/data";
  
 
  Serial.println("------------------------------------");
  Serial.println("Connected...");
  
}


void loop() {
 getDht();
 getBmpData();
 getMq135Data();
 sendData();
 delay(5000);
}
//
void getDht(){
   humi = dht.readHumidity();
   temp = dht.readTemperature(); 
}

void getBmpData(){
   bmpTemp = bmp.readTemperature();
   Pressure = bmp.readAltitude();
   Altitude = bmp.readSealevelPressure();
}

void getMq135Data(){
   MQ135.update();
   /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */
   MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
   CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
   Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
   CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
   Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
   NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
   Aceton = MQ135.readSensor(); 


    Serial.print("|   "); Serial.print(CO); 
  Serial.print("   |   "); Serial.print(Alcohol);
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  /*
  Motivation:
  We have added 400 PPM because when the library is calibrated it assumes the current state of the
  air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
  https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
  */
  Serial.print("   |   "); Serial.print(CO2 + 400); 
  Serial.print("   |   "); Serial.print(Toluen); 
  Serial.print("   |   "); Serial.print(NH4); 
  Serial.print("   |   "); Serial.print(Aceton);
  Serial.println("   |"); 
}

void sendData(){
    json1.set("/humi", humi);
      json1.set("/temp", temp);
   Firebase.updateNode(firebaseData,"/DHT22/Dhtdata",json1);
   
    
       json2.set("/temp", bmpTemp);
         json2.set("/Altitude", Altitude);
          json2.set("/Pressure", Pressure);
     
     Firebase.updateNode(firebaseData,"/BMP180/Bmpdata",json2);

      
     json3.set("/CO", CO);
        json3.set("/Alcohol", Alcohol);
         json3.set("/CO2", CO2);
          json3.set("/Toluen", Toluen);
            json3.set("/NH4", NH4);
             json3.set("/Aceton", Aceton);

      Firebase.updateNode(firebaseData,"/MQ-135/Mqdata",json3);
}
