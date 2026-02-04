//simple framework for now, no objects or anything complicated, just logic and stuff
#include <Arduino_LSM6DS3.h>  //chippy doodah
#include <SPI.h>             //library for SPI communication
#include <SD.h>              //library to run the SD card
#include <String.h>

//pin related constants
#define SCK 13         //clock pin number for SPI
#define MISO 12        //CIPO pin number for SPI
#define MOSI 11        //COPI pin number for SPI
#define BMP_CS 8          //chip select pin for BMP
#define CS_MPU 9           //chip select pin for ICM
const int CS_SD = 10;            //chip select pin for SD
const int LED0 = 7; // SD and file (off->SD error, pulsing->file error) green
const int LED1 = 6; // accelerometer status red-good, pulsing-accelerometer bad, off-IMU bad 
const int LED2 = 5;
const int LED3 = 4; // data collecting on yellow
const int BUTTON = 3;
const int LAUNCHBUTTON = 2; //button to launch rocket for test
File file;
String fileBase, fileExtension, fileTempS;
char fileTemp[100];
int i = 1;

float xA, yA, zA, xG, yG, zG; // acceleration and gyroscope values
float xAO, yAO, zAO, xGO, yGO, zGO = 0; // offsets

void setupPins(){
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LAUNCHBUTTON, INPUT);
}

void setupBloodyStrings(){
  fileBase = String("data");
  fileExtension = String(".txt");
  fileTempS = String();
  fileTempS = fileBase + fileExtension;
  fileTempS.toCharArray(fileTemp, 100);
}

void setupSD(){
  if (! SD.begin(CS_SD)){
    Serial.println("Error with the SD");
    while(1){
      digitalWrite(LED0, LOW);
    }
  } else {
    digitalWrite(LED0, HIGH);
  }

  delay(1000);
}

void setupFile(){
  if(SD.exists(fileTemp) == 0){
    file = SD.open(fileTemp, FILE_WRITE);
    if(!file){
      Serial.println("file creation error at start");
    }
    file.close();
  } else {
    fileTempS = fileBase + i + fileExtension;
    fileTempS.toCharArray(fileTemp, 100);
    while(SD.exists(fileTemp) == 1){
      i++;
      fileTempS = fileBase + i + fileExtension;
      fileTempS.toCharArray(fileTemp, 100);
    }
    Serial.print("new file created: ");
    Serial.println(fileTemp);
    file = SD.open(fileTemp, FILE_WRITE);
    if(!file){
      Serial.println("file creation error at numbering");
    }
    file.close();
  }
}

void setupChips(){
  if(!IMU.begin()){
    Serial.println("IMU failed to start");
    while(1){
      digitalWrite(LED1, LOW);
    }
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  if(!IMU.accelerationAvailable()){
    Serial.println("error with the accelerometer");
    while(1){
      digitalWrite(LED1, HIGH);
      delay(100);
      digitalWrite(LED1, LOW);
      delay(100);
    }
  } else {
    Serial.println("accelerometer good");
    digitalWrite(LED1, HIGH);
  }
}

void calculateOffsets(){
  
  delay(1000);
  IMU.readAcceleration(xA, yA, zA);
  IMU.readGyroscope(xG, yG, zG);
  xAO = 0 - xA;
  yAO = 0 - yA;
  zAO = 0 - zA;
  xGO = 0 - xG;
  yGO = 0 - yG;
  zGO = 0 - zG;

}

void readChips(){
  IMU.readAcceleration(xA, yA, zA);
  IMU.readGyroscope(xG, yG, zG);
  xA -= xAO;
  yA -= yAO;
  zA -= zAO;
  xG -= xGO;
  yG -= yGO;
  zG -= zGO;
}

//tick stuff
const int TICK_LENGTH = 300000; //tick length of the system clock in microseconds
unsigned int timeLast;
int uTickLength;
void tick(){
  uTickLength = (timeLast + TICK_LENGTH - micros() - 15000);
  delayMicroseconds(uTickLength);
  delayMicroseconds(15000);
  timeLast = micros();
}


void setup() {
  setupPins();
  delay(2000);
  Serial.begin(115200);
  Serial.println("start");
  delay(1000);
  setupChips();
  //calculateOffsets();
  setupSD();
  setupBloodyStrings();
  setupFile();
  timeLast = micros();
}

bool collecting = true;
bool test = false;

void loop() {
  digitalWrite(LED2, HIGH);
  //Step 1: collect data (if on) variables added: collecting (bool)
  readChips();
  if(collecting == true){
    if(digitalRead(BUTTON) == HIGH){
      collecting = false;
    }
  }

  //digitalWrite(LED2, digitalRead(LAUNCHBUTTON));

  
  //Step 2: update integrated values (angle, position(?))

  //Step 3: store data
  if(collecting){
    file = SD.open(fileTemp, FILE_WRITE);
    // if(!Telemetry){
    // Serial.println("Error with the file");
    // while(1){
    //   digitalWrite(LED0, HIGH);
    //   delay(100);
    //   digitalWrite(LED0, LOW);
    //   delay(100);
    // }
  // }
    file.print(xA);
    file.print(",");
    file.print(yA);
    file.print(",");
    file.print(zA);
    file.print(",");
    file.print(xG);
    file.print(",");
    file.print(yG);
    file.print(",");
    file.print(zG);
    file.print(",");
    file.println(millis());
    file.close();
    
    digitalWrite(LED3, HIGH);
  } else {
    digitalWrite(LED3, LOW);
  }
  digitalWrite(LED2, LOW);
  //Step 4: update tickers

  //Step 5: run switch-case statement thingy

  //Step 6: tick clock
  tick();

}
