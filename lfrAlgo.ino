#include <SoftwareSerial.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_I2C_ADDRESS 0x3C
#define OLED_RESET_PIN -1


#include <Wire.h>
#include <U8g2lib.h>

#define UP_BUTTON 8
#define DOWN_BUTTON 
#define SELECT_BUTTON 7 
#define MUX_SIG A7
#define S0 A0
#define S1 A2 
#define S2 A6 
#define led 6
//oledpins 
//bluetooth pins 
//potentiometer

#define pot A3
#define IN1 4 //ma
#define IN2 11
#define IN3 5 //mb
#define IN4 9
#define ENA 3   //ma
#define ENB 11  //mb

float Kp = 2.0, Ki = 0.1, Kd = 1.0;
float integral = 0, previousError = 0;
int threshold = 600;
int speedBase = 200;

char intersection[];
char path[];
bool mid=false;
bool left=false;
bool front=false;
bool right=false;
bool object=false;
bool deadend=false;
int array[8];



// oled menu
// motor test 
// pid folowcalibration 
// threshold 
// dry run 
// actual menu 
// ir values
void lineSection(){
    //add deadend too
}


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const char* menuItems[] = {
    "PID Tuning",
    "Callibration",
    "Threshold",
    "Dry Run",
    "Actual Run"
};
int menuIndex = 0;
int totalItems = sizeof(menuItems) / sizeof(menuItems[0]);


void drawMenu() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x13_tr);
    
    for (int i = 0; i < totalItems; i++) {
        if (i == menuIndex) {
            u8g2.drawFrame(5, 10 + (i * 15), 117, 13); // Highlight selected item
        }
        u8g2.drawStr(20, 20 + (i * 15), menuItems[i]);
    }
    
    u8g2.sendBuffer();
}

void handleSelection(int index) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.drawStr(20, 30, "Selected:");
    u8g2.drawStr(20, 50, menuItems[index]);
    u8g2.sendBuffer();
    delay(1000);
}


void setup() {
    Serial.begin(9600);
    

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);


    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    u8g2.begin();
    
    pinMode(UP_BUTTON, INPUT_PULLUP);
    pinMode(DOWN_BUTTON, INPUT_PULLUP);
    pinMode(SELECT_BUTTON, INPUT_PULLUP);
}


void selectChannel(int channel) {
    digitalWrite(S0, channel & 1);
    digitalWrite(S1, (channel >> 1) & 1);
    digitalWrite(S2, (channel >> 2) & 1);
}


int readSensor(int channel) {
    selectChannel(channel);
    delay(5);  
    return analogRead(MUX_SIG);
}

void uTurn() {
        
    analogWrite(ENA, speedBase);
    analogWrite(ENB, speedBase);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(50);
    while(true){
        if(readSensor(5)>threshold ||readSensor(6)>threshold || readSensor(3) >threshold || readSensor(4) >threshold){
            break;
        }
        delay(10);
    }
    stopMotors();
    
}
void moveForward() {
    analogWrite(ENA, speedBase);
    analogWrite(ENB, speedBase);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void turnLeft() {
    analogWrite(ENA, speedBase);
    analogWrite(ENB, speedBase);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    delay(50);
    while (true) {
        if((readSensor(3) > threshold || readSensor(4) > threshold)){
            break;
        }
        delay(10);
    }
    stopMotors();
}

void turnRight() {
    analogWrite(ENA, speedBase);
    analogWrite(ENB, speedBase);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    delay(50);
    while(true){
        if(readSensor(5)>threshold ||readSensor(6)>threshold){
            break;
        }
        delay(10);
    }
    stopMotors();
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}



float calculateError() {
    float x = 1.0, y = 0.5, z = 0.5; 
    float e = x * (readSensor(5) - readSensor(4)) 
            + y * (readSensor(6) - readSensor(3)) 
            + z * (readSensor(7) - readSensor(2));

    return e;
}

void applyPID() {
    float error = calculateError();

    float P = error;
    integral += error;
    integral = constrain(integral, -50, 50);

    float D = error - previousError;
    float pidValue = (Kp * P) + (Ki * integral) + (Kd * D);
    previousError = error;

    int leftSpeed = constrain(speedBase + pidValue, 0, 255);
    int rightSpeed = constrain(speedBase - pidValue, 0, 255);

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void pidTuningMode() {
    while (true) {
        if (digitalRead(8) == LOW) {
            break;
        }

        int error = calculateError();
        float P = error;
        integral += error;
        integral = constrain(integral, -50, 50);
        float D = error - previousError;
        float pidValue = (Kp * P) + (Ki * integral) + (Kd * D);
        previousError = error;

        int leftSpeed = constrain(speedBase - pidValue, 0, 255);
        int rightSpeed = constrain(speedBase + pidValue, 0, 255);

        analogWrite(ENA, leftSpeed);
        analogWrite(ENB, rightSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        checkBluetooth();

        delay(50);
    }
}
void checkBluetooth() {
    while (BTSerial.available()) {
        char received = BTSerial.read();
        if (received == '\n') {
            processInput(inputString);
            inputString = "";
        } else {
            inputString += received;
        }
    }
}




void dryRun(){
    while(true){
        int array[8];
        for(int i=0; i<8; i++){
            array[i]=readSensor(i);
        }
        bool front = (array[1]>threshold);
        bool left = (array[2] > threshold);
        bool mid = (array[3] > threshold || array[4] > threshold || array[5] > threshold 
        || array[6] > threshold
        & array[1]<threshold & array[2]<threshold & array[7]<threshold & array[0]<threshold); 
        bool right = (array[7] > threshold);
        bool object = (array[0] > threshold);


        if (mid){
            applyPID();
        }
        else if(left){
            turnLeft();

        }
        else if (front){
            turnRight();

        }
        else if (right){
            turnRight();

        }
        else if(object){
            uTurn();

        }


    }
}


void loop() {
    int array[8];
    
    for (int i = 0; i < 8; i++) {
        array[i] = readSensor(i);
        Serial.print(array[i]);
        Serial.print(" ");
    }
    Serial.println();

    // if (oledMenuSelection == PID_TUNING_MODE) {
    //     pidTuningMode();
    // } else {
    //     normalLineFollowing();
    // }

    bool front = (array[1] > threshold); 
    bool left = (array[2] > threshold); 
    bool mid = (array[3] > threshold || array[4] > threshold || array[5] > threshold || array[6] > threshold); 
    bool right = (array[7] > threshold);  
    bool object = (array[0] > threshold);
    if (mid) {
        applyPID();   
       
    } else if (left) {
        turnLeft();
        integral = 0;
    }
    else if(front){
        applyPID();
    }
    else if (right) {
        turnRight();
        integral = 0;  
    } 
    else {
        stopMotors();
    }
    delay(50);
//menucode
    if (digitalRead(UP_BUTTON) == LOW) {
        menuIndex = (menuIndex - 1 + totalItems) % totalItems;
        delay(200);
    }
    
    if (digitalRead(DOWN_BUTTON) == LOW) {
        menuIndex = (menuIndex + 1) % totalItems;
        delay(200);
    }

    if (digitalRead(SELECT_BUTTON) == LOW) {
        handleSelection(menuIndex);
        delay(300);
    }
    
    drawMenu();


}

void calibration(){
    rotate();
    delay(10);
    while()
}



































































































// #include <SoftwareSerial.h>

// SoftwareSerial BTSerial(2, 3); //rxtxbluetooth

// float Kp = 1.0, Ki = 0.0, Kd = 0.0;
// String inputString = "";

// void setup() {
//     Serial.begin(9600);
//     BTSerial.begin(9600);  

//     Serial.println("Bluetooth Ready: Send Kp=, Ki=, Kd= values");
// }

// void loop() {
//     while (BTSerial.available()) {
//         char received = BTSerial.read();
//         if (received == '\n') {
//             processInput(inputString);
//             inputString = "";
//         } else {
//             inputString += received;
//         }
//     }
// }


// void processInput(String data) {
//     if (data.startsWith("Kp=")) {
//         Kp = data.substring(3).toFloat();
//         Serial.print("New Kp: "); Serial.println(Kp);
//     } 
//     else if (data.startsWith("Ki=")) {
//         Ki = data.substring(3).toFloat();
//         Serial.print("New Ki: "); Serial.println(Ki);
//     } 
//     else if (data.startsWith("Kd=")) {
//         Kd = data.substring(3).toFloat();
//         Serial.print("New Kd: "); Serial.println(Kd);
//     } 
// }