#include <SoftwareSerial.h>



#define MUX_SIG A0  
#define S0 A0
#define S1 A2 
#define S2 A6 
#define led 6
#define push1 8
#define push2 7
#define IN1 4 //ma
#define IN2 11
#define IN3 5 //mb
#define IN4 9
#define ENA 3   //ma
#define ENB 11  //mb


int speedBase = 200;

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
}

void turnRight() {
    analogWrite(ENA, speedBase);
    analogWrite(ENB, speedBase);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
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




void loop() {
    int sensorValues[8];
    
    for (int i = 0; i < 8; i++) {
        sensorValues[i] = readSensor(i);
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.println();

    // if (oledMenuSelection == PID_TUNING_MODE) {
    //     pidTuningMode();
    // } else {
    //     normalLineFollowing();
    // }

    int threshold = 600;

    bool front = (sensorValues[1] > threshold); 
    bool left = (sensorValues[2] > threshold); 
    bool mid = (sensorValues[3] > threshold || sensorValues[4] > threshold || sensorValues[5] > threshold || sensorValues[6] > threshold); 
    bool right = (sensorValues[7] > threshold);  
    bool object = (sensorValues[0] > threshold);

    float Kp = 2.0, Ki = 0.1, Kd = 1.0;
    float integral = 0, previousError = 0;
    int threshold = 600;
    if (mid) {
        applyPID();   
       
    } else if (left) {
        turnLeft();
        integral = 0;
    } else if (right) {
        turnRight();
        integral = 0;  
    } else {
        stopMotors();
    }
    delay(50);
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