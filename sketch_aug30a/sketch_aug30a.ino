int PWMA = 23; // Initialising driver pins 
int AIN1=27;
int AIN2=26;

int PWMB = 19;
int BIN1=13;
int BIN2=14;

char command;
String direction ="";
void setup() {
 Serial.begin(9600);
 pinMode(PWMA, OUTPUT); // Setting pinmode to output 
 pinMode(AIN1, OUTPUT);
 pinMode(AIN2, OUTPUT);
 pinMode(PWMB, OUTPUT);
 pinMode(BIN1, OUTPUT);
 pinMode(BIN2, OUTPUT);

 digitalWrite(AIN1, LOW); // Making sure nothing spins at boot
 digitalWrite(AIN2, LOW);
 digitalWrite(BIN1, LOW);
 digitalWrite(BIN2, LOW);

 analogWrite(PWMA, 0);
 analogWrite(PWMB, 0);



}

void loop() {
if (Serial.available() > 0){

    command = Serial.read();  // take input from user

}


if (((direction == "forward") && (command == 'R')) || ((direction == "reverse") && (command == 'F'))) { // motor damage reduction
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    delay(500);
}

if (command == 'F'){ // Forward Case

    Serial.println("Forward!");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    direction = "forward";
    command = '0';

}

if (command == 'R'){ // Reverse case

    Serial.println("Reverse!");
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    direction = "reverse";
    command = '0';

}

if (command == 'S'){ // Stop case

    Serial.println("STOP!");
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    command = '0';

}

if (command == 'L'){ // Low speed case

    Serial.println("LOW");
    analogWrite(PWMA, 50);
    analogWrite(PWMB, 50);
    command = '0';

}

if (command == 'M'){ // Medium speed case

    Serial.println("Medium");
    analogWrite(PWMA,127);
    analogWrite(PWMB,127);
    command = '0';

}

if (command == 'H'){ // Medium speed case

    Serial.println("HIGH SPEED!");
    analogWrite(PWMA, 255);
    analogWrite(PWMB, 255);
    command = '0';

}


}
