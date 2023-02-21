const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
    pinMode(5, OUTPUT);
    analogWrite(5,1);
}

void loop() {
    recvWithEndMarker();
    // showNewData();
    commandSpeed();
    // analogWrite(5,100);
    // for(int i = 1000; i <= 2000; i++) {
    //   analogWrite(5, i);
    //   delay(300);
    //   Serial.println(i);
    // }
}

void commandSpeed() {
  if(newData == true) {

    Serial.print("This just in ... ");
    Serial.println(atoi(receivedChars));

    int speed = atoi(receivedChars);
    if (speed == 0) {analogWrite(5,1);}
    else if (abs(speed) < 145 && speed !=0 ) { Serial.println("Absolute (+/-) speed must be greater than 145RPM! (except 0)"); }
    else if (abs(speed) > 2850) { Serial.println("Absolute (+/-) speed must be less than 2850RPM!"); }
    else { analogWrite(5, map(speed,-2850,2850,2,254)); }
    newData = false;
  }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}