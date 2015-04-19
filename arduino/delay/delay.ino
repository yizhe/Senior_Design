
unsigned long timeBefore = 0;
int interval = 200;
int state[] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
int pin[] = {22, 25, 29, 33, 37, 41, 45, 49, 53};
String str;
int ind = 0;
void setup() {
  Serial.begin(9600);
  pinMode(23, INPUT);
  int i;
  for (i = 0; i < 9; i++) {
    pinMode(pin[i], OUTPUT);
  }
}
void loop() {
  unsigned long timeNow = millis();
  if (Serial.available() > 0){
    str =  Serial.readStringUntil('\n');
    interval = 0;
    ind = 0;
    Serial.println("Received: " + str);
    //Serial.read();
    while(str[ind] != '\0'){
      interval = interval*10+str[ind]-48;
      ind = ind+1;
    } 
    Serial.println(interval);
    state[0] = !state[0];
  }
  
  
    
  if (timeNow - timeBefore > interval) {
    timeBefore = timeNow;
    int i;
    for (i = 8; i >= 1; i--) {
      state[i] = state[i-1];
    }
    
    for (i = 0; i < 9; i++) {
    digitalWrite(pin[i], state[i]);
    }
  }
}
