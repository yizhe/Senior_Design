
int interval = 200;

String str;
int ind = 0;
void setup() {
  Serial.begin(115200);

  
}
void loop() {
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
  }
  

}
