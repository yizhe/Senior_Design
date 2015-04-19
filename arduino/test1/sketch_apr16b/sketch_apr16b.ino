
int interval = 200;
long timetrack[50];
int state[] = {LOW, LOW, LOW, LOW, LOW};
long oldtime;
long newtime;
int toprint;
//j is for looping through pin ids.
int i,j;

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
    if (interval >200){
      i = 0; j = 4;
      oldtime = micros();
      while (i < 50){
        newtime = micros();
        if (newtime - oldtime > 3){
          timetrack[i] = newtime;
          oldtime = newtime;
          for (j=4; j>=1; --j){
            state[j] = state[j-1];
          }
          state[0] = fastGpioDigitalRegSnapshot(GPIO_FAST_IO3);
          fastGpioDigitalRegWriteUnsafe(GPIO_FAST_IO2, state[j]);
           
          }
          
          ++i;
        }
        
      }
      for (i=0; i<50; ++i){
        toprint = timetrack[i];
        Serial.println(toprint);
      }
      interval = 0;
    }
  }
  

}
