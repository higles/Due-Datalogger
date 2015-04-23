float temp = 0;
float count = 0;
int timer = 0;

float Getsolar();

void setup() {
  // put your setup code here, to run once:
    pinMode( 56 , INPUT);
    Serial.begin(115200);
    analogReadResolution(13); 
    timer = millis();   
}

void loop() {
  // put your main code here, to run repeatedly:    
    temp += Getsolar();
    if (millis() - timer > 1000) {
      temp = temp/count;
      timer = millis();
      count = 0;
      Serial.print("voltage: ");
      Serial.println(temp);
      temp = 0;
    }
}



float Getsolar(){
    float t;
    t = (float)analogRead(56);
    t = t*100.00;
    t = t/1241.00;
    count++;
    return t;
    
}
