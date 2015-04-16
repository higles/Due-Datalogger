int Getsolar();

void setup() {
  // put your setup code here, to run once:
    pinMode( 80 , INPUT);
    Serial.begin(9600);
    analogReadResolution(12);
    
}

void loop() {
  // put your main code here, to run repeatedly:
    double x;
    
    x = Getsolar();
    Serial.print("voltage: ");
    Serial.println(x);
    
}



int Getsolar(){
  
    double temp;
    temp = analogRead(8);
    temp = temp/819;
    return temp;
    
}
