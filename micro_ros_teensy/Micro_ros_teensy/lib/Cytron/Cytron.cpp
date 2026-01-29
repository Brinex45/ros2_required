#include "Cytron.h"

Cytron::Cytron(int temp_pwm,int temp_dir_pin,bool temp_direction,int temp_max_pwm):pwm(temp_pwm),dir_pin(temp_dir_pin),direction(temp_direction),max_pwm(constrain(temp_max_pwm, 0, 255)) {
    
    pinMode(dir_pin,OUTPUT);
    pinMode(pwm,OUTPUT);
}

void Cytron::rotate(int value){

  if(value>=0){
        digitalWrite(dir_pin,direction);
    }
    else{
        digitalWrite(dir_pin,!direction);
        value = -value;
    }
    value = constrain(value, 0, max_pwm);
    analogWrite(pwm,abs(value));
}

void Cytron::test(){
   Serial.begin(9600);
   Serial.println(String("testing Cytron pins pwm - ") + pwm + String("  direction - ") + dir_pin);
   delay(2000);
   for(int i = 0;i<50;i+=5){
    rotate(i);
    Serial.println(String("PWM - ")+i);
    delay(200);
   }
   for(int i = 50;i>-50;i-=5){
    rotate(i);
    Serial.println(String("PWM - ")+i);
    delay(200);
   }
   for(int i = -50;i<0;i+=5){
    rotate(i);
    Serial.println(String("PWM - ")+i);
    delay(200);
   }
   Serial.println("test complete!");
}