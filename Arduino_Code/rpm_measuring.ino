#define encoderPinA  7 // Interrupt pin for encoder A
#define encoderPinB  6// Interrupt pin for encoder B
#define RPWM 5
#define LPWM 4

const float TICKS_PER_REVOLUTION = 103.5; // Set your encoder's ticks per revolution here

volatile unsigned long encoderTicks = 0; // Counts encoder ticks (volatile for interrupt)
unsigned long lastCheckTime = 0;         // Last RPM calculation time
unsigned long lastTickCount = 0;         // Previous tick count for delta calculation

int volts[1250];
//float omegas[1250],percentage[1250];
float perc[] = {0.0,0.2,0.3,0.65,1,0.4,0.1,0.8,0.6,0.0};
float intervals[] = {0.5,1,1.5,1,1,1.5,1.5,2,1,1};
float dt=0.01;
int sn=0;
float MaxV=12.3;
void fill_perc()
{
  int count;
  for(int i=0;i<10;i++){
    count = int(intervals[i]/dt);
    for(int j=0;j<count;j++){
      volts[sn++]=perc[i]*255;
    }
  }
}

// Interrupt service routine for encoder A
void encoderISR() {
  encoderTicks++;
}
void setup() {
  // put your setup code here, to run once:
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, RISING);
  analogWrite(RPWM,0);
  analogWrite(LPWM,0);
  
  fill_perc();
  Serial.begin(9600);
  Serial.print(sn);
  Serial.println();
}



int id = 0;
void loop() {
  if(id==sn){
//    Serial.print(volts[0]*MaxV/255);
//    for(int i=1;i<sn;i++){
//      Serial.print(',');
//      Serial.print(volts[i]*MaxV/255);
//    }
//    Serial.println();
//    Serial.print(omegas[0]);
//    for(int i=1;i<sn;i++){
//      Serial.print(',');
//      Serial.print(omegas[i]);
//    }
//    Serial.println();
    while(1);
  }
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastCheckTime;
  
  // Update RPM every second (adjust interval as needed)
  if (elapsedTime >= 10) {
    noInterrupts(); // Temporarily disable interrupts for accurate reading
    unsigned long currentTicks = encoderTicks;
    interrupts();

    // Calculate ticks since last update
    unsigned long deltaTicks = currentTicks - lastTickCount;
    lastTickCount = currentTicks;
    lastCheckTime = currentTime;

    analogWrite(RPWM,volts[id]);
    analogWrite(LPWM,0);
    // Calculate RPM (handle division as float)
    float rad = 2*PI*(deltaTicks / (float)TICKS_PER_REVOLUTION) * (1000.0 / elapsedTime);
    Serial.print(volts[id]*MaxV/255);
    Serial.print(',');
    Serial.println(rad);    

    id++;  
  }
}
