#include<Stopwatch.h>

void setup() {
  Serial.begin(9600);
  Stopwatch s;//The timing starts here by default
  Serial.println("Pausing for 2000 ms so the timer should reflect this");
  s.startNewTimer();//Restart the timer
  delay(2000);
  Serial.println("Delayed for 2 seconds, now pausing for 3 seconds so the timer won't add this as it's paused.");
  s.pauseTimer();
  delay(3000);
  Serial.println("Is the timer paused?");
  if(s.isPaused()){
     Serial.println("The timer is paused."); 
  }
  Serial.print("Elapsed time should be approximately 2000 ms: ");
  Serial.print(s.getElapsedMillis());
  Serial.println(" ms. Now resuming the timer followed by a pause of 3400 ms...");
  s.resumeTimer();
  delay(3400);
  Serial.println("So, the elapsed time should be approximately 2000 + 3400 = 5400 (printing to the screen takes time)");
  Serial.println(s.getElapsedMillis());
  Serial.println("Elapsed microseconds should be approximately 5400000:");
  Serial.println(s.getElapsedMicros());
  Serial.println("End of demonstration");
}

void loop() {


}