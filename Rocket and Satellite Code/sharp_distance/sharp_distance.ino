const int distancePin = A6;
int distance ;
void setup() {
  pinMode(distancePin, INPUT);
  Serial.begin(9600);
}

void loop() {
 distance = analogRead(distancePin);
 Serial.println(distance);

}
