void setup() {
  Serial.begin(9600);
  while (!Serial.find("g"));
  pinMode (A3, OUTPUT); // put your setup code here, to run once:
  delay(1000);
  digitalWrite(A3, HIGH);
  delay(500);
  digitalWrite(A3, LOW);
}

void loop() {


}
