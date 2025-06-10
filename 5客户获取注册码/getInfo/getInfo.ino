void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("==================");
  Serial.println(ESP.getEfuseMac());
  Serial.println(ESP.getChipModel());
  Serial.println(ESP.getChipCores());
  Serial.println(ESP.getChipRevision());
  Serial.println("==================");


}

void loop() {
  // put your main code here, to run repeatedly:

}
