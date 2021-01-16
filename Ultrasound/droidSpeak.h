//This Function Generates a Random series as beeps in order to create speech

void droidSpeak (int soundPin, uint8_t wordCount) {
  int toneDuration;
  int numberOfWords;
  int toneFreq;     // frequency of tone created
  int phraseDelay;  // the time between individual statements
  //Serial.print("Number of words = ");
  //Serial.println(numberOfWords);

  // generate the random set of words
  for ( int i=0; i <= wordCount; i++) {
    toneDuration = random (50, 300);
    toneFreq = random (200, 1500);
    tone(soundPin, toneFreq);
    delay(toneDuration);
    noTone(soundPin);

  }

  //phraseDelay = random(100, 10000);
  //delay(phraseDelay);
  //noTone(soundPin);

}
