//This Function Generates a Random series as beeps in order to create speech

void droidSpeak (int soundPin, uint8_t wordCount) {
  int toneDuration;
  int toneFreq;     // frequency of tone created

  // generate the random set of words
  for ( int i=0; i < wordCount; i++) {
    toneDuration = random (50, 300);
    toneFreq = random (200, 1500);
    tone(soundPin, toneFreq);
    delay(toneDuration);
    noTone(soundPin);

  }

}
