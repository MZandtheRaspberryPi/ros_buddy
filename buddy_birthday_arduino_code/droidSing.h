void droidSing (int soundPin, uint16_t toneFreq, uint16_t toneDuration){
  // function to produce a given pitch for given duration
  tone(soundPin, toneFreq);
  delay(toneDuration);
  noTone(soundPin);
}

void singCB(const buddy_msg::buddy_music_note& buddyMusicMsg){
  // callback function for ROS to use the message to make a note
  droidSing(buzzerPin, buddyMusicMsg.frequency, buddyMusicMsg.duration);
}
