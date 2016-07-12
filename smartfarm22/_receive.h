void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  msg[msglen] = '\0';
  Serial.printf("Incoming topic = %s, msg = %s \r\n", topic, (char *)msg);
  String msgin = String((char*)msg);

  if (msgin == "ON") {
    digitalWrite(RELAY, LOW);
  }
  else if (msgin == "OFF") {
    digitalWrite(RELAY, HIGH);
  }
}
