#include "unity.h"
#include <Arduino.h>

void runDriverTest();

void runUnityTests(void) {
  UNITY_BEGIN();
  runDriverTest();
  UNITY_END();
}

/**
  * For Arduino framework
  */
void setup() {
  // Wait ~2 seconds before the Unity test runner
  // establishes connection with a board Serial interface
  delay(2000);

  runUnityTests();
}

void loop() {}