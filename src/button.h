#include "ezButton.h"

#define BUTTON_1_PIN 13
#define BUTTON_2_PIN 12
#define BUTTON_3_PIN 14
#define BUTTON_4_PIN 27
#define BUTTON_5_PIN 26
#define BUTTON_6_PIN 25
#define BUTTON_7_PIN 33
#define BUTTON_8_PIN 32
#define BUTTON_9_PIN 35
#define BUTTON_10_PIN 34
#define BUTTON_11_PIN 39
#define BUTTON_12_PIN 36

ezButton button1(BUTTON_1_PIN);
ezButton button2(BUTTON_2_PIN);
ezButton button3(BUTTON_3_PIN);
ezButton button4(BUTTON_4_PIN);
ezButton button5(BUTTON_5_PIN);
ezButton button6(BUTTON_6_PIN);
ezButton button7(BUTTON_7_PIN);
ezButton button8(BUTTON_8_PIN);
ezButton button9(BUTTON_9_PIN);
ezButton button10(BUTTON_10_PIN);
ezButton button11(BUTTON_11_PIN);
ezButton button12(BUTTON_12_PIN);

long debounceDelay = 50; // the debounce time; increase if the output flickers

void setupButton(int pin, ezButton button)
{
    button.setDebounceTime(debounceDelay); // set debounce time to 50 milliseconds
    pinMode(pin, INPUT_PULLUP);
}

void setupButtons()
{
    setupButton(BUTTON_1_PIN, button1);
    setupButton(BUTTON_2_PIN, button2);
    setupButton(BUTTON_3_PIN, button3);
    setupButton(BUTTON_4_PIN, button4);
    setupButton(BUTTON_5_PIN, button5);
    setupButton(BUTTON_6_PIN, button6);
    setupButton(BUTTON_7_PIN, button7);
    setupButton(BUTTON_8_PIN, button8);
    setupButton(BUTTON_9_PIN, button9);
    setupButton(BUTTON_10_PIN, button10);
    setupButton(BUTTON_11_PIN, button11);
    setupButton(BUTTON_12_PIN, button12);
}