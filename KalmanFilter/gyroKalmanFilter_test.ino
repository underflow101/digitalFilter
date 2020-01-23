// MPU6050 with Kalman Filter applied

#include "gyroKalmanFilter.h"
#include <LedControl.h>

LedControl lc = LedControl(12, 11, 10, 1);

byte circle[8] = {B00011000,
                  B00100100,
                  B01000010,
                  B10000001,
                  B10000001,
                  B01000010,
                  B00100100,
                  B00011000};
byte leftarrow[8] = {B00011000,
                     B00100000,
                     B01000000,
                     B11111111,
                     B11111111,
                     B01000000,
                     B00100000,
                     B00011000};
byte strongleftarrow[8] = {B00011010,
                           B00100100,
                           B01001000,
                           B11111111,
                           B11111111,
                           B01001000,
                           B00100100,
                           B00011010};
byte rightarrow[8] = {B00011000,
                      B00000100,
                      B00000010,
                      B11111111,
                      B11111111,
                      B00000010,
                      B00000100,
                      B00011000};
byte strongrightarrow[8] = {B01011000,
                            B00100100,
                            B00010010,
                            B11111111,
                            B11111111,
                            B00010010,
                            B00100100,
                            B01011000};
byte uparrow[8] = {B00011000,
                   B00111100,
                   B01011010,
                   B10011001,
                   B00011000,
                   B00011000,
                   B00011000,
                   B00011000};
byte stronguparrow[8] = {B00011000,
                         B00111100,
                         B01011010,
                         B10111101,
                         B01011010,
                         B10011001,
                         B00011000,
                         B00011000};
byte downarrow[8] = {B00011000,
                     B00011000,
                     B00011000,
                     B00011000,
                     B10011001,
                     B01011010,
                     B00111100,
                     B00011000};
byte strongdownarrow[8] = {B00011000,
                           B00011000,
                           B10011001,
                           B01011010,
                           B10111101,
                           B01011010,
                           B00111100,
                           B00011000};
                


void setup() {
    Serial.begin(115200);
    gyroInit();
    lc.shutdown(0, false);
    lc.clearDisplay(0);
    for(int i = 0; i < 8; i++) {
        lc.setRow(0, i, circle[i]);
    }
    delay(3000);
}

void loop() {
    int x, y, z;
    int prevx = 0, prevy = 0, prevz = 0;
    int dx, dy, dz;
    gyroExecute();
    x = getXcoord();
    y = getYcoord();
    z = getZcoord();
    Serial.println(x);
    Serial.println(y);
    Serial.println(z);
    dx = x - prevx;
    dy = y - prevy;
    dz = z - prevz;
    if(dx > 1000) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, strongleftarrow[i]);
        }
    } else if(dx < -1000) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, strongrightarrow[i]);
        }
    } else if(dy > 1000) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, strongdownarrow[i]);
        }
    } else if(dy < -1000) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, stronguparrow[i]);
        }
    } else if(dx > 500) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, leftarrow[i]);
        }
    } else if(dx < -500) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, rightarrow[i]);
        }
    } else if(dy > 500) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, downarrow[i]);
        }
    } else if(dy < -500) {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, uparrow[i]);
        }
    } else {
        for(int i = 0; i < 8; i++) {
            lc.setRow(0, i, circle[i]);
        }
    }

    prevx = x;
    prevy = y;
    prevz = z;
    newDelay(1000);
}
