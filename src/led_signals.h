
#pragma once


class LEDSignals {
public:
    void init(int brightness = 50);

    void signalInitState(int state);
    void signalErrorState();

    void setPixel(int num, int r, int g, int b);
};