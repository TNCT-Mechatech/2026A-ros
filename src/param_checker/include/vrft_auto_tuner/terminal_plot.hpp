#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

class TerminalPlot {
   public:
    TerminalPlot (float minValue, float maxValue, int width = 80) : minValue_ (minValue), maxValue_ (maxValue), width_ (width) {}

    void plot (float value) {
        if (maxValue_ <= minValue_) return;

        float v   = std::clamp (value, minValue_, maxValue_);
        int   pos = static_cast<int> ((v - minValue_) / (maxValue_ - minValue_) * (width_ - 1));

        for (int i = 0; i < width_; i++) {
            if (i == zeroPosition ()) {
                std::cout << "|";
            } else if (i == pos) {
                std::cout << "*";
            } else {
                std::cout << " ";
            }
        }

        std::cout << " " << value << std::endl;
    }

    void plot (float value, float target, float time) {
        if (maxValue_ <= minValue_) return;

        float v = std::clamp (value, minValue_, maxValue_);
        float t = std::clamp (target, minValue_, maxValue_);

        int posValue  = toPosition (v);
        int posTarget = toPosition (t);
        int posZero   = zeroPosition ();

        for (int i = 0; i < width_; i++) {
            if (i == posValue && i == posTarget) {
                std::cout << "X";
            } else if (i == posValue) {
                std::cout << "*";
            } else if (i == posTarget) {
                std::cout << "T";
            } else if (i == posZero) {
                std::cout << "|";
            } else {
                std::cout << " ";
            }
        }

        std::cout << " y=" << value << " target=" << target << " time=" << time << std::endl;
    }

   private:
    int toPosition (float value) const {
        return static_cast<int> ((value - minValue_) / (maxValue_ - minValue_) * (width_ - 1));
    }

    int zeroPosition () const {
        if (minValue_ > 0.0f || maxValue_ < 0.0f) {
            return -1;
        }

        return toPosition (0.0f);
    }

   private:
    float minValue_;
    float maxValue_;
    int   width_;
};