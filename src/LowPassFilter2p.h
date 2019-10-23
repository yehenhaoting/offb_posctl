//
// Created by zm on 19-3-11.
//

#ifndef OFFB_POSCTL_LOWPASSFILTER2P_H
#define OFFB_POSCTL_LOWPASSFILTER2P_H


class LowPassFilter2p {
public:
    LowPassFilter2p(float sample_freq, float cutoff_freq) {
        set_cutoff_frequency(sample_freq, cutoff_freq);
        _delay_element_1 = _delay_element_2 = 0;
    }

    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    // apply - Add a new raw value to the filter
    // and retrieve the filtered result
    float apply(float sample);

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _cutoff_freq;
    }

private:
    float _cutoff_freq;
    float _a1;
    float _a2;
    float _b0;
    float _b1;
    float _b2;
    float _delay_element_1;        // buffered sample -1
    float _delay_element_2;        // buffered sample -2
};

#endif //OFFB_POSCTL_LOWPASSFILTER2P_H
