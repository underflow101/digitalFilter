// All Pass Filter
// Delay wanted frequency's phase by 90 degrees

#define PI 3.141592

void APF(float *_input, float *_output, float _samplingFrequency, float _cutOffFrequency, float *_pastInput, float *_pastOutput) {
    float a1, b0, b1, w0;
    w0 = 2 * PI * _cutOffFrequency;
    a1 = (w0 - (2 * _samplingFrequency)) / (2 * _samplingFrequency + w0);
    b0 = a1;
    b1 = 1;

    *_output = b0 * (*_input) + b1 * (*_pastInput) - a1 * (*_pastOutput);
    *_pastOutput = *_output;
    *_pastInput = *_input;
}