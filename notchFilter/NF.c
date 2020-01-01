// Notch Filter (Band Stop Filter)
// Filter that rejects specific frequency band

#define PI 3.141592

void _NFCoefficient(float _f_peak, float _bandwidth, float _Ts,
                    float *_a0, float *_a1, float *_a2,
                    float *_b1, float *_b2) {
    float w0_peak = 2 * PI * _f_peak;
    float Q = _f_peak / _bandwidth;
    int H0 = 1;
    float a0, a1, a2, b0, b1, b2;

    a0 = ((H0 * 4) / (_Ts * _Ts)) + (H0 * (w0_peak * w0_peak));
    a1 = ((-2) * H0 * 4 / (_Ts * _Ts)) + (2 * (w0_peak * w0_peak));
    a2 = (H0 * 4 / (_Ts * _Ts)) + (H0 * (w0_peak * w0_peak));
    
    b0 = (4 / (_Ts * _Ts)) + (2 * (w0_peak / Q) / _Ts) + (w0_peak * w0_peak);
    b1 = ((-8) / (_Ts * _Ts)) + (2 * (w0_peak * w0_peak));
    b2 = (4 / (_Ts * _Ts)) - (2 * (w0_peak / Q) / _Ts) + (w0_peak * w0_peak);

    *_a0 = a0 / b0;
    *_a1 = a1 / b0;
    *_a2 = a2 / b0;
    *_b1 = b1 / b0;
    *_b2 = b2 / b0;
}

void NF(int *_x, int *_y, int _n, float f_peak, float bandwidth, float Ts) {
    float _x1 = 0.0f, _x2 = 0.0f;
    float _y1 = 0.0f, _y2 = 0.0f;
    float a0 = 0, a1 = 0, a2 = 0;
    float b1 = 0, b2 = 0;

    _NFCoefficient(f_peak, bandwidth, Ts, *a0, *a1, *a2, *b1, *b2);
    
    for(i = 0; i < _n; i++) {
        _y[i] = (a0 * _x[i]) + (a1 * _x1) + (a2 * _x2) + (b1 * _y1) + (b2 * _y2);
        _x2 = _x1;
        _x1 = _x[i];
        _y2 = _y1;
        _y1 = _y[i];
    }
}