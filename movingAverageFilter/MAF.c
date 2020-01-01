// Moving Average Filter
// Decrease noise with dynamic range of input signal

#define FILTERSIZE 10

int flag = 0;

int data[FILTERSIZE];   // Make this as global variable

void MAF(int *_input) {
    int sum, avg;
    
    for(int i = 0; i < FILTERSIZE; i++) {
        sum += data[i];
        sum += *_input;
    }
    avg = sum / (FILTERSIZE + 1);
    *_input = avg;
}

void _collectData(int _data) {
    if(flag < FILTERSIZE) {
        data[flag] = _data;
        flag++;
    } else {
        for(int i = 0; i < FILTERSIZE; i++) {
            data[i] = data[i+1];
        }
        MAF(*_data);
        data[FILTERSIZE-1] = _data;
    }
}