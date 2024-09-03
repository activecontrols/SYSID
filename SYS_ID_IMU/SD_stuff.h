#ifndef LOG_H
#define LOG_H

namespace logger {
    typedef struct {
        int segment;
        float time;
        float throttle;
        float vane;
        float rpm;
        float roll;
        float pitch;
        float yaw;
        float accelx;
        float accely;
        float accelz;
        float gx;
        float gy;
        float gz;
        float magx;
        float magy;
        float magz;
    } Data;

    extern int FILE_WRITE_ERR, FILE_OPEN_ERR;

    extern int write_count;
    int open(const char* filename);
    void write(Data* data);
    void close();
}

void initializeSD();

#endif //LOG_H
