#include <stdio.h>

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
        float encoderGamma;
        float encoderBeta;
} Data;


int main() {
    // read in file:
    FILE *inFilePtr = fopen("./spongebob2.bin", "rb");
    if (inFilePtr == NULL) {
        perror("Error opening file");
        return 1;
    }
    // write csv
    FILE *outFilePtr = fopen("log.csv", "w");

    if (inFilePtr == NULL) {
        fclose(inFilePtr);
        inFilePtr = NULL;
        return -1;
    }

    int count = 0;

    char* header = "segment,time,throttle,vane,rpm,roll,pitch,yaw,accelx,accely,accelz,gx,gy,gz,magx,magy,magz,encoderGamma,encoderBeta,\n"; // remember to change if data changes
    fprintf(outFilePtr, "%s", header);
    while (1) {
        Data d;
        int bytesRead = fread(&d, sizeof(Data), 1, inFilePtr);
        if (bytesRead != 1) {
            fclose(inFilePtr);
            fflush(outFilePtr);
            fclose(outFilePtr);
            printf("Done writing. Wrote %d data points.\n", count);
            break;
        }
        count++;
        fprintf(outFilePtr,
        "%d,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,\n",
        d.segment, d.time,
        d.throttle, d.vane, d.rpm, d.roll, d.pitch, d.yaw,
        d.accelx, d.accely, d.accelz, d.gx, d.gy, d.gz, d.magx, d.magy, d.magz,
        d.encoderGamma, d.encoderBeta);
    }
}

