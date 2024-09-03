#include <SD.h>
#include "SD_stuff.h"

namespace logger {
    int FILE_WRITE_ERR = -2, FILE_OPEN_ERR = -1;
    int write_count = 0;

    File file;
}

void initializeSD() {
    serial.print("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        serial.println("SD initialization failed!");
        while (true);
    }
    serial.println("SD initialization done.");
}


void logger::close() {
    file.flush();
    file.close();
}

void logger::write(Data* data) {
    // open file
    // File file = SD.open(filename, FILE_WRITE);
    // if (!file) return FILE_WRITE_ERR;
    // write data
    file.write((uint8_t*) data, sizeof(*data));
    write_count++;
    if (write_count % 100 == 0) { // flush every 100 writes
        file.flush();
    }
    // file.close();
}

int logger::open(const char* filename) {
    file = SD.open(filename, FILE_WRITE_BEGIN);
    file.truncate();
    if (!file) return FILE_OPEN_ERR;
    return 0;
}