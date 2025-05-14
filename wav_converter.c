#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#define SAMPLE_RATE 10000
#define NUM_CHANNELS 1
#define BITS_PER_SAMPLE 16
#define NUM_SAMPLES 25000  // 50000 bytes / 2 bytes per sample

void write_little_endian_32(uint32_t value, FILE *file) {
    uint8_t bytes[4];
    bytes[0] = value & 0xFF;
    bytes[1] = (value >> 8) & 0xFF;
    bytes[2] = (value >> 16) & 0xFF;
    bytes[3] = (value >> 24) & 0xFF;
    fwrite(bytes, 1, 4, file);
}

void write_little_endian_16(uint16_t value, FILE *file) {
    uint8_t bytes[2];
    bytes[0] = value & 0xFF;
    bytes[1] = (value >> 8) & 0xFF;
    fwrite(bytes, 1, 2, file);
}

int main(){
    int data_size = NUM_SAMPLES;
    FILE *file = fopen("raw_ADC_values.data","rb"); // "rb" = read binary
    if (file != NULL){  //check file presence
        FILE *wav = fopen("output.wav","wb");
        fwrite("RIFF", 1, 4, wav);
        write_little_endian_32(44 + data_size, wav);     // ChunkSize
        fwrite("WAVE", 1, 4, wav);
        fwrite("fmt ", 1, 4, wav);                       // Subchunk1 ID
        write_little_endian_32(16, wav);                // Subchunk1 Size
        write_little_endian_16(1, wav);                 // Audio format (1 = PCM)
        write_little_endian_16(1, wav);                 // NumChannels
        write_little_endian_32(SAMPLE_RATE, wav);       // SampleRate
        write_little_endian_32(SAMPLE_RATE * BITS_PER_SAMPLE*1/8, wav);   // ByteRate
        write_little_endian_16(2, wav);                 // BlockAlign   
        write_little_endian_16(BITS_PER_SAMPLE, wav);   // BitsPerSample
        fwrite("data", 1, 4, wav);                      // Subchunk2 ID
        write_little_endian_32(data_size, wav);         // Subchunk2 Size
        
        // a byte is still 8 bits here, its just that only the lower 12 bits contain information
        uint16_t ADC_value; //each ADC value is a 12-bit value, stored in 2 bytes (uint16_t)
        while(fread(&ADC_value,sizeof(uint16_t),1,file) == 1){ //this reads 2 bytes at a time
            //convert ADC value (0-4095) to range of -32768 and +32767
            // 0 should be -32768
            int16_t scaled_value = (int16_t)((((double)ADC_value/4095.0)*(32767.0+32768.0))-32768.0);
            write_little_endian_16(scaled_value, wav);

        }
        fclose(file);
        fclose(wav);
        return 0;
    }
    else{
        printf("Error in opening input file \n");
    }
}

