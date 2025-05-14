#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SAMPLE_RATE 10000         // 10kSPS (matches Python)
#define NUM_CHANNELS 1  
#define BITS_PER_SAMPLE 8         // 8-bit audio

void write_little_endian_32(uint32_t value, FILE *file) {
    uint8_t bytes[4] = {
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF
    };
    fwrite(bytes, 1, 4, file);
}

void write_little_endian_16(uint16_t value, FILE *file) {
    uint8_t bytes[2] = {
        value & 0xFF,
        (value >> 8) & 0xFF
    };
    fwrite(bytes, 1, 2, file);
}

int main() {
    FILE *file = fopen("raw_ADC_values.data", "rb");
    if (!file) {
        printf("Error: Could not open raw_ADC_values.data\n");
        return 1;
    }

    // Check if input file is empty
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    rewind(file);

    if (file_size == 0) {
        printf("Error: raw_ADC_values.data is empty!\n");
        fclose(file);
        return 1;
    }

    FILE *wav = fopen("output.wav", "wb");  // Use "wb" instead of "wb+"
    if (!wav) {
        printf("Error: Could not create output.wav\n");
        fclose(file);
        return 1;
    }

    // --- Write WAV Header ---
    fwrite("RIFF", 1, 4, wav);                             // ChunkID
    write_little_endian_32(0, wav);                        // Placeholder (ChunkSize)
    fwrite("WAVE", 1, 4, wav);                             // Format

    fwrite("fmt ", 1, 4, wav);                             // Subchunk1 ID
    write_little_endian_32(16, wav);                       // Subchunk1 Size (16 for PCM)
    write_little_endian_16(1, wav);                        // AudioFormat (1 = PCM)
    write_little_endian_16(NUM_CHANNELS, wav);             // NumChannels
    write_little_endian_32(SAMPLE_RATE, wav);              // SampleRate
    write_little_endian_32(SAMPLE_RATE * NUM_CHANNELS * BITS_PER_SAMPLE / 8, wav); // ByteRate
    write_little_endian_16(NUM_CHANNELS * BITS_PER_SAMPLE / 8, wav); // BlockAlign
    write_little_endian_16(BITS_PER_SAMPLE, wav);          // BitsPerSample

    fwrite("data", 1, 4, wav);                             // Subchunk2 ID
    write_little_endian_32(0, wav);                        // Placeholder (Subchunk2 Size)

    // --- Copy Samples ---
    uint8_t sample;
    int sample_count = 0;
    while (fread(&sample, 1, 1, file) == 1) {
        fwrite(&sample, 1, 1, wav);
        sample_count++;
    }

    // --- Update Header with Correct Sizes ---
    uint32_t data_chunk_size = sample_count;  // 1 byte per sample
    uint32_t riff_chunk_size = 36 + data_chunk_size;  // 36 = header size

    fseek(wav, 4, SEEK_SET);
    write_little_endian_32(riff_chunk_size, wav);  // Update RIFF chunk size

    fseek(wav, 40, SEEK_SET);
    write_little_endian_32(data_chunk_size, wav);  // Update data chunk size

    fclose(file);
    fclose(wav);

    if (sample_count == 0) {
        printf("Warning: WAV file created but contains 0 samples!\n");
    } else {
        printf("Success: WAV file created with %d samples (8-bit, %d Hz)\n", sample_count, SAMPLE_RATE);
    }

    return 0;
}