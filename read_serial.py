import serial
import subprocess

# Set serial UART parameters
ser = serial.Serial(port="COM9", baudrate=230400, bytesize=8, parity="N", stopbits=1, timeout=5)
print(ser.name)

file_1 = open("raw_ADC_values.data", "wb")
total_bytes_to_read = 250 * 100  # 25,000 bytes
bytes_read = 0

while bytes_read < total_bytes_to_read:
    x = ser.read(500)
    actual_len = len(x)
    if actual_len == 0:
        continue  # don't increment if nothing was read
    file_1.write(x)
    bytes_read += actual_len

file_1.close()
ser.close()
print("Finished reading 25,000 bytes.")

# For compiling and executing C file that writes the wav file
compile_command = ["gcc", "write_wav.c", "-o", "write_wav"]
compilation = subprocess.run(compile_command, capture_output=True, text=True)

if compilation.returncode != 0:
    print("Compilation failed:")
    print(compilation.stderr) #standard error
else:
    print("Compilation succeeded!")

run_command = ["write_wav"]
execution = subprocess.run(run_command, capture_output=True, text=True)