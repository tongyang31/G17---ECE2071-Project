import serial
import subprocess
import threading
import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.io import wavfile

# Serial setup
ser = serial.Serial(port="COM4", baudrate=230400, bytesize=8, parity="N", stopbits=1, timeout=1)
print(f"Connected to: {ser.name}")

sampling_rate = 10000
stop_distance = threading.Event()

def record_data(mode, file_1):
    try:
        if mode == "M":
            ser.write(b'M')
            duration = int(input("Enter recording duration in seconds: "))
            total_bytes_to_read = sampling_rate * duration
            bytes_read = 0
            while bytes_read < total_bytes_to_read:
                x = ser.read(min(512, total_bytes_to_read - bytes_read))
                file_1.write(x)
                file_1.flush()
                bytes_read += len(x)
            print(f"Finished reading {bytes_read} bytes. For total of {duration}s")

        elif mode == "D":
            ser.write(b'D')
            distance = int(input("Enter a recording distance:"))
            ser.write(f"{distance}\n".encode())  # Send actual number as string + newline, represented as ascii
            print("Distance mode activated. Type 'stop' to end recording.")
            stop_distance.clear()
            while not stop_distance.is_set():
                x = ser.read(512)
                if x:
                    file_1.write(x)
                    file_1.flush()
            print("Recording stopped.")
            ser.write(b'M')

        else:
            print("Invalid mode.")

    except KeyboardInterrupt:
        print("Recording interrupted.")

def wait_command():
    while not stop_distance.is_set():
        try:
            cmd = input()
            if cmd.strip().lower() == "stop":
                stop_distance.set()
        except (EOFError, KeyboardInterrupt):
            stop_distance.set()
            break

def compile_and_run_wav():
    compile_command = ["gcc", "write_wav.c", "-o", "write_wav"]
    result = subprocess.run(compile_command, capture_output=True, text=True)
    if result.returncode != 0:
        print("Compilation failed:")
        print(result.stderr)
        return False
    else:
        print("Compilation succeeded!")
        run_command = ["write_wav"]
        subprocess.run(run_command)
        return True

def process_output():
    try:
        sample_rate, data = wavfile.read("output.wav")
        time_axis = np.linspace(0, len(data) / sample_rate, num=len(data))

        with open("output.csv", "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([f"Sample Rate: {sample_rate} Hz"])
            writer.writerow(["Time (s)", "Amplitude"])
            for t, amp in zip(time_axis, data):
                writer.writerow([t, amp])
        print("CSV saved as 'output.csv'.")

        plt.figure(figsize=(12, 4))
        plt.plot(time_axis, data, linewidth=0.5)
        plt.title("Audio Waveform")
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude")
        plt.tight_layout()
        plt.savefig("output.png")
        plt.close()
        print("Waveform saved as 'output.png'.")

    except Exception as e:
        print(f"Error processing WAV file: {e}")

# Open file once for appending throughout
file_1 = open("raw_ADC_values.data", "wb")

try:
    while True:
        mode = input("\nSelect mode:\nManual Recording Mode (M)\nDistance Trigger Mode (D)\n").strip().upper()
        if mode not in ['M', 'D']:
            print("Invalid mode. Try again.")
            continue

        # Start recording thread
        record_thread = threading.Thread(target=record_data, args=(mode, file_1))
        record_thread.start()

        # Handle stop command thread if needed
        if mode == "D":
            command_thread = threading.Thread(target=wait_command)
            command_thread.start()
            command_thread.join()

        record_thread.join()
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        cont = input("\nDo you want to continue recording? (Y/N): ").strip().upper()
        if cont != 'Y':
            break

finally:
    file_1.close()
    ser.close()

    # Compile and run post-processing only once
    if compile_and_run_wav():
        process_output()
