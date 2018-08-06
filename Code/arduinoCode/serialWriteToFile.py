import serial
import keyboard

serial_port = '/dev/cu.usbmodem1421';
baud_rate = 115200; #In arduino, Serial.begin(baud_rate)
write_to_file_path = "output.txt";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(serial_port, baud_rate)
while True:
    if keyboard.is_pressed('q'):#if key 'q' is pressed 
        print('Byeeeeee')
        break#finishing the loop
    else:
        line = ser.readline();
        line = line.decode("utf-8", "ignore") #ser.readline returns a binary, convert to string
        #print(line);
        output_file.write(line);