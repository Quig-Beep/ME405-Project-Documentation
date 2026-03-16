import serial
import csv
from matplotlib import pyplot as plt
from time import sleep

ser = serial.Serial("COM6",115200,timeout=0)

def plot_data(data,title):

    time = [row[0] for row in data]
    velocity = [row[1] for row in data]

    plt.plot(time, velocity, marker='o')
    plt.title('Velocity vs Time')
    plt.xlabel("Time")
    plt.ylabel("Velocity [mm/s]")
    plt.title(f"{title}")
    plt.grid()
    plt.show()

def parse(line):
    if ("Time" in line and "Position" in line) or line.startswith(">:") or line.startswith("|") or line.startswith("+"):
        return None
    elif line.strip() == "--------------------":
        return "DELIM"
    else:
        nums = line.strip().split(',')
        if len(nums)>=2:
            try:
                parsed_line = [float(nums[0]),float(nums[1])]
                return(parsed_line)
            except ValueError:
                pass
        
#set one universal setpoint for all of these graphs, dont see a point in changing it halfway
Setpoint = input("Input Setpoint mm/s: ") + "\r\n"  
ser.write(b"s")
ser.write(Setpoint.encode('utf-8'))

while True:
    #set gains for each loop
    buffer = bytearray()
    i = 0
    a = input("Press enter to trigger new test: ")
    ser.reset_input_buffer()
    ser.write(b"k")
    sleep(0.1)
    Kp = input("Input Kp gain: ")
    ser.write((Kp + "\r\n").encode('utf-8'))
    sleep(0.5)
    ser.reset_input_buffer()
    Ki = input("Input Ki gain: ")
    ser.write((Ki + "\r\n").encode('utf-8'))
    sleep(0.5)
    ser.reset_input_buffer()
    #type go here to run response
    ser.write(b"g")
    sleep(2.0)
    while(True):
        raw = ser.read(ser.in_waiting or 1)
        if not raw:
            i+=1
            sleep(0.001)
            if i == 5000:
                i = 0
                break
        else:
            buffer += raw
            i = 0
    text = buffer.decode('utf-8', errors='replace')
    lines = text.splitlines()
    leftData = [[0,0] for _ in range(30)]
    rightData = [[0,0] for _ in range(30)]
    delim_counter = 0
    i = 0
    for line in lines:
        data = parse(line)
        if data is not None:
            if data == "DELIM":
                delim_counter += 1
                i = 0
            elif delim_counter == 1:
                leftData[i] = data
                i += 1
            elif delim_counter == 2:
                rightData[i] = data
                i += 1
            elif delim_counter == 3:
                break
            else:
                print("Error in data")
                break
    
            
    # left plot
    plot_data(leftData, f"Left motor step response for Kp = {Kp}, Ki = {Ki}")
    # right plot
    plot_data(rightData, f"Right motor step response for Kp = {Kp}, Ki = {Ki}")

    #with open(f"CSV_files/set{Setpoint.strip()}_kp{Kp}_ki{Ki}_step.csv", 'w', newline = "") as file:
    #    writer = csv.writer(file)
    #    writer.writerow(["Time Left", "Velocity Left", "Time Right", "Velocity Right"])
    #    for (timeL, velL), (timeR, velR) in zip(leftData, rightData):
    #        writer.writerow([timeL, velL, timeR, velR])


