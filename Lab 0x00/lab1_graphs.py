from matplotlib import pyplot as plt
import math
import numpy as np


from numpy import array

with open('lab1_data.csv', 'r') as data:
    items = data.readlines()
    headers = items[0].strip().split(',')

data = []
for i in range(1, len(items)):
    values = items[i].strip().split(',')
    subvalues = []
    for val in values: 
        if '#' in val:
            subvals = val.partition('#')
            for subval in subvals:
                if subval.strip() != '':
                    subvalues.append(subval.strip())
        else:
            subvalues.append(val)
    data.append(subvalues)

clean_data = []
for i in range(len(data)):
    row = []
    for j in range(len(data[i])):
        try:
            row.append(float(data[i][j]))
        except ValueError:
            if len(data[i])<= 2 and (len(data[i]) == 0 or data[i][j] == ''):
                print(f"Empty line found on line {i+2}, skipping line.")
                continue
            elif "#" in data[i][j]:
                print(f"Comment found on line {i+2}")
                continue
            else:
                print(f"Invalid data on line {i+2}, skipping line.")
                continue
    if len(row) >= len(headers):
        clean_data.append(row)

time = [row[0] for row in clean_data]
height = [row[1] for row in clean_data]

# time constant

step_vlt = .632*height[-1]

tau = 0.08461548312758245

theoretical = [0]*len(time)

for i in range(len(time)):
    theoretical[i] = 3.3*(1 - math.exp(-1*time[i]/tau))


plt.plot(time, height, label= 'Experimental',marker='o')
plt.title('RC Curcuit Step Response')
plt.xlabel(headers[0])
plt.ylabel(headers[1])
plt.grid()
plt.plot(time, theoretical, label='Theoretical', color='red')
plt.legend()
plt.show()

# linearized step response

linearized = [0]*len(height)
for i in range(len(height)):
    linearized[i] = math.log(abs(1-(height[i]/(max(height)+.000001))))

# best fit data

coefficients = np.polyfit(time[0:60], linearized[0:60], 1)

best_fit = [coefficients[0]*t + coefficients[1] for t in time]

print("Time constant from best fit: ", -1/coefficients[0])

plt.plot(time, linearized, label='Linearized Experimental', linestyle='None', marker='o')
plt.title('Linearized RC Circuit Step Response')
plt.xlabel('Time (s)')
plt.ylabel('Linearized Voltage')
plt.plot(time, best_fit, label='Best Fit Line', color='red')
plt.grid()
plt.legend()
plt.show()

