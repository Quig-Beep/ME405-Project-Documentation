from matplotlib import pyplot as plt

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



plt.plot(time, height, marker='o')
plt.title('Height vs Time')
plt.xlabel(headers[0])
plt.ylabel(headers[1])
plt.grid()
plt.show()