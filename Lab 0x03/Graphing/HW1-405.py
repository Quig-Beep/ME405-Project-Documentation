from matplotlib import pyplot as py

filepath = "./step response data.csv"

xdata = []
ydata = []

with open(filepath, "r", encoding="utf-8") as data:
    Title = [t.strip() for t in data.readline().split(",")]
    line_number = 1
    for line in data:
        #ticker to index errors
        line_number+=1

        #remove whitespace and /n
        line = line.strip()
        if not line:
            print("Line " + str(line_number) +" rejected: empty line")
            continue

        #check for comments/strip them and if line is only comments throw error
        if "#" in line: 
            line = line.split("#", 1)[0].strip()
            if not line:
                print("Line " + str(line_number) +" rejected: comment only")
                continue

        #split data into pairs of x n y, skip pairs with only 1 and chop pairs w more than 3 down to 2 
        pairs = [p.strip() for p in line.split(",")]
        if len(pairs) < 2:
            print("Line " + str(line_number) + " rejected: less than 2 columns")
            continue
        pairs = pairs[:2]

        #convert to floats and throw error if cannot
        try:
            x = float(pairs[0])
            y = float(pairs[1])
        except ValueError:
            print("Line " + str(line_number) + " rejected: non-numeric data")
            continue

        #create x and y sets
        xdata.append(x)
        ydata.append(y)

#plot data
py.plot(xdata,ydata,'-o')
py.xlabel(Title[0])
py.ylabel(Title[1])
py.title("CSV Data")
py.show()
