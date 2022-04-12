import csv
import matplotlib.pyplot as plt

def array2csv(headerArray, dataArray, outputFile):
    csvFile = open(outputFile, 'w', encoding='utf-8')
    csvFileWriter = csv.writer(csvFile)

    csvFileWriter.writerow(headerArray)

    N = len(dataArray)
    for i in range(N):
        csvLine = [dataArray[0][i], dataArray[1][i]]
        csvFileWriter.writerow(csvLine)

def plot_graph(dataArray, title):
    N = len(dataArray)
    x = dataArray[0]
    y = dataArray[1]

    plt.plot(x, y)
    plt.title(title)
    plt.xlabel("Time in Simulation / s")
    plt.ylabel("Average Waiting Time in Lane / s")
    plt.show()
