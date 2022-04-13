import csv
import matplotlib.pyplot as plt
import numpy as np

def array2csv(headerArray, dataArray, outputFile):
    csvFile = open(outputFile, 'w', encoding='utf-8')
    csvFileWriter = csv.writer(csvFile)

    csvFileWriter.writerow(headerArray)

    N = len(dataArray["steps"])
    for i in range(N):
        csvLine = [dataArray["steps"][i], dataArray["waitTime"][i]]
        csvFileWriter.writerow(csvLine)

def plot_graph(dataArray, title, average = False):
    N = len(dataArray)
    x = dataArray["steps"]
    if average:
        y = dataArray["averageOvertime"]
    else:
        y = dataArray["waitTime"]

    plt.plot(x, y)
    plt.title(title)
    plt.xlabel("Time in Simulation / s")
    plt.ylabel("Average Waiting Time in Lane / s")
    plt.show()
