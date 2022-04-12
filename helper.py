import csv
import matplotlib.pyplot as plt
import numpy as np

def array2csv(headerArray, dataArray, outputFile):
    csvFile = open(outputFile, 'w', encoding='utf-8')
    csvFileWriter = csv.writer(csvFile)

    csvFileWriter.writerow(headerArray)

    N = len(dataArray)
    for i in range(N):
        csvLine = [dataArray[i][0], dataArray[i][1]]
        csvFileWriter.writerow(csvLine)

def plot_graph(dataArray, title):
    N = len(dataArray)
    x = []
    y = []
    
    for i in range(N):
        x.append(dataArray[i][0])
        y.append(dataArray[i][1])
    
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel("Time in Simulation / s")
    plt.ylabel("Average Waiting Time in Lane / s")
    plt.show()
