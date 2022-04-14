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

def plot_graphs(dataArray, trafficLightType, average = False):
    # Gathering Data from dataArray
    x = [lane[trafficLightType]["steps"] for lane in dataArray.values()]
    if average:
        y = [lane[trafficLightType]["averageOvertime"] for lane in dataArray.values()]
    else:
        y = [lane[trafficLightType]["waitTime"] for lane in dataArray.values()]

    # Plotting Data
    fig, axs = plt.subplots(2, 2)
    for i in range(2):
        for j in range(2):
            axs[i][j].plot(x[j*2+i], y[j*2+i])

    # Setting labels and title
    for i in range(2):
        for j in range(2):
            axs[i][j].set(title="Lane {}".format(j*2 + i + 1),
                          xlabel="Time in Simulation / s",
                          ylabel="Average Waiting Time in Lane / s")
    # Displaying graphs
    plt.show()
