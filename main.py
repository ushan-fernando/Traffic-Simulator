from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy as np
from helper import array2csv, plot_graph

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Creating Nessary Directories
output_dir = os.path.join("outputs")
queue_path = os.path.join(output_dir, "queue")
statistics_path = os.path.join(output_dir, "statistics")

if not os.path.exists(queue_path):
    os.makedirs(queue_path)
if not os.path.exists(statistics_path):
    os.makedirs(statistics_path)

from sumolib import checkBinary
import traci

# ------- Constants -------
SEED = 42
STEPS = 3600
N = 3000

class TrafficSimulator:
    def __init__(self, numberOfCars, steps):
        self.numberOfCars = numberOfCars
        self.steps = steps

        self.waitingTimeLane1 = []
        self.waitingTimeLane2 = []
        self.waitingTimeLane3 = []
        self.waitingTimeLane4 = []

        self._lane1 = "E0_0"
        self._lane2 = "E2_0"
        self._lane3 = "-E1_0"
        self._lane4 = "-E3_0"
    
    def generate_routefile(self):
        np.random.seed(SEED)
        
        with open("traffic.rou.xml", "w") as routes:
            print("""<routes>
            <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
            <route id="right" edges="E0 E1"/>
            <route id="left" edges="-E1 -E0"/>
            <route id="down" edges="E2 E3"/>
            <route id="up" edges="-E3 -E2"/>
            <route id="right-up" edges="E0 -E2"/>
            <route id="down-right" edges="E2 E1"/>
            <route id="left-down" edges="-E1 E3"/>
            <route id="up-left" edges="-E3 -E0"/>""", file=routes)
        
            rate = self.numberOfCars / self.steps
            carId = 0
            for step in range(self.steps):
                numCars = np.random.poisson(rate)
                for _ in range(numCars):
                    straight_or_turn = np.random.uniform()
                    if straight_or_turn > 0.25:
                        straight = np.random.randint(1, 5)
                        if straight == 1:
                            print('    <vehicle id="right_%i" type="car" route="right" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if straight == 2:
                            print('    <vehicle id="down_%i" type="car" route="down" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if straight == 3:
                            print('    <vehicle id="left_%i" type="car" route="left" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if straight == 4:
                            print('    <vehicle id="up_%i" type="car" route="up" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                    else:
                        turn = np.random.randint(1, 5)
                        if turn == 1:
                            print('    <vehicle id="right-up_%i" type="car" route="right-up" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if turn == 2:
                            print('    <vehicle id="down-right_%i" type="car" route="down-right" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if turn == 3:
                            print('    <vehicle id="left-down_%i" type="car" route="left-down" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
                        if turn == 4:
                            print('    <vehicle id="up-left_%i" type="car" route="up-left" depart="%i" departSpeed="10" />' % (carId, step), file=routes)
                            carId += 1
            print("</routes>", file=routes)
    

    def run(self):
        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            numVehiclesLane1 = traci.lane.getLastStepVehicleNumber(self._lane1)
            numVehiclesLane3 = traci.lane.getLastStepVehicleNumber(self._lane3)
            numVehiclesLane2 = traci.lane.getLastStepVehicleNumber(self._lane2)
            numVehiclesLane4 = traci.lane.getLastStepVehicleNumber(self._lane4)           

            if traci.trafficlight.getPhase("J2") == 0:
                if numVehiclesLane1 != 0:
                    self.waitingTimeLane1.append((step, traci.lane.getWaitingTime(self._lane1) / numVehiclesLane1))
                if numVehiclesLane3 != 0:
                    self.waitingTimeLane3.append((step, traci.lane.getWaitingTime(self._lane3) / numVehiclesLane3))
            if traci.trafficlight.getPhase("J2") == 2:
                if numVehiclesLane2 != 0:
                    self.waitingTimeLane2.append((step, traci.lane.getWaitingTime(self._lane2) / numVehiclesLane2))
                if numVehiclesLane4 != 0:
                    self.waitingTimeLane4.append((step, traci.lane.getWaitingTime(self._lane4) / numVehiclesLane4))        
                
            step += 1

        traci.close()
        sys.stdout.flush()


    def get_options(self):
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                            default=False, help="run the commandline version of sumo")
        options, args = optParser.parse_args()
        return options
    
    def generate_output_statistics(self):
        array2csv(["timestep", "waitingtime-lane1"], self.waitingTimeLane1, "outputs/statistics/waitingtime-lane1.csv")
        array2csv(["timestep", "waitingtime-lane2"], self.waitingTimeLane2, "outputs/statistics/waitingtime-lane2.csv")
        array2csv(["timestep", "waitingtime-lane3"], self.waitingTimeLane3, "outputs/statistics/waitingtime-lane3.csv")
        array2csv(["timestep", "waitingtime-lane4"], self.waitingTimeLane4, "outputs/statistics/waitingtime-lane4.csv")

        plot_graph(self.waitingTimeLane1, "Lane 1")
        plot_graph(self.waitingTimeLane2, "Lane 2")
        plot_graph(self.waitingTimeLane3, "Lane 3")
        plot_graph(self.waitingTimeLane4, "Lane 4")


# this is the main entry point of this script
if __name__ == "__main__":
    traffic = TrafficSimulator(N, STEPS)
    options = traffic.get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    traffic.generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "traffic.sumocfg",
                             "--queue-output", "outputs/queue/queue.xml"])
    traffic.run()
    traffic.generate_output_statistics()




