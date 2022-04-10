from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy as np
from helper import array2csv, plot_graph

import xml.etree.ElementTree as ET

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

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
    
    def generate_routefile(self, file_name):
        """
        Generates the route files for sumo

        Parameters
        ----------
        file_name
            The xml file name to write to

        Returns
        -------
        None
        """
        # Setting up Random Seed
        np.random.seed(SEED)
        
        with open(file_name, "w") as routes_file:
            routes = ET.Element("routes")
            vType = ET.SubElement(routes, "vType")
            vType.attrib = {
                "id": "car",
                "accel": "0.8",
                "decel": "4.5",
                "sigma": "0.5",
                "length": "5",
                "minGap": "2.5",
                "maxSpeed": "25",
                "guiShape": "passenger",
            }
            route_ids = ["right", "left", "down", "up", "right-up", "down-right", "left-down", "up-left"]
            route_edges = ["E0 E1", "-E1 -E0", "E2 E3", "-E3 -E2", "E0 -E2", "E2 E1", "-E1 E3", "-E3 -E0"]
            for id, edge in zip(route_ids, route_edges):
                route = ET.Element("route")
                route.attrib = {
                    "id": id,
                    "edges": edge,
                }
                routes.append(route)
#            print("""<routes>
#            <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
#            <route id="right" edges="E0 E1"/>
#            <route id="left" edges="-E1 -E0"/>
#            <route id="down" edges="E2 E3"/>
#            <route id="up" edges="-E3 -E2"/>
#            <route id="right-up" edges="E0 -E2"/>
#            <route id="down-right" edges="E2 E1"/>
#            <route id="left-down" edges="-E1 E3"/>
#            <route id="up-left" edges="-E3 -E0"/>""", file=routes_file)
        
            rate = self.numberOfCars / self.steps
            carId = 0
            for step in range(self.steps):
                numCars = np.random.poisson(rate)
                for _ in range(numCars):
                    vehicle = ET.Element("vehicle")
                    vehicle.attrib = {
                        "type": "car",
                        "departSpeed": str(10),
                        "depart": str(step),
                    }
                    # RNG to decide if the car will turn or go straight
                    straight_or_turn = np.random.uniform()
                    # Go straight
                    if straight_or_turn > 0.25:
                        # Decide the direction
                        straight = np.random.randint(1, 5)
                        # Right
                        if straight == 1:
                            #print('    <vehicle id="right_%i" type="car" route="right" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"right_{carId}"
                            vehicle.attrib["route"] = "right"
                        # Down
                        if straight == 2:
                            #print('    <vehicle id="down_%i" type="car" route="down" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"down_{carId}"
                            vehicle.attrib["route"] = "down"
                        # Left
                        if straight == 3:
                            #print('    <vehicle id="left_%i" type="car" route="left" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"left_{carId}"
                            vehicle.attrib["route"] = "left"
                        # Up
                        if straight == 4:
                            #print('    <vehicle id="up_%i" type="car" route="up" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"up_{carId}"
                            vehicle.attrib["route"] = "up"
                    # Turn
                    else:
                        # Decide direction to turn
                        turn = np.random.randint(1, 5)
                        # Right then up
                        if turn == 1:
                            #print('    <vehicle id="right-up_%i" type="car" route="right-up" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"right_{carId}"
                            vehicle.attrib["route"] = "right-up"
                        # Down then right
                        if turn == 2:
                            #print('    <vehicle id="down-right_%i" type="car" route="down-right" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"right_{carId}"
                            vehicle.attrib["route"] = "down-right"
                        # Left then down
                        if turn == 3:
                            #print('    <vehicle id="left-down_%i" type="car" route="left-down" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"right_{carId}"
                            vehicle.attrib["route"] = "left-down"
                        # Up then left
                        if turn == 4:
                            #print('    <vehicle id="up-left_%i" type="car" route="up-left" depart="%i" departSpeed="10" />' % (carId, step), file=routes_file)
                            vehicle.attrib["id"] = f"right_{carId}"
                            vehicle.attrib["route"] = "up-left"
                    carId += 1
                    routes.append(vehicle)
            tree = ET.ElementTree(routes)
            ET.indent(tree)
            tree.write(file_name)


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
    traffic.generate_routefile("traffic.rou.xml")

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "traffic.sumocfg",
                             "--queue-output", "outputs/queue/queue.xml"])
    traffic.run()
    traffic.generate_output_statistics()
