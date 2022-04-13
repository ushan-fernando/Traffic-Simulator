from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy as np
from helper import array2csv, plot_graph
from fuzzy_controller import fuzzy_logic_controller
import matplotlib.pyplot as plt

import xml.etree.ElementTree as ET

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

# Creating Options
optParser = optparse.OptionParser()
optParser.add_option("--nogui", action="store_true",
                    default=False, help="run the commandline version of sumo")
# ------- Constants -------
SEED = 42
STEPS = 3600
N = 3000

class TrafficSimulator:
    """
    Create a Traffic Simulator

    Parameters
    ----------
    numberOfCars
        Number of cars to simluate
    steps
        The amount of virtual time to simulate
    gui
        Set to run with gui or no
    fixedCycleTime
        The amount of virtual time per cycle for fixed time traffic light simulation
        Default is 42 virtual seconds
    """
    def __init__(self, numberOfCars, steps, gui, fixedCycleTime = 42):
        # Saving Variables
        self.numberOfCars = numberOfCars
        self.steps = steps
        self.fixedCycleTime = fixedCycleTime

        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        if gui:
            self.sumoBinary = checkBinary('sumo')
        else:
            self.sumoBinary = checkBinary('sumo-gui')

        self.waitingTime = {
            "Lane 1": {
                "Fixed": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                },
                "Fuzzy": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                }
            },
            "Lane 2": {
                "Fixed": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                },
                "Fuzzy": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                }
            },
            "Lane 3": {
                "Fixed": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                },
                "Fuzzy": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                }
            },
            "Lane 4": {
                "Fixed": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                },
                "Fuzzy": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                }
            },
            "All": {
                "Fixed": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                },
                "Fuzzy": {
                    "steps": [],
                    "waitTime": [],
                    "averageOvertime": [],
                    "average": 0,
                }
            },
        }

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

        # Root routes
        routes = ET.Element("routes")
        # vType
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

        # Routes and edges
        route_ids = ["right", "left", "down", "up", "right-up", "down-right", "left-down", "up-left"]
        route_edges = ["E0 E1", "-E1 -E0", "E2 E3", "-E3 -E2", "E0 -E2", "E2 E1", "-E1 E3", "-E3 -E0"]
        for id, edge in zip(route_ids, route_edges):
            route = ET.Element("route")
            route.attrib = {
                "id": id,
                "edges": edge,
            }
            routes.append(route)

        # Vehicles
        rate = self.numberOfCars / self.steps
        carId = 0
        for step in range(self.steps):
            numCars = np.random.poisson(rate)
            for _ in range(numCars):
                # Intializing Vehicle
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
                        vehicle.attrib["id"] = f"right_{carId}"
                        vehicle.attrib["route"] = "right"
                    # Down
                    if straight == 2:
                        vehicle.attrib["id"] = f"down_{carId}"
                        vehicle.attrib["route"] = "down"
                    # Left
                    if straight == 3:
                        vehicle.attrib["id"] = f"left_{carId}"
                        vehicle.attrib["route"] = "left"
                    # Up
                    if straight == 4:
                        vehicle.attrib["id"] = f"up_{carId}"
                        vehicle.attrib["route"] = "up"
                # Turn
                else:
                    # Decide direction to turn
                    turn = np.random.randint(1, 5)
                    # Right then up
                    if turn == 1:
                        vehicle.attrib["id"] = f"right_{carId}"
                        vehicle.attrib["route"] = "right-up"
                    # Down then right
                    if turn == 2:
                        vehicle.attrib["id"] = f"right_{carId}"
                        vehicle.attrib["route"] = "down-right"
                    # Left then down
                    if turn == 3:
                        vehicle.attrib["id"] = f"right_{carId}"
                        vehicle.attrib["route"] = "left-down"
                    # Up then left
                    if turn == 4:
                        vehicle.attrib["id"] = f"right_{carId}"
                        vehicle.attrib["route"] = "up-left"
                # Updating carID
                carId += 1
                # Adding vehicles to routes
                routes.append(vehicle)

        # Writing XML file
        tree = ET.ElementTree(routes)
        if sys.version_info.major >= 3 and sys.version_info.minor >= 9:
            ET.indent(tree)
        else:
            sys.stderr.write("XML indentation only works for python version 3.9 and above. Skipping\n")
        tree.write(file_name)
    
    def run_fixed(self, cycleTime = -1):
        """
        Running the simulation with traffic light cycle time at a fixed time.

        Parameters
        ----------
        cycleTime
            The cycle time of the fixed timed traffic light
            Default is set to the value set during object creation

        Returns
        -------
        None
        """
        # Setting Cycle Time
        if cycleTime == -1:
            cycleTime = self.fixedCycleTime

        # Changing Cycle Time
        tree = ET.parse("traffic.net.xml")
        root = tree.getroot()
        tlLogic = root.findall("tlLogic")[0]
        tlLogic[0].attrib["duration"] = str(cycleTime)
        tlLogic[2].attrib["duration"]= str(cycleTime)
        tree.write('traffic.net.xml')

        # Starting SUMO
        traci.start([self.sumoBinary, "-c", "traffic.sumocfg",
                                "--queue-output", "outputs/queue/queue.xml"])
        step = 0

        # Alias Lanes
        lane1 = self.waitingTime["Lane 1"]["Fixed"]
        lane2 = self.waitingTime["Lane 2"]["Fixed"]
        lane3 = self.waitingTime["Lane 3"]["Fixed"]
        lane4 = self.waitingTime["Lane 4"]["Fixed"]

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            numVehiclesLane1 = traci.lane.getLastStepVehicleNumber(self._lane1)
            numVehiclesLane3 = traci.lane.getLastStepVehicleNumber(self._lane3)
            numVehiclesLane2 = traci.lane.getLastStepVehicleNumber(self._lane2)
            numVehiclesLane4 = traci.lane.getLastStepVehicleNumber(self._lane4)

            if traci.trafficlight.getPhase("J2") == 0:
                if numVehiclesLane1 != 0:
                    lane1["steps"].append(step)
                    lane1["waitTime"].append(traci.lane.getWaitingTime(self._lane1) / numVehiclesLane1)
#                    lane1[3].append(np.mean(lane1[1]))
#                    lane1[2] = signal.convolve(lane1[1],b) #filter output using convolution
                    #lane1[2] = signal.lfilter(b,a,lane1[1]) #filter output using lfilter function
                if numVehiclesLane3 != 0:
                    lane3["steps"].append(step)
                    lane3["waitTime"].append(traci.lane.getWaitingTime(self._lane3) / numVehiclesLane3)
#                    lane3[2].append(np.mean(lane3[1]))

            if traci.trafficlight.getPhase("J2") == 2:
                if numVehiclesLane2 != 0:
                    lane2["steps"].append(step)
                    lane2["waitTime"].append(traci.lane.getWaitingTime(self._lane2) / numVehiclesLane2)
#                    lane2[2].append(np.mean(lane2[1]))
                if numVehiclesLane4 != 0:
                    lane4["steps"].append(step)
                    lane4["waitTime"].append(traci.lane.getWaitingTime(self._lane4) / numVehiclesLane4)
#                    lane4[2].append(np.mean(lane4[1]))

            step += 1

        traci.close()
        sys.stdout.flush()


    def run_fuzzy(self):
        """
        Running the simulation with Fuzzy Logic

        Returns
        -------
        None
        """
        # Starting SUMO
        traci.start([self.sumoBinary, "-c", "traffic.sumocfg",
                                "--queue-output", "outputs/queue/queue.xml"])

        step = 0
        fuzzyLogic = fuzzy_logic_controller()

        # Alias Lanes
        lane1 = self.waitingTime["Lane 1"]["Fuzzy"]
        lane2 = self.waitingTime["Lane 2"]["Fuzzy"]
        lane3 = self.waitingTime["Lane 3"]["Fuzzy"]
        lane4 = self.waitingTime["Lane 4"]["Fuzzy"]

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            numVehiclesLane1 = traci.lane.getLastStepVehicleNumber(self._lane1)
            numVehiclesLane3 = traci.lane.getLastStepVehicleNumber(self._lane3)
            numVehiclesLane2 = traci.lane.getLastStepVehicleNumber(self._lane2)
            numVehiclesLane4 = traci.lane.getLastStepVehicleNumber(self._lane4)        

            if traci.trafficlight.getPhase("J2") == 0:
                fuzzyLogic.input['arrivingVehicles'] = numVehiclesLane1 + numVehiclesLane3
                fuzzyLogic.input['queuingVehicles'] = numVehiclesLane2 + numVehiclesLane4

                if numVehiclesLane1 != 0:
                    lane1["steps"].append(step)
                    lane1["waitTime"].append(traci.lane.getWaitingTime(self._lane1) / numVehiclesLane1)
                if numVehiclesLane3 != 0:
                    lane3["steps"].append(step)
                    lane3["waitTime"].append(traci.lane.getWaitingTime(self._lane3) / numVehiclesLane3)

            if traci.trafficlight.getPhase("J2") == 2:
                fuzzyLogic.input['arrivingVehicles'] = numVehiclesLane2 + numVehiclesLane4
                fuzzyLogic.input['queuingVehicles'] = numVehiclesLane1 + numVehiclesLane3

                if numVehiclesLane2 != 0:
                    lane2["steps"].append(step)
                    lane2["waitTime"].append(traci.lane.getWaitingTime(self._lane2) / numVehiclesLane2)
                if numVehiclesLane4 != 0:
                    lane4["steps"].append(step)
                    lane4["waitTime"].append(traci.lane.getWaitingTime(self._lane4) / numVehiclesLane4)

            fuzzyLogic.compute()
            output = fuzzyLogic.output['cycleTime']
            phase = traci.trafficlight.getPhase('J2')

            if (phase == 0 or phase == 2):
                remainingDuration = traci.trafficlight.getNextSwitch('J2') - traci.simulation.getTime()
                traci.trafficlight.setPhaseDuration('J2', min(remainingDuration, output))
            
            step += 1

        traci.close()
        sys.stdout.flush()


    def generate_output_statistics(self, trafficLightType, showGraph = True):

        """
        Generating Statistics for a specific traffic light type

        Parameters
        ----------
        trafficLightType
            The type of traffic light
            It can be "Fixed or "Fuzzy"
        showGraph
            Determines to show the graph or not

        Returns
        -------
        None
        """
        # Writing csv to file
        array2csv(["timestep", "waitingtime-lane1"], self.waitingTime["Lane 1"][trafficLightType],
                  f"outputs/statistics/waitingtime-{trafficLightType}-lane1.csv")
        array2csv(["timestep", "waitingtime-lane2"], self.waitingTime["Lane 2"][trafficLightType],
                  f"outputs/statistics/waitingtime-{trafficLightType}-lane2.csv")
        array2csv(["timestep", "waitingtime-lane3"], self.waitingTime["Lane 3"][trafficLightType],
                  f"outputs/statistics/waitingtime-{trafficLightType}-lane3.csv")
        array2csv(["timestep", "waitingtime-lane4"], self.waitingTime["Lane 4"][trafficLightType],
                  f"outputs/statistics/waitingtime-{trafficLightType}-lane4.csv")


        # Plotting the graph
        if showGraph:
            plot_graph(self.waitingTime["Lane 1"][trafficLightType], "Lane 1")
            plot_graph(self.waitingTime["Lane 2"][trafficLightType], "Lane 2")
            plot_graph(self.waitingTime["Lane 3"][trafficLightType], "Lane 3")
            plot_graph(self.waitingTime["Lane 4"][trafficLightType], "Lane 4")


# this is the main entry point of this script
if __name__ == "__main__":
    options, args = optParser.parse_args()
    traffic = TrafficSimulator(N, STEPS, options.nogui)

    # first, generate the route file for this simulation
    traffic.generate_routefile("traffic.rou.xml")

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs

    traffic.run_fixed()
    traffic.generate_output_statistics("Fixed")
