import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def fuzzy_logic_controller():
    # Input variables and output variable
    arrivingVehicles = ctrl.Antecedent(np.arange(0, 101, 1), 'arrivingVehicles')
    queuingVehicles = ctrl.Antecedent(np.arange(0, 101, 1), 'queuingVehicles')
    cycleTime = ctrl.Consequent(np.arange(0, 101, 1), 'cycleTime')

    # Fuzzy sets defined by triangular membership function
    arrivingVehicles['low'] = fuzz.trimf(arrivingVehicles.universe, [0, 0, 10])
    arrivingVehicles['medium'] = fuzz.trimf(arrivingVehicles.universe, [7, 10, 25])
    arrivingVehicles['high'] = fuzz.trimf(arrivingVehicles.universe, [20, 101, 101])

    queuingVehicles['low'] = fuzz.trimf(queuingVehicles.universe, [0, 0, 10])
    queuingVehicles['medium'] = fuzz.trimf(queuingVehicles.universe, [7, 10, 25])
    queuingVehicles['high'] = fuzz.trimf(queuingVehicles.universe, [20, 101, 101])

    cycleTime['short'] = fuzz.trimf(cycleTime.universe, [0, 0, 10])
    cycleTime['medium'] = fuzz.trimf(cycleTime.universe, [7, 20, 25])
    cycleTime['long'] = fuzz.trimf(cycleTime.universe, [20, 50, 50])

    # Rule Base
    rule1 = ctrl.Rule(arrivingVehicles['high'] & queuingVehicles['low'], cycleTime['long'])
    rule2 = ctrl.Rule(arrivingVehicles['high'] & queuingVehicles['medium'], cycleTime['medium'])
    rule3 = ctrl.Rule(arrivingVehicles['high'] & queuingVehicles['high'], cycleTime['short'])
    rule4 = ctrl.Rule(arrivingVehicles['medium'] & queuingVehicles['low'], cycleTime['long'])
    rule5 = ctrl.Rule(arrivingVehicles['medium'] & queuingVehicles['medium'], cycleTime['medium'])
    rule6 = ctrl.Rule(arrivingVehicles['medium'] & queuingVehicles['high'], cycleTime['short'])
    rule7 = ctrl.Rule(arrivingVehicles['low'] & queuingVehicles['low'], cycleTime['short'])
    rule8 = ctrl.Rule(arrivingVehicles['low'] & queuingVehicles['medium'], cycleTime['medium'])
    rule9 = ctrl.Rule(arrivingVehicles['low'] & queuingVehicles['high'], cycleTime['short'])

    # Inference and Defuzzification
    trafficController = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
    fuzzyLogic = ctrl.ControlSystemSimulation(trafficController)    

    plt.show()
    
    return fuzzyLogic