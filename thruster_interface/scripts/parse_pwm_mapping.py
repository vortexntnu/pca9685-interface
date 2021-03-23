#!/usr/bin/env python

from openpyxl import load_workbook
import numpy as np


def create_pwm_lookup_function(path_to_mapping):
    """Creates a function that finds a good pwm value for a 
    desired thrust and voltage level. The function parses a file with mappings from 
    desired thrust to pwm values for blue robotics T200 thrusters at different voltage levels. 

    Args:
        path_to_mapping (String): [description]
    """
    workbook = load_workbook(filename=path_to_mapping)
    
    voltages = [10, 12, 14, 16, 18, 20]
    
    # pwm values are the same for all voltages
    pwm_values = [cell[0].value for cell in workbook['10 V']['A2':'A202']]
    
    # * 9.8 for converting from kg f to newton
    thrusts_dict = dict()
    for voltage in voltages:
        thrusts_dict[voltage] = [cell[0].value * 9.8 for cell in workbook['%i V' % voltage]['F2':'F202']]
    
    new_voltage_steps = np.round(np.arange(10, 20.01, 0.1), decimals=1)
    thrusts_from_voltage = dict()
    for voltage in new_voltage_steps:
        thrusts_from_voltage[voltage] = []
        
    for i in range(len(pwm_values)):
        pwm = pwm_values[i]
        thrusts = [thrusts_dict[voltage][i] for voltage in voltages]
        
        # create interpolation function
        interpolated_thrusts = np.interp(new_voltage_steps, voltages, thrusts)
        
        # save interpolated thrusts
        for (voltage, thrust) in zip(new_voltage_steps, interpolated_thrusts):
            thrusts_from_voltage[voltage].append(thrust)
            
    def pwm_lookup(thrust, voltage):
        voltage_rounded = np.round(voltage, decimals=1)
        return np.interp(thrust, thrusts_from_voltage[voltage_rounded], pwm_values)
    
    return pwm_lookup
        

if __name__ == '__main__':
    pwm_lookup = create_pwm_lookup_function(
        'thruster_interface/T200-Public-Performance-Data-10-20V-September-2019.xlsx'
    )
    print(pwm_lookup(20.2, 15.17778))
