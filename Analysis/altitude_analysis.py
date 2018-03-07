# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 19:34:31 2015

@author: Tennessee
"""

import numpy as np
import matplotlib.pyplot as plt

def altitude(atm_hpa, sea_level_hpa):
    return 44330 * (1.0 - np.power(atm_hpa / sea_level_hpa, 0.1903))
    
def plot_alt():
    default_msl = 101300.0
    
    pressure = np.linspace(97772.58 / 100.0, 79495.0 / 100.0, 2000)
    alt_nominal = altitude(pressure, default_msl) - altitude(97772.58 / 100.0, default_msl)
    alt_too_high = altitude(pressure, default_msl + (1000 / 100.0)) - altitude(97772.58 / 100.0, default_msl + (1000 / 100.0))
    alt_too_low = altitude(pressure, default_msl - (1000 / 100.0)) - altitude(97772.58 / 100.0, default_msl - (1000 / 100.0))
    
    f1 = plt.figure()
    ax = f1.gca()
    ax.plot(pressure, alt_nominal, "b-", label="nom")
    ax.plot(pressure, alt_too_high, "r-", label="high")
    ax.plot(pressure, alt_too_low, "g-", label="low")
    
    ax.legend()
    
    f1.show()
    
    f2 = plt.figure()
    ax = f2.gca()

    ax.plot(pressure, alt_too_high - alt_nominal, "r-", label="high")
    ax.plot(pressure, alt_too_low - alt_nominal, "g-", label="low")
    
    ax.legend()
    
    f2.show()
    
    
    
