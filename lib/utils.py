# autor: Diego Roldan Urueña - 841723
# fecha: 09/02/2025

import math

def rads_to_deg(rads):
    return rads * 180 / math.pi

def deg_to_rads(deg):
    return deg * math.pi / 180

def loc_to_deg(loc):
    return [loc[0], loc[1], rads_to_deg(loc[2])]
