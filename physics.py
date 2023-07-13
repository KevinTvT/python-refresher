"""Problem 1:
Write a Python function, calculate_buoyancy, that calculates the buoyancy force exerted on a object submerged in water.
The function should take the following arguments:
V: the volume of the object in m^3
density_fluid: the density of the fluid in kg/m^3
Use the formula above to calculate the buoyancy force. g is a constant equal to 9.81 m/s^2
The function should return the buoyancy force in Newtons."""
g = 9.81  # m/s^2


def calculate_buoyancy(V, density_fluid):
    return V * density_fluid * g  # Newtons or kg*m/s^2


"""Problem 2:
Write a Python function, will_it_float, that determines whether an object will float or sink in water.
The function should take the following arguments:
V: the volume of the object in cubic meters,
mass: the mass of the object in kilograms.
The function should return True if the object will float and False if the object will sink."""
water_density = 1000  # kg/m^3


def will_it_float(V, mass):
    return True if V * water_density > mass else False


"""Problem 3:
Write a Python function, calculate_pressure, that calculates the pressure at a given depth in water.
The function should take the following arguments:
depth: the depth in meters.
Use the formula above to calculate the pressure. g is a constant equal to 9.81^m/s^2
and the density of water is 1000 kg/m^3
The function should return the pressure in Pascals."""


def calculate_pressure(depth):
    return water_density * g * depth  # Pascals or N/m^2
