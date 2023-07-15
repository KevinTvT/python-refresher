import math
import numpy as np

"""CREATED A ROUNDING FUNCTION TO THREE DECIMAL PLACES"""


def myRound(n):
    return round(n, 3)


"""Problem 1:
Write a Python function, calculate_buoyancy, that calculates the buoyancy force exerted on a object submerged in water.
The function should take the following arguments:
V: the volume of the object in m^3
density_fluid: the density of the fluid in kg/m^3
Use the formula above to calculate the buoyancy force. g is a constant equal to 9.81 m/s^2
The function should return the buoyancy force in Newtons."""
g = 9.81  # m/s^2


def calculate_buoyancy(V, density_fluid):
    if V < 0:
        raise ValueError("Volume cannot be less than 0")
    return V * density_fluid * g  # Newtons or kg*m/s^2


"""Problem 2:
Write a Python function, will_it_float, that determines whether an object will float or sink in water.
The function should take the following arguments:
V: the volume of the object in cubic meters,
mass: the mass of the object in kilograms.
The function should return True if the object will float and False if the object will sink."""
water_density = 1000  # kg/m^3


def will_it_float(V, mass):
    if V < 0:
        raise ValueError("Volume cannot be less than 0")
    if V * water_density == mass:
        return "NEUTRAL BUOYANCY"
    return V * water_density > mass


"""Problem 3:
Write a Python function, calculate_pressure, that calculates the pressure at a given depth in water.
The function should take the following arguments:
depth: the depth in meters.
Use the formula above to calculate the pressure. g is a constant equal to 9.81^m/s^2
and the density of water is 1000 kg/m^3
The function should return the pressure in Pascals."""


def calculate_pressure(depth):
    surface = 101325
    if depth <= 0:
        raise ValueError("Volume cannot be less than 0")
    return water_density * g * depth + surface  # Pascals or N/m^2


"""Problem 4
Write a Python function, calculate_acceleration, that calculates the acceleration of an object given the force applied to it and its mass.
The function should take the following arguments:
F: the force applied to the object in Newtons,
m: the mass of the object in kilograms.
"""


def calculate_acceleration(force, mass):
    if mass <= 0:
        raise ValueError("Mass cannot be less than or equal to 0")
    return force / mass


"""Problem 5
Write a Python function, calculate_angular_acceleration, that calculates the angular acceleration of an object given the torque applied to it and its moment of inertia.
The function should take the following arguments:
tau: the torque applied to the object in Newton-meters,
I: the moment of inertia of the object in kg/m^2"""


def calculate_angular_acceleration(tau, inertia):
    if inertia <= 0:
        raise ValueError("Inertia cannot be less than or equal to 0")
    return tau / inertia


"""Problem 6
Write a Python function, calculate_torque, that calculates the torque applied to an object given the force applied to it and the distance from the axis of rotation to the point where the force is applied.
The function should take the following arguments:
F_magnitude: the magnitude of force applied to the object in Newtons,
F_direction: the direction of the force applied to the object in degrees,
r: the distance from the axis of rotation to the point where the force is applied in meters."""


def calculate_torque(F_magnitude, F_direction, r):
    if r <= 0:
        raise ValueError("Distance is a magnitude cannot be negative")
    return myRound(F_magnitude * r * math.sin(math.radians(F_direction)))


"""Problem 7
Write a Python function, calculate_moment_of_inertia, that calculates the moment of inertia of an object given its mass and the distance from the axis of rotation to the center of mass of the object.
The function should take the following arguments:
m: the mass of the object in kilograms,
r: the distance from the axis of rotation to the center of mass of the object in meters."""


def calculate_moment_of_inertia(mass, r):
    if mass <= 0:
        raise ValueError("Mass cannot be negative or 0")
    return mass * r**2


"""Problem 8 (part 1)
An AUV is submerged in water. The AUV has a mass of 100 kg and a volume of 0.1 m^3
The AUV is equipped with a thruster that can apply a force up to 100N. The thruster is located 0.5m from the center of mass of the AUV. The thruster can rotate 30 degrees in either direction of the x-axis.

Write a Python function, calculate_auv_acceleration, that calculates the acceleration of the AUV in the 2D plane.
The function should take the following arguments:
F_magnitude: the magnitude of force applied by the thruster in Newtons.
F_angle: the angle of the force applied by the thruster in radians. The angle is measured from the x-axis. Positive angles are measured in the counter-clockwise direction.
mass (optional): the mass of the AUV in kilograms. The default value is 100kg.
volume (optional): the volume of the AUV in cubic meters. The default value is 0.1 m^3
thruster_distance (optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5m.
The function should return the acceleration of the AUV in a tuple (x, y) in meters per second squared."""


def calculate_auv_acceleration(
    F_magnitude, F_angle, mass=100, volume=0.1, thruster_distance=0.5
):
    if mass <= 0 or volume <= 0:
        raise ValueError("Mass or volume cannot be negative or 0")
    return (
        F_magnitude * math.cos(F_angle) / mass,
        F_magnitude * math.sin(F_angle) / mass,
    )


"""Problem 8 (part 2)
Write a Python function, calculate_auv_angular_acceleration, that calculates the angular acceleration of the AUV.
The function should take the following arguments:
F_magnitude: the magnitude of force applied by the thruster in Newtons.
F_angle: the angle of the force applied by the thruster in radians.
inertia (optional): the moment of inertia of the AUV in kg/m^2. The default value is 0.1 m^3
thruster_distance (optional): the distance from the center of mass of the AUV to the thruster in meters. The default value is 0.5 m
The function should return the angular acceleration of the AUV in radians per second squared."""


def calculate_auv_angular_acceleration(
    F_magnitude, F_angle, inertia=0.1, thruster_distance=0.5
):
    if inertia <= 0:
        raise ValueError("Inertia cannot be negative or 0")
    tau = calculate_torque(F_magnitude, math.degrees(F_angle), thruster_distance)
    print(tau)
    angular_acceleration = calculate_angular_acceleration(tau, inertia)
    return myRound(angular_acceleration)


"""Problem 9
Write a Python function, calculate_auv2_acceleration, that calculates the acceleration of the AUV in the 2D plane.
The function should take the following arguments:
T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
alpha: the angle of the thrusters in radians.
theta: the angle of the AUV in radians.
mass (optional): the mass of the AUV in kilograms. The default value is 100kg.
Will return a tuple of the forces in the X-direction and Y-direction in m/s^2"""


def calculate_auv2_acceleration(forces, alpha, theta, mass=100):
    if mass <= 0:
        raise ValueError("Mass cannot be less than zero")
    alpha_rotation_matrix = np.copy(
        [
            [math.cos(alpha), math.cos(alpha), -math.cos(alpha), -math.cos(alpha)],
            [math.sin(alpha), -math.sin(alpha), -math.sin(alpha), math.sin(alpha)],
        ]
    )
    theta_rotation_matrix = np.copy(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    )
    world_forces = np.matmul(alpha_rotation_matrix, forces)
    return np.matmul(world_forces, theta_rotation_matrix)


"""Write a Python function, calculate_auv2_angular_acceleration, that calculates the angular acceleration of the AUV.
The function should take the following arguments:
T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
alpha: the angle of the thrusters in radians.
L: the distance from the center of mass of the AUV to the thrusters in meters.
l: the distance from the center of mass of the AUV to the thrusters in meters.
inertia (optional): the moment of inertia of the AUV in kg⋅m^2. The default value is 100kg⋅m^2
The function should return the angular acceleration of the AUV in radians per second squared."""


def calculate_auv2_angular_acceleration(forces, alpha, L, l, inertia=100):
    if L <= 0 or l <= 0 or inertia <= 0:
        raise ValueError(
            "Distance is a magnitude cannot be 0 or negative, inertia cannot be 0 or negative"
        )
    netForce = 0
    for x in range(0, len(forces)):
        netForce += forces[x] if x % 2 == 0 else -forces[x]
    netTau = (L * math.sin(alpha) + l * math.cos(alpha)) * (netForce)
    return netTau / inertia
