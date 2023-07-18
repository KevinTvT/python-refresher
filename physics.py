import numpy as np
import matplotlib.pyplot as plt

"""CREATED A ROUNDING FUNCTION TO THREE DECIMAL PLACES"""


def myRound(n):
    return np.round(n, 3)


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
    return myRound(F_magnitude * r * np.sin(np.deg2rad(F_direction)))


# math.radians(float) is the same as np.deg2rad(float)

"""Problem 7
Write a Python function, calculate_moment_of_inertia, that calculates the moment of inertia of an object given its mass and the distance from the axis of rotation to the center of mass of the object.
The function should take the following arguments:
m: the mass of the object in kilograms,
r: the distance from the axis of rotation to the center of mass of the object in meters."""


def calculate_moment_of_inertia(mass, r):
    if mass <= 0:
        raise ValueError("Mass cannot be negative or 0")
    return mass * np.power(r, 2)


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
    return np.array(
        [
            F_magnitude * np.cos(F_angle) / mass,
            F_magnitude * np.sin(F_angle) / mass,
        ]
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
    tau = calculate_torque(F_magnitude, np.rad2deg(F_angle), thruster_distance)
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
    if not np.shape(forces) == (4,):
        raise ValueError("The shape of the array is not right")
    projection_matrix = np.copy(
        [
            [np.cos(alpha), np.cos(alpha), -np.cos(alpha), -np.cos(alpha)],
            [np.sin(alpha), -np.sin(alpha), -np.sin(alpha), np.sin(alpha)],
        ]
    )
    rotation_matrix = np.copy(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
    )
    world_forces = np.matmul(projection_matrix, forces)
    return np.matmul(rotation_matrix, world_forces) / mass


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
    """netForce = 0
    for x in range(0, len(forces)):
        netForce += forces[x] if x % 2 == 0 else -forces[x]"""
    projection_array = np.array([1, -1, 1, -1]).T
    netForce = np.dot(projection_array, forces)
    netTau = (L * np.sin(alpha) + l * np.cos(alpha)) * netForce
    return netTau / inertia


"""Write a Python function, simulate_auv2_motion, that simulates the motion of the AUV in the 2D plane.
The function should take the following arguments:
T: an np.ndarray of the magnitudes of the forces applied by the thrusters in Newtons.
alpha: the angle of the thrusters in radians.
L: the distance from the center of mass of the AUV to the thrusters in meters.
l: the distance from the center of mass of the AUV to the thrusters in meters.
mass (optional): the mass of the AUV in kg. The default value is 100 kg.
inertia (optional): the moment of inertia of the AUV in kg⋅m^2. The default value is 100kg⋅m^2.
dt (optional): the time step of the simulation in seconds. The default value is 0.1s.
t_final (optional): the final time of the simulation in seconds. The default value is 10s.
x0 (optional): the initial x-position of the AUV in meters. The default value is 0m.
y0 (optional): the initial y-position of the AUV in meters. The default value is 0m.
theta0 (optional): the initial angle of the AUV in radians. The default value is 0rad.

The function should return the following:
t: an np.ndarray of the time steps of the simulation in seconds.
x: an np.ndarray of the x-positions of the AUV in meters.
y: an np.ndarray of the y-positions of the AUV in meters.
theta: an np.ndarray of the angles of the AUV in radians.
v: an np.ndarray of the velocities of the AUV in meters per second.
omega: an np.ndarray of the angular velocities of the AUV in radians per second.
a: an np.ndarray of the accelerations of the AUV in meters per second squared."""


def simulate_auv2_motion(
    T, alpha, L, l, mass=100, inertia=100, dt=0.1, t_final=10, x0=0, y0=0, theta0=0
) -> tuple:
    if L <= 0 or l <= 0 or mass <= 0 or inertia <= 0 or dt <= 0 or t_final <= 0:
        raise ValueError("L, l, mass inertia, dt, or t_final is negative")
    time = np.arange(0, t_final, dt)
    x = np.zeros_like(time)
    y = np.zeros_like(time)
    theta = np.zeros_like(time)
    omega = np.zeros_like(time)
    v = np.zeros((len(time), 2))
    a = np.zeros((len(time), 2))

    x[0] = x0
    y[0] = y0

    angular_acceleration = calculate_auv2_angular_acceleration(
        T, alpha, L, l, inertia=inertia
    )
    a[0] = calculate_auv2_acceleration(T, alpha, theta0, mass=mass)

    theta[0] = theta0

    for t in range(1, len(time)):
        # POSITION STUFF
        a[t] = calculate_auv2_acceleration(T, alpha, theta[t - 1], mass=mass)
        v[t] = a[t - 1] * dt + v[t - 1]
        x[t] = v[t - 1][0] * dt + x[t - 1]
        y[t] = v[t - 1][1] * dt + y[t - 1]

        # ANGLE STUFF
        omega[t] = angular_acceleration * dt + omega[t - 1]
        theta[t] = omega[t - 1] * dt + theta[t - 1]

    return [time, x, y, theta, v, omega, a]


"""Write a Python function, plot_auv2_motion, that plots the motion of the AUV in the 2D plane. No need to test this function. You can try it out in the Jupyter Notebook.
The function should take the following arguments:
t: an np.ndarray of the time steps of the simulation in seconds.
x: an np.ndarray of the x-positions of the AUV in meters.
y: an np.ndarray of the y-positions of the AUV in meters.
theta: an np.ndarray of the angles of the AUV in radians.
v: an np.ndarray of the velocities of the AUV in meters per second.
omega: an np.ndarray of the angular velocities of the AUV in radians per second.
a: an np.ndarray of the accelerations of the AUV in meters per second squared."""


def plot_auv2_motion(t, x, y, theta, v, omega, a):
    plt.plot(t, x, label="X-Position")
    plt.plot(t, y, label="Y-Position")
    """plt.plot(t, theta, label="Theta")
    plt.plot(t, v, label="Velocity")
    plt.plot(t, omega, label="Omega")
    plt.plot(t, a, label="Acceleration")"""
    plt.xlabel("Time (s)")
    plt.legend()
    plt.show()
