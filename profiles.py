import numpy as np


class FootForceProfile:
    """Class to generate foot force profiles over time using a single CPG oscillator"""

    def __init__(self, f0: float, f1: float, Fx: float, Fy: float, Fz: float):
        """
        Create instance of foot force profile with its arguments.

        Args:
            f0 (float): Frequency of the impulse (Hz)
            f1 (float): Frequency between impulses (Hz)
            Fx (float): Foot force amplitude in X direction (N)
            Fy (float): Foot force amplitude in Y direction (N)
            Fz (float): Foot force amplitude in Z direction (N)
        """
        self.theta = 0
        self.f0 = f0
        self.f1 = f1
        self.F = np.array([Fx, Fy, Fz])

    def step(self, dt: float):
        """
        Step the oscillator by a single timestep.

        Args:
            dt (float): Timestep duration (s)
        """
        # TODO: Pas sûr ?? integrate the oscillator equation
        # Determine frequency based on PREVIOUS phase value
        if np.pi <= self.theta < 2 * np.pi:
            fi = self.f0  # Impulse phase
        else:  # 0 <= theta < pi
            fi = self.f1  # Idle phase
        
        # Calculate θ̇ = 2π * fi using the previous theta value
        theta_dot = 2 * np.pi * fi
        
        # Integrate to get new theta: θ(t+dt) = θ(t) + θ̇ * dt
        self.theta += theta_dot * dt
        
        # Keep theta in [0, 2π] range
        self.theta = self.theta % (2 * np.pi)

    def phase(self) -> float:
        """Get oscillator phase in [0, 2pi] range."""
        # Return the phase of the oscillator in [0, 2pi] range
        return self.theta % (2 * np.pi)

    def force(self) -> np.ndarray:
        """
        Get force vector of the force profile at the current timestep.

        Returns:
            np.ndarray: An R^3 array [Fx, Fy, Fz]
        """
        # Calcul de sin(theta)
        sin_theta = np.sin(self.theta)
        
        # Application de l'équation (5) du document
        if sin_theta < 0:
            # Phase d'impulsion : F = [Fx, Fy, Fz]^T * sin(θ)
            return self.F * sin_theta
        else:
            # Phase idle : F = [0, 0, 0]^T
            return np.zeros(3)

    def impulse_duration(self) -> float:
        """Return impulse duration in seconds."""
        # TODO: DONE compute the impulse duration in seconds
        impulse_time = 1/(2*self.f0)
        return impulse_time

    def idle_duration(self) -> float:
        """Return idle time between impulses in seconds"""
        # TODO: DONE compute the idle duration in seconds
        idle_time = 1/(2*self.f1)
        return idle_time
