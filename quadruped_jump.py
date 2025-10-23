import numpy as np
import sys
from env.simulation import QuadSimulator, SimulationOptions

from profiles import FootForceProfile

N_LEGS = 4
N_JOINTS = 3


def quadruped_jump(Jump: str = "Front_jump"):
    # Initialize simulation
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=True,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)
    # Parameters to generate a vertical jump
    # f0: frequency during the impulse (Hz) - higher for a shorter impulse
    # f1: frequency between impulses (Hz) - lower for a longer rest period
    # Fx, Fy: horizontal forces (N) - zero for a pure vertical jump
    # Fz: vertical force (N) - positive to push upward
    front_bias = 1
    side_bias = 1
    Y_foot_offset=0
    if Jump == "Front_jump" : 
        # Force profile for a forward hopping controller maximizing distance
        force_profile = FootForceProfile(f0=1.66753828223525, f1=0.7055836738954222, Fx= 291.68816007097973, Fy=0.0, Fz=393.86642764650566)

        front_bias = 1.1

    elif Jump == "Front_jump_s" :
        # Force profile for a forward hopping controller maximizing speed
        force_profile = FootForceProfile(f0=2.254410954737309, f1=1.7125240994760635, Fx=229.158395950383, Fy=0.0, Fz=222.44100403126365)
        front_bias = 1.1

    elif Jump == "Side_jump_L":
        # Force profile for a side hopping controller maximizing distance
        force_profile = FootForceProfile(f0=1.7, f1=0.8, Fx=0.0, Fy=100, Fz=170)
        side_bias = 1.5
        Y_foot_offset=0.03

    elif Jump == "Side_jump_R":
        # Force profile for a side hopping controller maximizing distance
        force_profile = FootForceProfile(f0=1.7, f1=0.8, Fx=0.0, Fy=-100, Fz=170)
        side_bias = 0.66
        Y_foot_offset=0.03

    elif Jump == "Cw_twist":
        # Force profile for a twist-jump controller maximizing clockwise rotation
        force_profile = FootForceProfile(f0=2.458437941554863, f1=0.6607910025439511, Fx=0.0, Fy=359.11839020913663, Fz=394.07298717610945)
        Y_foot_offset=0.05
        front_bias=0.9

    elif Jump == "Ccw_twist":
        # Force profile for a twist-jump controller maximizing counter-clockwise rotation
        force_profile = FootForceProfile(f0=2.458437941554863, f1= 0.6607910025439511, Fx=0.0, Fy=359.11839020913663, Fz=394.07298717610945)
        Y_foot_offset=0.05
        front_bias=0.9

    #VMC gains
    kvmc = 465.2674868749205

    # Determine number of jumps to simulate
    n_jumps = 10  # Feel free to change this number
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()

    # Total steps of the simulation
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    # Initial position of the robbot
    initial_x = simulator.get_base_position()[0]
    initial_y = simulator.get_base_position()[1]

    # Total time of the simulation
    tot_sim_time = n_steps * sim_options.timestep

    for _ in range(n_steps):
        # If the simulator is closed, stop the loop
        if not simulator.is_connected():
            break

        # Step the oscillator
        force_profile.step(sim_options.timestep)

        # Compute torques as motor targets
        # The convention is as follows:
        # - A 1D array where the torques for the 3 motors follow each other for each leg
        # - The first 3 elements are the hip, thigh, calf torques for the FR leg.
        # - The order of the legs is FR, FL, RR, RL (front/rear,right/left)
        # - The resulting torque array is therefore structured as follows:
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]

        # Initializing torque array
        tau = np.zeros(N_JOINTS * N_LEGS)

        # Applying torque on individual motors
        tau += nominal_position(simulator,Y_foot_offset=Y_foot_offset)
        tau += apply_force_profile(simulator, force_profile, case=Jump, front_bias=front_bias, side_bias=side_bias)
        tau += gravity_compensation(simulator)

        # If a foot is on the ground, we add the virtual model to the torques
        on_ground = np.any(simulator.get_foot_contacts()) 
        if on_ground:
            tau += virtual_model(simulator, k_vmc=kvmc)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()

    #Getting the robot's final position
    final_x = simulator.get_base_position()[0]
    final_y = simulator.get_base_position()[1]
    
    # Close the simulation
    simulator.close()

    # Calculating distances and speeds for the results
    dist_x = final_x - initial_x
    dist_y = final_y - initial_y

    # Commented prints for final positions and speeds
    # print("Simulation time : ", tot_sim_time)
    # print("Distance X : ", dist_x)
    # print("Distance y : ", dist_y)
    # print("Avg:speed X : ", dist_x/tot_sim_time, " m/s")


def nominal_position(
    simulator: QuadSimulator,
    Kp_cartesian: np.ndarray = None,
    Kd_cartesian: np.ndarray = None,
    Y_foot_offset: float = 0
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    Z_foot_pos = -0.15

    # If the gains are not provided, use the default values
    # The default controller values were obtained through controller optimization
    if Kp_cartesian is None:
        Kp_cartesian = np.diag([505.10473362470265, 505.10473362470265, 599.5677387305549])
    if Kd_cartesian is None:
        Kd_cartesian = np.diag([69.9127575693794, 69.9127575693794, 79.9207146504714]) 
    
    for leg_id in range(N_LEGS):
        if leg_id in [0, 2]:
            # Applying desired positions for the right legs
            p_desired = np.array([0., -simulator.config["link_length_hip"]-Y_foot_offset , Z_foot_pos])
        elif leg_id in [1, 3]:
            # Applying desired positions for the left legs
            p_desired = np.array([0., simulator.config["link_length_hip"]+Y_foot_offset, Z_foot_pos])

        # Getting the current position and velocity of the foot
        J, p_current = simulator.get_jacobian_and_position(leg_id)
        
        # Calculating the foot velocity: v = J * q_dot
        q_dot = simulator.get_motor_velocities(leg_id)
        v_current = J @ q_dot
        
        # Calculating the desired Cartesian force 
        F_cartesian = Kp_cartesian @ (p_desired - p_current) + Kd_cartesian @ (- v_current)
        
        # Conversion to motor torques: τ = J^T * F
        tau_i = J.T @ F_cartesian

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    return tau


def virtual_model(
    simulator: QuadSimulator,
    k_vmc: float
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    R = simulator.get_base_orientation_matrix()

    for leg_id in range(N_LEGS):
        # VMC Implementation

        # Positions of the robot's XY plane corners (order: FR, FL, RR, RL)
        corners = np.array([[1, 1, -1, -1], [-1, 1, -1, 1], [0, 0, 0, 0]])  # 3x4
        
        # XY plane in the world frame: P = R * corners
        P = R @ corners
        
        # VMC force: F_VMC = [0; 0; k_VMC * P[2, leg_id]] (équation 9)
        F_vmc = np.zeros((3, 4))
        F_vmc[2, :] = k_vmc * np.array([0., 0., 1.]) @ P

        # Conversion to torques via the Jacobian: τ = J^T * F
        J, _ = simulator.get_jacobian_and_position(leg_id)
        tau_i = J.T @ F_vmc[:, leg_id]

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


def gravity_compensation(
    simulator: QuadSimulator
) -> np.ndarray:
    # All motor torques are in a single array
    tau = np.zeros(N_JOINTS * N_LEGS)
    for leg_id in range(N_LEGS):

        # Gravity compensation: τ = J^T * F_gravity
        
        # Gravity force for a leg
        mass_robot = simulator.get_mass()
        weight_per_leg = - mass_robot * 9.81 / N_LEGS
        
        # Vertical gravity force (negative Z direction in the world frame)
        F_gravity = np.array([0.0, 0.0, weight_per_leg])
        
        # Conversion to motor torques via the Jacobian
        J, _ = simulator.get_jacobian_and_position(leg_id)
        tau_i = J.T @ F_gravity

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i

    return tau


def apply_force_profile(
    simulator: QuadSimulator,
    force_profile: FootForceProfile,
    case: str,
    front_bias: float = 1,
    side_bias: float = 1,
) -> np.ndarray:
    tau = np.zeros(N_JOINTS * N_LEGS)
    force = force_profile.force().copy()
    
    for leg_id in range(N_LEGS):
        force_leg = force.copy()
        
        # Applying bias on legs
        if(leg_id == 0):
            force_leg[2] *= front_bias / side_bias
        
        elif(leg_id == 1):
            force_leg[2] *= front_bias * side_bias
        
        elif(leg_id == 2):
            force_leg[2] *= 1/(side_bias * front_bias)

        elif(leg_id == 3):
            force_leg[2] *= side_bias / front_bias
        
        #Back legs push in oposite direction for Ccw_twist
        if case == "Ccw_twist": 
            if leg_id in [2, 3]: 
                force_leg[1] = -force_leg[1]  
        
        #Front legs push in oposite direction for Cw_twist
        elif case == "Cw_twist":
            if leg_id in [0, 1]:  
                force_leg[1] = -force_leg[1]
        
        # Conversion to motor torques via the Jacobian
        J, _ = simulator.get_jacobian_and_position(leg_id)   
        tau_i = J.T @ force_leg  

        # Store in torques array
        tau[leg_id * N_JOINTS : leg_id * N_JOINTS + N_JOINTS] = tau_i
    
    return tau


if __name__ == "__main__":
    # Define valid jump types
    valid_jumps = ["Front_jump", "Front_jump_s", "Side_jump_L", "Side_jump_R", "Cw_twist", "Ccw_twist"]
    
    # Get jump type from command line or user input
    if len(sys.argv) < 2:
        jump_type = input(f"Input the jump type, options are: {', '.join(valid_jumps)}\n")
    else:
        jump_type = sys.argv[1]
    
    # Validate jump type
    if jump_type not in valid_jumps:
        print(f"Invalid jump type: {jump_type}")
        print(f"Valid options are: {', '.join(valid_jumps)}")
        sys.exit(1)
    
    # Run simulation
    quadruped_jump(jump_type)