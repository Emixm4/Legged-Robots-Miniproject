import optuna
import numpy as np
from functools import partial
from optuna.trial import Trial
from env.simulation import QuadSimulator, SimulationOptions
import matplotlib.pyplot as plt
import sys

from profiles import FootForceProfile

from quadruped_jump import (
    nominal_position,
    gravity_compensation,
    apply_force_profile,
    virtual_model,
)


N_LEGS = 4
N_JOINTS = 3


def quadruped_jump_optimization(T_type: str = "opt_front", n_opt_trials: int = 50):
    # Initialize simulation
    sim_options = SimulationOptions(
        on_rack=False,  # Whether to suspend the robot in the air (helpful for debugging)
        render=False,  # Whether to use the GUI visualizer (slower than running in the background)
        record_video=False,  # Whether to record a video to file (needs render=True)
        tracking_camera=True,  # Whether the camera follows the robot (instead of free)
    )
    simulator = QuadSimulator(sim_options)

    # Create a maximization problem
    objective = partial(evaluate_jumping, simulator=simulator, T_type=T_type)
    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(
        study_name="Quadruped Jumping Optimization",
        sampler=sampler,
        direction="maximize",
    )

    # Run the optimization
    # You can change the number of trials here
    study.optimize(objective, n_trials=n_opt_trials)

    # Close the simulation
    simulator.close()

    # Log the results
    print("Best value:", study.best_value)
    print("Best params:", study.best_params)


def evaluate_jumping(
    trial: Trial, 
    simulator: QuadSimulator, 
    T_type: str
) -> float:

    #Controller values found through optimization
    Kp_xy = 505.10473362470265
    Kp_z = 599.5677387305549
    Kd_xy = 69.9127575693794
    Kd_z = 79.9207146504714 
    K_vmc = 465.2674868749205
    
    front_bias=1
    Y_foot_offset=0
    side_bias = 1
    if T_type=="opt_stab" :
        #Optimisation of the stabilisation to find K_p, K_d and K_vmc
        jump_type = "Front_jump"

        K_vmc = trial.suggest_float(name="K_vmc", low=400.0, high=500.0)
        Kp_xy = trial.suggest_float(name="Kp_xy", low=500.0, high=600.0)
        Kp_z = trial.suggest_float(name="Kp_z", low=500.0, high=600.0)
        Kd_xy = trial.suggest_float(name="Kd_xy", low=60.0, high=80.0)
        Kd_z = trial.suggest_float(name="Kd_z", low=60.0, high=80.0)

        f0 = 1.50
        f1 = 0.8  
        Fx = 0.0   
        Fy = 0.0
        Fz = 120.0 
    
    elif T_type=="opt_front" :
        #Optimisation of the forward jump
        jump_type = "Front_jump"

        #Values to optimize for front jump
        f0 = trial.suggest_float(name="f0", low=1, high=2) 
        f1 = trial.suggest_float(name="f1", low=0.3, high=0.8)
        Fx = trial.suggest_float(name="Fx", low=200, high=300)
        Fz = trial.suggest_float(name="Fz", low=300, high=400)

        #Fixed values for front jump optimisation
        Fy = 0.0 
        front_bias = 1.1 

    elif T_type=="opt_side" :
        #Optimisation of the Side jump
        jump_type = "Side_jump_L"

        #Values to optimize for side jump
        f0 = trial.suggest_float(name="f0", low=1.7, high=2.3)
        f1 = trial.suggest_float(name="f1", low=0.6, high=0.9)
        Fy = trial.suggest_float(name="Fy", low=80.0, high=100.0)
        Fz = trial.suggest_float(name="Fz", low=140.0, high=170.0)
        
        #Fixed values for side jump optimisation
        Fx = 0.0
        side_bias = 1.5

    elif T_type=="opt_twist" :
        #Optimisation of the twist jump
        jump_type="Cw_twist"

        #Values to optimize for side twist jump
        f0 = trial.suggest_float(name="f0", low=2, high=4) # Vitesse de saut
        f1 = trial.suggest_float(name="f1", low=0.4, high=0.8) # Repos 
        Fy = trial.suggest_float(name="Fy", low=270, high=360)
        Fz = trial.suggest_float(name="Fz", low=350, high=400) # Force verticale

        #Fixed values for twist jump optimisation
        Fx = 0.0
        front_bias = 0.9
        Y_foot_offset = 0.05
    
    elif T_type=="opt_speed" :
        #Optimisation of the fastest forward jump
        jump_type = "Front_jump"

        #Values to optimize for fastest forward jump
        f0 = trial.suggest_float(name="f0", low=2, high=2.5)
        f1 = trial.suggest_float(name="f1", low=1.3, high=1.8)
        Fx = trial.suggest_float(name="Fx", low=150, high=250)
        Fz = trial.suggest_float(name="Fz", low=150, high=250)

        #Fixed values for fastest forward jump
        Fy = 0
        front_bias = 1.1 

    else :
        f0 = 1.50
        f1 = 0.8 
        Fx = 0.0
        Fy = 0.0
        Fz = 120.0
        
    
    # Create the gain matrices
    Kp_cartesian = np.diag([Kp_xy, Kp_xy, Kp_z])
    Kd_cartesian = np.diag([Kd_xy, Kd_xy, Kd_z])
    
    # Reset the simulation
    simulator.reset()
    
    # Save the initial positions
    initial_x = simulator.get_base_position()[0]
    initial_y = simulator.get_base_position()[1]

    # Extract simulation options
    sim_options = simulator.options

    # Setting parameters for the foot force profile
    force_profile = FootForceProfile(f0=f0, f1=f1, Fx=Fx, Fy=Fy, Fz=Fz)
    
    # Variables for evaluation
    fallen = False
    flight_stab = 0.0
    flight_stps = 0
    nb_yaw_full_rot = -1
    cnt_yaw_adder = 0
    
    # List of all the robot orientations during simulation 
    # [:][0] is roll, [:][1] is pitch, [:][2] is yaw
    orientations = []
    orientations.append(simulator.get_base_orientation_roll_pitch_yaw())
    

    # Determine number of jumps to simulate
    n_jumps = 10

    # Calculation of the number of steps
    jump_duration = force_profile.impulse_duration() + force_profile.idle_duration()
    n_steps = int(n_jumps * jump_duration / sim_options.timestep)

    for step in range(n_steps):
        force_profile.step(sim_options.timestep)

        # Calculation of the different torques using the functions from quadruped_jump.py
        tau = np.zeros(N_JOINTS * N_LEGS)
        tau += nominal_position(simulator, Kp_cartesian=Kp_cartesian, Kd_cartesian=Kd_cartesian, Y_foot_offset=Y_foot_offset)
        tau += apply_force_profile(simulator, force_profile, case=jump_type, front_bias=front_bias, side_bias=side_bias)
        tau += gravity_compensation(simulator)

         # If a foot is on the ground, we add the virtual model to the torques
        on_ground = np.any(simulator.get_foot_contacts())
        if on_ground:
            tau += virtual_model(simulator, k_vmc=K_vmc)

        # Set the motor commands and step the simulation
        simulator.set_motor_targets(tau)
        simulator.step()
        
        # Getting position and orientation of the robot
        base_pos = simulator.get_base_position()
        orientations.append(simulator.get_base_orientation_roll_pitch_yaw())
        
        # Flight phase calculation
        if not on_ground and base_pos[2] > 0.12:
            flight_stps += 1
            if abs(orientations[-1][0]) < 0.3 and abs(orientations[-1][1]) < 0.3:
                flight_stab += 1.0
        
        # Fall criteria : stops if robot has fallen
        if abs(orientations[-1][0]) > 1.3 or abs(orientations[-1][1]) > 1.3 or base_pos[2] < 0.05: # Trop incliné ou trop bas
            fallen = True
            break
        
        # Calculates every 20 steps whether the robot has completed a full turn around the yaw axiss
        if(cnt_yaw_adder>20) : 
            orientations[-1][2] = simulator.get_base_orientation_roll_pitch_yaw()[2]
            cnt_yaw_adder=0

            if((orientations[-1][2] < 0) and (orientations[-2][2] > 0) and 
               (abs(orientations[-1][2] - orientations[-2][2]) < 3.14)) :
                nb_yaw_full_rot += 1

            elif((orientations[-1][2] > 0) and (orientations[-2][2] < 0) and 
                 (abs(orientations[-1][2] - orientations[-2][2]) < 3.14)) :
                nb_yaw_full_rot -= 1
        
        cnt_yaw_adder +=1

    if fallen :
        print("Robot fall")
        return -20.0
    
    # Getting the final position of the robot and calculating the distances traveled
    final_y = simulator.get_base_position()[1]
    final_x = simulator.get_base_position()[0]
    dist_x = final_x - initial_x
    dist_y = final_y - initial_y

    # Calculation of the maximum pitch and roll angles
    max_pitch = np.max(np.abs(np.array(orientations)[:, 1]))
    max_roll = np.max(np.abs(np.array(orientations)[:, 0]))
    
    # Calculation of the total yaw rotation angle in radians
    rot_tot = 0
    if(orientations[-1][2] >= 0) :
        rot_tot = np.pi
    rot_tot += nb_yaw_full_rot * 2 * np.pi + abs(orientations[-1][2])

    # Calculation of the different objective functions for the various optimizations
    if T_type == "opt_stab":
        # Call to the function calculating the objective function for stabilization optimization
        score = stab_objective(dist_y, orientations)

    elif T_type=="opt_front" :
        # Calculation of the objective function for the forward jump
        score = dist_x - 3*abs(dist_y) - 3*max_pitch

    elif T_type=="opt_side" :
        # Call to the function calculating the objective function for side jump optimization
        score = side_objective(dist_x, dist_y, orientations, max_pitch, flight_stps, 
                               flight_stab)
    

    elif T_type=="opt_twist" :
        # print("Total rotation : ",rot_tot)
        # print("Total x : ",abs(dist_x))
        # print("Total y : ",abs(dist_y))
        # Calculation of the objective function for the twist jump
        score = rot_tot - 30*abs(dist_x) - 30*abs(dist_y) - 20*max_pitch
    
    elif T_type=="opt_speed" :
        print("total steps : ", n_steps)
        # Calculation of the objective function for the speed jump
        score = (10000*dist_x)/n_steps - 3*abs(dist_y) - 6*max_pitch

    return score


def stab_objective(dist_y, orientations) -> float:
    orientations_array = np.array(orientations)

    max_roll = np.max(np.abs(orientations_array[:, 0]))
    max_pitch = np.max(np.abs(orientations_array[:, 1]))
    mean_roll = np.mean(np.abs(orientations_array[:, 0]))
    mean_pitch = np.mean(np.abs(orientations_array[:, 1]))
    
    roll_std = np.std(orientations_array[:, 0])
    pitch_std = np.std(orientations_array[:, 1])

    stab_score = -(20.0*max_roll + 20.0*max_pitch + 10.0*mean_roll + 
                   10.0*mean_pitch + 15.0*roll_std + 15.0*pitch_std + 5.0*dist_y)
    
    return stab_score

def side_objective(
    dist_x, 
    dist_y, 
    orientations,
    max_pitch, 
    flight_stps, 
    flight_stab
) -> float:
    score = 0
    smoothness_score = 0
    tot_steps = len(orientations)

    # Calculating the smoothness of the roll and pitch angle change
    for i in range(tot_steps-1):
        pitch_change = abs(orientations[i][1] - orientations[i+1][1])
        roll_change = abs(orientations[i][0] - orientations[i+1][0])
        if pitch_change < 0.05 and roll_change < 0.05:     
            smoothness_score += 1.0

    if dist_y < 0.12:
            score = -5.0
    else:
        # Lateral distance (70 points) – main criterion
        score += 70.0 * abs(dist_y)

        # Flight stability (25 points)
        if flight_stps > 0:
            score += 25.0 * (flight_stab / flight_stps)
        
        # Smoothness (10 points)
        score += 10.0 * (smoothness_score / tot_steps)
        
        # Maximum pitch (5 points)
        if max_pitch < 0.15 :
            score += 5.0
        elif max_pitch < 0.3 :
            score += 3.0
        
        # Penalty: forward drift (10 points)
        score -= 10.0 * min(abs(dist_x) / 0.2, 1.0)
    return score
    

if __name__ == "__main__":
    # Define valid optimization types
    valid_opt = ["opt_stab", "opt_front", "opt_side", "opt_twist", "opt_speed"]
    
    # Get optimization type from command line or user input
    if len(sys.argv) < 2:
        opt_type = input(f"Input the optimisiation type options are: {', '.join(valid_opt)} \n")
    else :
        opt_type = sys.argv[1]
    
    # Validate optimization type
    if opt_type not in valid_opt:
        print(f"Invalid optimisation type: {opt_type}")
        print(f"Valid options are: {', '.join(valid_opt)}")
        sys.exit(1)
    
    # Get number of trials from command line or user input
    if len(sys.argv) < 3:
        n_opt_trials = input("Input the number of optimisiation trials :\n")
    else:
        n_opt_trials = int(sys.argv[2])

    # Validate number is an integer
    try:
        userdata = int(n_opt_trials)
    except ValueError:
        print(f"Invalid number of trial: {n_opt_trials}")
        print("Choose the number of trials between 1 and 50")
        sys.exit(1)

    n_opt_trials = int(n_opt_trials)

    # Validate number is in valid range
    if n_opt_trials < 1 or n_opt_trials > 50:
        print(f"Invalid number of trial: {opt_type}")
        print("Choose the number of trials between 1 and 50")
        sys.exit(1)

    # Run optimization
    quadruped_jump_optimization(opt_type, n_opt_trials)
