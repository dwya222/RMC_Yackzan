# Refresh visualization
delete!(vis)
# Load mechanism info
urdfPath = "panda.urdf"
mvis, mechanism = display_urdf(urdfPath,vis)
# define initial and final joint angle arrays
q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]
# Create state and set initial config and velocity
state = MechanismState(mechanism)
set_configuration!(state,q0)
zero_velocity!(state)
# Update mechanism visual
set_configuration!(mvis, configuration(state))

#simple PD control
function simple_PD!(τ, t, state)
    # set desired joint angle vector and gains
    kd = diagm([20,80,20,80,20,20,20,20,20])
    kp = diagm([50, 200, 50, 700, 50, 250, 50, 50, 50])
    # Compute a value for τ
    τ .= -kd * velocity(state) - kp * (configuration(state) - qd)
    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

# Define ODE Problem, which defines closed loop using a simple PD control to
# get base_trajectories
problem = ODEProblem(Dynamics(mechanism,simple_PD!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme
global base_sol = solve(problem, Tsit5());

# PART 1 Trajectory function
function traj(t::Float64)
    # Retrieve q, qdot, and qddot at time t from base solution array
    q_of_t = base_sol(t,Val{0})[1:9]
    q_dot_of_t = base_sol(t,Val{1})[1:9]
    q_ddot_of_t = base_sol(t,Val{2})[1:9]

    # return q vectors
    q_of_t, q_dot_of_t, q_ddot_of_t
end

function control_PD!(τ, t, state)
    # set desired joint angle vector and gains
    kd = diagm([20,40,20,40,20,20,20,20,20])
    kp = diagm([500, 7000, 500, 2000, 500, 500, 500, 500, 500])
    # get joint angles, velocities and accelerations at t from traj function
    q, q_dot, q_ddot = traj(t)
    # Compute a value for τ
    τ .= kp * (q - configuration(state)) + kd * (q_dot - velocity(state))
    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

# PART 2: Define ODE Problem, which defines closed loop using control_PD!
problem2 = ODEProblem(Dynamics(mechanism,control_PD!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme
sol2 = solve(problem2, Tsit5());
norm2 = norm(qd[1:7]-sol2(10)[1:7])
println("PD Controller norm of: $norm2")
#setanimation!(mvis, sol2; realtime_rate = 1.0);

function control_CTC!(τ, t, state)
    kd = diagm([20,80,20,80,20,20,20,20,20])
    kp = diagm([50, 200, 50, 700, 50, 250, 50, 50, 50])
    # get joint angles, velocities and accelerations at t from traj function
    q, q_dot, q_ddot = traj(t)
    # calculate M, C, G and aq double dot terms
    aq_ddot = q_ddot + kd*(q_dot - velocity(state)) + kp*(q - configuration(state))
    M = mass_matrix(state)
    dummy_state = deepcopy(state)
    zero_velocity!(dummy_state)
    G = inverse_dynamics(dummy_state, velocity(dummy_state))
    C = inverse_dynamics(state, velocity(state)) - G
    # Compute a value for τ
    τ .= M*aq_ddot + C + G
    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

# PART 3: Define ODE Problem, which defines closed loop using control_CTC!
problem3 = ODEProblem(Dynamics(mechanism,control_CTC!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme
sol3 = solve(problem3, Tsit5());
# setanimation!(mvis, sol3; realtime_rate = 1.0);
