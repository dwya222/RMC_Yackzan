import Pkg;
Pkg.activate(@__DIR__);
using LinearAlgebra, StaticArrays
using RigidBodyDynamics, RigidBodySim
using MeshCat, MeshCatMechanisms
vis = Visualizer();open(vis)
using Gadfly, Cairo, Fontconfig
using CSV
using DataFrames
using PyCall

serial = pyimport("serial")

function update_robot_state(state,mvis,q)
    set_configuration!(state, q)
    set_configuration!(mvis, configuration(state))
end

function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    mechanism = parse_urdf(Float64,urdfPath)
    state = MechanismState(mechanism)
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    q_start = [0,-pi/4,0,-2,0,pi,pi/4,.05,.05]
    update_robot_state(state,mvis,q_start)
    # zero_configuration!(state);
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    for bd in bodies(mechanism)
        setelement!(mvis,default_frame(bd),0.5,"$bd")
    end
    return mvis, mechanism
end

function jacobian_transpose_ik!(state::MechanismState,
                               body::RigidBody,
                               point::Point3D,
                               desired::Point3D;
                               α=0.5,
                               iterations=1)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, RigidBodyDynamics.transform(state, point, world))

    q = copy(configuration(state))
    for i in 1:iterations
        # Update the position of the point
        point_in_world = RigidBodyDynamics.transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (RigidBodyDynamics.transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
        update_robot_state(state, mvis, q)
    end
    state
end

display_urdf("panda.urdf",vis)
