import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics
using LinearAlgebra
using MeshCat, MeshCatMechanisms
using GeometryTypes, CoordinateTransformations, ColorTypes
vis = Visualizer();open(vis)

function display_grabber(urdfPath,vis)
    mechanism = parse_urdf(Float64,urdfPath)

    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    for bd in bodies(mechanism)
       setelement!(mvis,default_frame(bd),1,"$bd")
    end
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mvis, mechanism
end

mvis, mechanism = display_grabber("grabber.urdf",vis)
state=MechanismState(mechanism)
set_configuration!(state, [0,0,0,0,0,0,0,0,0,0,0])
set_configuration!(mvis, configuration(state))
include("main.jl")
