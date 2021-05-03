sp = serial.Serial("/dev/cu.usbserial-AB0L9U99",115200)
if sp.inWaiting() != 0
    initial_data = sp.readline()
    initial_split = split(initial_data, ",")
    initial_yaw = parse(Float64, data_split[3])
end

mechanism = parse_urdf(Float64,"panda.urdf")
state = MechanismState(mechanism)
mvis = MechanismVisualizer(mechanism, URDFVisuals("panda.urdf"),vis)

q_start = [0,-pi/4,0,-2,0,pi,pi/4,.05,.05]
update_robot_state(state,mvis,q_start)

ee_frame = default_frame(bodies(mechanism)[12])
body = findbody(mechanism, "end_effector")
control_point = Point3D(default_frame(body), 0., 0., 0.)
global desired_tip_location = Point3D(root_frame(mechanism), 0.4, 0.0, 0.65)
setelement!(mvis,control_point, 0.015, "p1")

sleep(3)

data = "0.00, 0.00, 0.00\n"
while true
    while sp.inWaiting() == 0
    end

    global data = data * sp.read(sp.inWaiting())
    if occursin("\n", data)
        lines = split(data, "\n")
        len = length(lines)
        global recent_data = lines[len-1]
        global data = lines[len]
    end

    print(recent_data)
    data_split = split(recent_data, ",")
    pitch = parse(Float64, data_split[1])
    roll = parse(Float64, data_split[2])
    yaw = parse(Float64, data_split[3])


    x = desired_tip_location.v[1]
    y = desired_tip_location.v[2]
    z = desired_tip_location.v[3]

    change = false

    if pitch > 45
        x += .005
        change = true
    end
    if pitch < -45
        x -= .005
        change = true
    end

    if roll > 45
        y += .005
        change = true
    end
    if roll < -45
        y -= .005
        change = true
    end

    if yaw > 45
        z += .005
        change = true
    end
    if yaw < -45
        z -= .005
        change = true
    end

    if change == true
        global desired_tip_location = Point3D(root_frame(mechanism), x,y,z)
        jacobian_transpose_ik!(state, body, control_point, desired_tip_location)
    end
end
