function update_grabber_state(state,mvis,qv)
    set_configuration!(state, qv)
    set_configuration!(mvis, configuration(state))
end

# primary function
function from_to(start_pos, end_pos)
  q1=q2=q3=q4=q5=q6=q7=q8=q9=q10=q11=0.0
  qv = [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11]

  new_diff = get_diff(state,start_pos,qv,false)
  while new_diff > 0.05
    for i = 1:11
      while true
        diff = new_diff
        qv[i] = qv[i] + 0.01
        new_diff = get_diff(state,start_pos,qv,false)
        if new_diff < 0.05
          break
        end
        new_diff < diff || break
        end
      if new_diff < 0.05
        break
      end
      while true
        diff = new_diff
        qv[i] = qv[i] - 0.01
        new_diff = get_diff(state,start_pos,qv,false)
        if new_diff < 0.05
          break
        end
        new_diff < diff || break
      end
      if new_diff < 0.05
        break
      end
    end
  end

  new_diff = get_diff(state,end_pos,qv,true)
  while new_diff > 0.05
    for i = 1:11
      while true
        diff = new_diff
        qv[i] = qv[i] + 0.01
        new_diff = get_diff(state,end_pos,qv,true)
        if new_diff < 0.05
          break
        end
        new_diff < diff || break
        end
      if new_diff < 0.05
        println("Final position reached.")
        break
      end
      while true
        diff = new_diff
        qv[i] = qv[i] - 0.01
        new_diff = get_diff(state,end_pos,qv,true)
        if new_diff < 0.05
          break
        end
        new_diff < diff || break
      end
      if new_diff < 0.05
        println("Ending position reached.")
        break
      end
    end
  end
end

function get_diff(state,desired_position,qv,visual_on)
  # update the state of the robot
  if visual_on
    update_grabber_state(state,mvis,qv)
  else
    set_configuration!(state, qv)
  end

  # create variables for base and end effector frames
  ee_frame=default_frame(bodies(mechanism)[end])
  base_frame = default_frame(bodies(mechanism)[1])

  # create a point at the tip of the end effector
  x = Point3D(ee_frame, 0.0, -1.25, 0.0)

  # calculate end effector point position in the base frame
  ee_in_base_frame = relative_transform(state, base_frame, ee_frame)
  x_in_w = inv(ee_in_base_frame) * x

  print("Distance from final position: ")
  print(distance(x_in_w.v, desired_position))
  println(" units")
  return distance(x_in_w.v, desired_position)
end

function distance(v1, v2)
  return sqrt((v1[1]-v2[1])^2 + (v1[2]-v2[2])^2 + (v1[3]-v2[3])^2)
end

# Load mechanism and visualize at some config
delete!(vis)
mvis, mechanism = display_grabber("grabber.urdf",vis)
state = MechanismState(mechanism)
update_grabber_state(state,mvis,[0,0,0,0,0,0,0,0,0,0,0])

ee_frame=default_frame(bodies(mechanism)[end])

# Define point in frame of end effector
x = Point3D(ee_frame, 0.0, -1.25, 0.0)

# Visualize this point
setelement!(mvis,x, 0.05, "p1") # mvis, point, radius, name

start_position_input = [0.0,0.0,0.0]
end_position_input = [0.0,0.0,0.0]

println("""Please enter 3 numbers line-by-line to represent the start position vector""")

for number in 1:3
  num = readline()
  start_position_input[number] = parse(Float64, num)
end
println("Start position set to: ")
println(start_position_input)

println("""Please enter 3 numbers line-by-line to represent the end position vector""")

for number in 1:3
  num = readline()
  end_position_input[number] = parse(Float64, num)
end
println("End position set to: ")
println(end_position_input)

from_to(start_position_input,end_position_input)
