import sys
from math import *
import time
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time

IP_ADDRESS = '192.168.0.209'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}

# Starting Pos: 2.8, 1.6
# Close to Box: 2.4, 1.6
# To the right: 2.4, 2.2
# Forward: 0.9, 2.4
# Get Duck and Go: 0.9, 1.3
# Back to Other Side: 2.5, 1.1
# Starting Pos: 2.8, 1.6

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


if __name__ == "__main__":
    import socket
    ## getting the hostname by socket.gethostname() method
    hostname = socket.gethostname()
    ## getting the IP address using socket.gethostbyname() method
    ip_address = socket.gethostbyname(hostname)
    print(ip_address)
    
    clientAddress = "192.168.0.9"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 209

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    # x,y, rotation
    xy_des = [[-1.31, 1.51], [-1.35, 2.13], [-2.65, 2.15], [-2.69, 0.98], [-1.44, 0.85], [-0.70, 1.50]]

    # Point index
    p = 0

    # waypoints = [(5.33,3.58),(-4.27,3.357),(-4.27,-3), (5.458,-3.036)]
    ### alex
    waypoints = [(6.1,2.3),(4.13,2.357),(4.13,-.5), (6.1,-.5)]
    # waypoints = [(-1,-2),(-1,0),(1,0), (1,-2)]
    idx=4

    ### center of the circle
    # while True:
    #     if robot_id in positions:
    #         pos = positions[robot_id]
    #         cx = pos[0]
    #         cy = pos[1]
    #         break

    # Bound is max pwm input
    bound = 1500

    # Radius for waypoints
    radius = .4

    # P-Controller gain
    k_v  = 1700
    k_pr = 200

    t = 0.
    while is_running:
        try:
            if robot_id in positions:
                if p > 5:
                    break
                theta = rotations[robot_id] * pi/180

                # i = idx%4
                # ## Lap around the track
                # if abs(waypoints[i][0]-positions[robot_id][0])<radius and abs(waypoints[i][1]-positions[robot_id][1])<radius:
                #     print('HIT WAYPOINT')
                #     idx+=1

                x = xy_des[p][0] 
                y = xy_des[p][1]
                # x = waypoints[i][0]
                # y = waypoints[i][1]

                ## Circular Trajectory
                # x = cx+(1.5*cos(t))
                # y = cy+(1.5*sin(t))
                # print('(%f,%f)'%(x,y))

                # P control for x
                errx = x - positions[robot_id][0]
                # P control for y
                erry = y - positions[robot_id][1]
                print('(%f,%f)'%(positions[robot_id][0],positions[robot_id][1]))
                print('(%f,%f)'%(errx,erry))

                # P control for rotation
                alpha = atan2(erry, errx)
                errw  = degrees(atan2(sin(alpha-theta), cos(alpha-theta)))
                print(errw)
                omega = k_pr*errw

                v = k_v*(sqrt(errx**2 + erry**2))
                u = np.array([v-omega, v+omega])
                # set bound of motor input
                u[u > 1500] = 1500
                u[u < -1500] = -1500

                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                print(command)
                s.send(command.encode('utf-8'))

                if sqrt(errx**2 + erry**2) < 0.19:
                    p += 1

                time.sleep(.1)
                t += .02
                
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()
            streaming_client.shutdown()
    
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

    # Close the connection
    s.shutdown(2)
    s.close()
    streaming_client.shutdown()