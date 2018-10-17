import argparse
import time
import enum as en

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(en.Enum):
        MANUAL = 0
        ARMING = 1
        TAKEOFF = 2
        WAYPOINT = 3
        LANDING = 4
        DISARMING = 5

class BackyardFlyer(Drone):

        def __init__(self, connection):
            super().__init__(connection)
            self.target_position = np.array([0.0, 0.0, 0.0])
            self.all_waypoints = []
            self.in_mission = True
            self.check_state = {}

            #initial state
            self.flight_state = States.MANUAL

            #TODO: Register all your callbacks here
            self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
            self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
            self.register_callback(MsgID.STATE, self.state_callback)

        def local_position_callback(self):
            if(self.flight_state == States.TAKEOFF):
                #check if the drone is near of the altitude limit
                if(-1.0 * self.local_position[2] > 0.95 * self.target_position[2]):
                    #if yes, start to run through the waypoint
                    self.all_waypoints = self.calculate_box()
                    self.waypoint_transition()
            elif(self.flight_state == States.WAYPOINT):
                #check if the drone position is near of the waypoint (using the frobenius norm)
                dist = np.power((self.local_position[0] - self.target_position[0]),2) + np.power((self.local_position[1] - self.target_position[1]),2)
                dist = np.sqrt(dist);
                if(dist < 1.0):
                    #check if there is at least one element in waypoint
                    if(len(self.all_waypoints) > 0):
                        self.waypoint_transition()
                        if(len(self.all_waypoints)>1 and len(self.all_waypoints) < 3):
                            print('Hello Trees in front of me')
                    else:
                        #check if the drone is stoping
                        if(np.linalg.norm(self.local_velocity[0:2]) < 1.0):
                            #if yes, and how there is no waypoints, start to landing
                            self.landing_transition()

        def velocity_callback(self):
            if(self.flight_state == States.LANDING):
                if(self.global_position[2] - self.global_home[2] < 0.1):
                    if(abs(self.local_position[2]) < 0.01):
                        self.disarming_transition()

        def state_callback(self):
            if self.in_mission:
                if self.flight_state == States.MANUAL:
                    self.arming_transition()
                elif self.flight_state == States.ARMING:
                    if self.armed:
                        self.takeoff_transition()
                elif self.flight_state == States.DISARMING:
                    if ~self.armed & ~self.guided:
                        self.manual_transition()

        def calculate_box(self):
            print("Setting Home")
            local_waypoints = [[30.0, 0.0, 4.0], [30.0, 30.0, 4.0], [0.0, 30.0, 4.0], [0.0, 0.0, 4.0]]
            return local_waypoints

        # this method is the same that up and down
        def arming_transition(self):
            print("arming transition")
            self.take_control()
            self.arm()
            self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
            self.flight_state = States.ARMING

        #this method is the same that up and down
        def takeoff_transition(self):
            print("takeoff transition")
            target_altitude = 4.0
            self.target_position[2] = target_altitude
            self.takeoff(target_altitude)
            self.flight_state = States.TAKEOFF

        def waypoint_transition(self):
            print("waypoint transition")
            #get the first waypoint position
            self.target_position = self.all_waypoints.pop(0)
            print('target position', self.target_position)
            print('My velocity now is (m\s):', self.local_velocity)
            print('My altitude now is (m):', (-1 * self.local_position[2]))
            #command the drone to move to a specific position, very important function
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
            #and keep in waypoint state
            self.flight_state = States.WAYPOINT

        # this method is the same that up and down
        def landing_transition(self):
            print("landing transition")
            self.land()
            self.flight_state = States.LANDING

        # this method is the same that up and down
        def disarming_transition(self):
            print("disarm transition")
            self.disarm()
            self.release_control()
            self.flight_state = States.DISARMING

        # this method is the same that up and down
        def manual_transition(self):
            print("manual transition")
            self.stop()
            self.in_mission = False
            self.flight_state = States.MANUAL

        # this method is the same that up and down
        def start(self):
            self.start_log("Logs", "NavLog.txt")
            print("starting connection")
            super().start()
            self.stop_log()

if(__name__ == "__main__"):
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()