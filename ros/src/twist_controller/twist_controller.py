import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, accel_limit, decel_limit, vehicle_mass, wheel_radius, refresh_rate):
        # TODO: Implement
    	self.Kp = vehicle_mass * wheel_radius
        self.rate = refresh_rate
    	self.yawController = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
    	self.pid = PID(2.5,0.3,0.,decel_limit, accel_limit)
    	self.lowPass = LowPassFilter(0.75,1.0)
        pass

    def control(self, v_target,w_target,v_current,w_current,dbw_enabled):
        if not dbw_enabled:
            self.reset()
            pass
        else:
            error = v_target-v_current
            throttle = self.pid.step(error,1.0/self.rate)
            if throttle < 0:
                brake = throttle * self.Kp
                throttle = 0
            else:
                brake = 0
            steer = self.yawController.get_steering(v_target, w_target, v_current)
            #steer = self.lowPass.filt(steer)
            #rospy.logwarn('steer: %s v_target: %s v_current: %s throttle: %s brake: %s',steer,v_target,v_current,throttle,brake)        # TODO: Change the arg, kwarg list to suit your needs
            # Return throttle, brake, steer
            return throttle, brake, steer

    def reset(self):
    	self.pid.reset()
    	pass
