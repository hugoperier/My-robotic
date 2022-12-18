from modules.motors.OsoyooMotor import OsoyooMotor

class OsoyooBase:
    def __init__(self, configuration):
        self.i = 0
        self.configuration = configuration

        self.left_wheel = OsoyooMotor(configuration["wheels"]["left"])
        self.right_wheel = OsoyooMotor(configuration["wheels"]["right"])
        self.max_speed = configuration["max_speed"]
        
    @property
    def is_moving(self):
        """Check if the robot is moving"""
        for wheel in self.wheels:
            if wheel.is_moving:
                return True
        return False

    def set_velocity(self, x, yaw):
        if ((not (0 < x < 100)) or (not (0 < yaw < 100))):
            raise ValueError("Velocity must be define between 0 and 100")

        # Calculate the velocity of each wheel
        wheel_velocity_left = x
        wheel_velocity_right = x

        # Adjust the velocity of each wheel based on the yaw value
        if yaw > 0:
            wheel_velocity_left = max(0, wheel_velocity_left - yaw)
            wheel_velocity_right = min(100, wheel_velocity_right + yaw)
        elif yaw < 0:
            wheel_velocity_left = min(100, wheel_velocity_left - yaw)
            wheel_velocity_right = max(0, wheel_velocity_right + yaw)
        
    def set_velocity(x, yaw):
        if ((not (-100 <= x <= 100)) or (not (-100 <= yaw <= 100))):
            raise ValueError("Velocity must be define between -100 and 100")

            if x > 0:
                if yaw > 0:
                    return [x, max(0, x-yaw)]
                else:
                    return [max(0,x+yaw), x]
            elif x < 0:
                if yaw > 0:
                    return [min(0,x+yaw), x]
                else:
                    return [x, min(0,x-yaw)]
            else:
                    return [0, yaw]

    def stop(self):
        for wheel in self.wheels:
            wheel.stop()