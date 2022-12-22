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
        if ((not (-100 <= x <= 100)) or (not (-100 <= yaw <= 100))):
            raise ValueError("Velocity must be define between -100 and 100")

        velocities = self.get_velocity_absolute(x, yaw)
        print(f"moving using velocity [{velocities[0]};{velocities[1]}]")

        left_wheel_speed = int(abs(velocities[0]) * self.left_wheel.max_speed / 100)
        self.left_wheel.changespeed(left_wheel_speed)
        if velocities[0] > 0:
            self.left_wheel.forward()
        elif velocities[0] < 0:
            self.left_wheel.backward()

        right_weel_speed = int(abs(velocities[1]) * self.right_wheel.max_speed / 100)
        self.right_wheel.changespeed(right_weel_speed)
        if velocities[1] > 0:
            self.right_wheel.forward()
        elif velocities[1] < 0:
            self.right_wheel.backward()


    def get_velocity_absolute(self, x, yaw):
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
        self.left_wheel.stop()
        self.right_wheel.stop()