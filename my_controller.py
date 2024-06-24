from controller import Robot, Motor
import math

class MyRobot:
    MOTOR_NAMES = ["FL_MOTOR_1", "FR_MOTOR_1", "FL_MOTOR_2", "FR_MOTOR_2", 
    "BL_MOTOR_1", "BR_MOTOR_1", "BL_MOTOR_2", "BR_MOTOR_2"]

    gait_phase_shift = [  
        [0, 0.5, 0.25, 0.75],    
        [0, 0.5, 0.5, 0],  
        [0, 0, 0.5, 0.5]
    ]

    gait_setup = [
        ["FL_MOTOR_1", "FL_MOTOR_2"],
        ["FR_MOTOR_1", "FR_MOTOR_2"],
        ["BR_MOTOR_1", "BR_MOTOR_2"],
        ["BL_MOTOR_1", "BL_MOTOR_2"]
    ]

    def __init__(self, control_step_duration=32):
        self._controlStep = control_step_duration
        self._stepCount = 0

        self.robot = Robot()
        self.motors = []
        self.position_sensors = []

        for motor_name in MyRobot.MOTOR_NAMES:
            motor = self.robot.getDevice(motor_name)
            self.motors.append(motor)
            motor_position_sensor = motor.getPositionSensor()
            self.position_sensors.append(motor_position_sensor)
            motor_position_sensor.enable(self._controlStep)

    def setMotorPosition(self, motorId, value):
        self.motors[motorId].setPosition(value)
    
    def getMotorPosition(self, motorId):
        return self.position_sensors[motorId].getValue()

    def WalkingPositionComputation(self, motorsPosition, t, gait_freq, gait_type, legId, stride_length_factor, backward):
        freq = gait_freq
        n = int(t / (1 / freq))
        t = t - n * (1 / freq)
        if backward:
            t = (1 / freq) - t

        L1 = 0.2
        L2 = 0.2
        a = 0.95 * L1 * stride_length_factor
        h = 0
        k = -(L1 + L2 / 2)
        b = -k - math.sqrt(L1 * L1 + L2 * L2)

        x = h + a * math.cos(2 * math.pi * freq * t + MyRobot.gait_phase_shift[gait_type][legId] * 2 * math.pi)
        y = k + b * math.sin(2 * math.pi * freq * t + MyRobot.gait_phase_shift[gait_type][legId] * 2 * math.pi)

        A2 = math.acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2))
        A1 = math.acos(((L1 + L2 * math.cos(A2)) * x - (-L2 * math.sin(A2)) * y) /
                       ((L1 + L2 * math.cos(A2)) ** 2 + (-L2 * math.sin(A2)) ** 2))
        A1 = math.pi / 2 - A1

        motorsPosition[0] = A1
        motorsPosition[1] = A2

    def walk(self):
        gait_type = 1
        freq = 1.0
        stride_length_factor = [0.4, 0.4, 0.4, 0.4]
        backward = False

        while self.robot.step(self._controlStep) != -1:
            motorPositions = [0, 0]
            for legId in range(4):
                self.WalkingPositionComputation(motorPositions, self._stepCount * (self._controlStep / 1000.0), freq, gait_type, legId,
                                            stride_length_factor[legId], backward)
                self.setMotorPosition(MyRobot.MOTOR_NAMES.index(MyRobot.gait_setup[legId][0]), motorPositions[0])
                self.setMotorPosition(MyRobot.MOTOR_NAMES.index(MyRobot.gait_setup[legId][1]), motorPositions[1])
                
                if legId == 0:
                    print("FL_LEG :", self.getMotorPosition(0), self.getMotorPosition(2))
                elif legId == 1:
                    print("FR_LEG :", self.getMotorPosition(1), self.getMotorPosition(3))
                elif legId == 2:
                    print("BR_LEG :", self.getMotorPosition(5), self.getMotorPosition(7))
                else:
                    print("BL_LEG :", self.getMotorPosition(4), self.getMotorPosition(6))

            self._stepCount += 1

if __name__ == "__main__":
    my_robot = MyRobot()
    my_robot.walk()
