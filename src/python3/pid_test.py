import unittest
import numpy
from pid import PIDController

class TestPIDController(unittest.TestCase):

    def test_P_zero(self):

        kp = 1
        ki = 0
        kd = 0
        state = 0
        minOutput = -10
        maxOutput = 10

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 0.0
        targetState = 0.0
        timeDelta = 1  # timeDelta is irrelevant for P
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertTrue(output == 0)

    def test_P_lower_limit(self):

        kp = 1
        ki = 0
        kd = 0
        state = 0
        minOutput = 0
        maxOutput = 10

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 0.1
        targetState = 0
        timeDelta = 1  # timeDelta is irrelevant for P
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertTrue(output == 0)


    def test_P_positive(self):

        kp = 0.01
        ki = 0
        kd = 0
        state = 0
        minOutput = 0.0
        maxOutput = 5.0

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 4900
        targetState = 5000
        timeDelta = 0.134  # timeDelta is irrelevant for P
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, 1.0)


    def test_P_negative(self):

        kp = 1
        ki = 0
        kd = 0
        state = 0
        minOutput = -10
        maxOutput = 10

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 0.1
        targetState = 0
        timeDelta = 10009  # timeDelta is irrelevant for P
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -0.1)


    def test_D_zero(self):

        kp = 0
        ki = 0
        kd = 1
        state = 1
        minOutput = 0.0
        maxOutput = 500.0

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 1
        targetState = 13    # a change in targetState should not influence derivative
        timeDelta = 1
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output,0.0)

    def test_D_one(self):

        kp = 0
        ki = 0
        kd = 1
        state = 1
        minOutput = -500.0
        maxOutput = 500.0

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 2
        targetState = 1
        timeDelta = 1
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -1)

    def test_I_count(self):

        kp = 0
        ki = 1
        kd = 0
        state = 1
        minOutput = -500.0
        maxOutput = 500.0

        pid = PIDController(kp, ki, kd, state, minOutput, maxOutput)

        currentState = 2
        targetState = 1
        timeDelta = 0.5
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -0.5)

        currentState = 3
        targetState = 2.5
        timeDelta = 0.5
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -0.75)

        currentState = 300
        targetState = 250
        timeDelta = 2
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -100.75)

        currentState = 400
        targetState = 0
        timeDelta = 2
        output = pid.compute(currentState, targetState, timeDelta)

        self.assertEqual(output, -500)

if __name__ == '__main__':
    unittest.main()
