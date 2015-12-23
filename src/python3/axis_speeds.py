import numpy

# This constant matrix is used in toMotorSignals().
# Columns: x/roll, y/yaw, z/pitch, vertical/up
# Rows: front left, front right, rear left, rear right
# For a derivation of this matrix, see
# https://timdelbruegger.wordpress.com/2015/10/04/quadrocopter-control-loop-recombining-axes-accelerations-to-motor-accelerations/
TRANSFORMATION_MATRIX = 0.25*numpy.transpose(numpy.matrix([
    [ 1, 1, 1, 1],
    [-1,-1, 1, 1],
    [ 1,-1,-1, 1],
    [-1, 1,-1, 1]]))



# Utility class that stores target rotation speeds for the 3 axes along with a
# target vertical speed. With the method toMotorSignals, these target speeds can
# be transformed to speeds for the 4 motors.
# The target speeds that we get here are already the result of axis rotation
# planning and PID calculations, not really meters per second or any other
# physical unit.
class AxisSpeeds():

    def __init__(self):
        self.axis_rotation_speeds = numpy.zeros(3)
        self.vertical_speed = 0 # up

    # Returns the speeds as an array. Does no transformation!
    def asArray(self):
        return numpy.append(self.axis_rotation_speeds, self.vertical_speed)

    # Transforms the axis specific speeds into motor specific speeds
    def toMotorSignals(self):
        return numpy.dot(self.asArray(),TRANSFORMATION_MATRIX).A1
