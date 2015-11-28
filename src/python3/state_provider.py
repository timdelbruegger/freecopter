from accelerometer import AccelerometerReading, GyroReading
from state_change_planning import State

# contains settings and calibration of accelerometer/gyroscope/magnetometer
RT_IMU_LIB_SETTINGS_FILE = "RTIMULib"

# Accumulates and fuses data from all sensors for a complete sense of the current state. Mostly delegates to RTIMULib for a first implementation
class StateProvider:
    
    def __init__(self):
        self.listeners = []
    
        print("Using settings file " + RT_IMU_LIB_SETTINGS_FILE + ".ini")
        if not os.path.exists(RT_IMU_LIB_SETTINGS_FILE + ".ini"):
          print("Settings file does not exist, will be created")
        
        self.settings = RTIMU.Settings(SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(s)
        self.pressure = RTIMU.RTPressure(s)
        
        print("IMU Name: " + imu.IMUName())
        print("Pressure Name: " + pressure.pressureName())
            
        if (not imu.IMUInit()):
            print("IMU Init Failed. Exiting...")
            sys.exit(1)
        else:
            print("IMU Init Succeeded");

        # this is a good time to set any fusion parameters

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        if (not self.pressure.pressureInit()):
            print("Pressure sensor Init Failed")
        else:
            print("Pressure sensor Init Succeeded")

        # we should try to ask for new data every x seconds
        # IMUGetPollInterval returns milliseconds, we translate to seconds
        self.pollInterval = self.imu.IMUGetPollInterval()*1.0/1000.0
        print("Recommended Poll Interval: %dmS\n" % self.pollInterval)
        
        firstReading = True
    
    def registerListener(self, listener):
        self.listeners.append(listener)
        
    def update(self, timeDelta):    
        self.timeWaited += timeDelta
        if self.timeWaited >= self.pollInterval:
            if self.imu.IMURead():
            
                # here we compile the data
                newState = QuadrocopterState()
                
                # TODO: Maybe inject height info from ultrasonic sensor?
                            
                # collect fused data
                data = self.imu.getIMUData()
                print(data)
                
                newState = self.updateHeightInfoWithPressure(data, newState)
                # TODO: check if I really need additional pressure here... isn't that also included in getIMUData???
                (pressureValid, pressure, temperatureValid, temperature) = self.pressure.pressureRead()

                # pressure should always be valid, but the code should not crash if we get a False here.
                if pressureValid:
                    data.heightAboveSea = computeHeight(data["pressure"])
            
                    # special operation for first reading
                    if firstReading:
                        self.startHeightAboveSea = data.heightAboveSea
                        firstReading = False
                    
                    # calculate height above starting point
                    newState.elevation = State(data.heightAboveSea-self.startHeightAboveSea, data.linearSpeed.y)
            
                (newState.rotation.x.value, newState.rotation.y.value, newState.rotation.z.value) = data["fusionPose"]
                (newState.rotation.x.speed, newState.rotation.y.speed, newState.rotation.z.speed) = data["gyro"]
            
                # give data to listeners
                for(listener: self.listeners):
                    listener.newState(self.timeWaited, newState)
                    
                self.timeWaited = 0
    
class QuadrocopterState:
    elevation = State()
    rotation = State3d()
    
class State3dChangeSelectTimeToReachTarget:
    def __init__(self, currentStates, targetStates):
        self.currentStates = currentStates
        self.targetStates = targetStates

    def inSeconds(self, timeToReachTargetState):

        rotPlans = map(
            lambda (state, target):
                state.planChangeTo(target).inSeconds(timeToReachTargetState),
            zip(self.currentStates, self.targetStates)
        )

        return rotPlans
    
class State3d:
    x = State(0,0)
    y = State(0,0)
    z = State(0,0)
    
    def planChangeTo(self, target):
        rotStates  = [self.x, self.y, self.z]
        rotTargets = [target.x, target.y, target.z]
        return State3dChangeSelectTimeToReachTarget(rotStates, rotTargets)
        
        
#  computeHeight() - the conversion uses the formula:
#
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperatur elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = mloecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#
#  Given the constants, this works out to:
#
#  h = 44330.8 * (1 - (p / P0)**0.190263)

def computeHeight(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));