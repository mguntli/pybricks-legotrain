from pybricks.hubs import CityHub
from pybricks.pupdevices import DCMotor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# Adjusted red color to reduce false positives
Color.RED = Color(h=350, s=90, v=80)
MARKER_STATION = Color.RED
# Sensor reflection values to detect grey studs of train tracks on white surface
LIGHT = 57
DARK = 16
# Add a bit of hysteresis to make sure we skip extra changes on the edge of each track.
HYSTERESIS = (LIGHT - DARK) / 4
THRESHOLD_UP = (LIGHT + DARK) / 2 + HYSTERESIS
THRESHOLD_DOWN = (LIGHT + DARK) / 2 - HYSTERESIS
# It's two studs (16 mm) for each position increase.
MM_PER_COUNT = 16
# Maximum speed to drive in case of wrong readings
MAX_DUTY_CYCLE = 75
# Desired drive speed in mm per second.
SPEED = 400
MAX_STATION_STOP_TIME_MS = 5*60000
MIN_STATION_STOP_TIME_MS = 5000
BLE_REMOTE_START_CHANNEL = 1
BLE_TRAIN_CHANNEL = 2
BLE_PROTOCOL_TRAIN_DRIVING = 100
BLE_PROTOCOL_TRAIN_STOPPED = 101

def initConstantSpeedControl():
    global onRailway
    global position
    global driveWatch
    
    onRailway = True
    position = 0
    driveWatch.reset()

def driveConstantSpeed():
    global onRailway
    global position
    global driveWatch

    # Measure the reflection.
    reflection = sensor.reflection()

    # If the reflection exceeds the threshold, increment position.
    if (reflection > THRESHOLD_UP and onRailway) or (reflection < THRESHOLD_DOWN and not onRailway):
        onRailway = not onRailway
        position += 1
        
    # Compute the target position based on the time.
    targetPos = driveWatch.time() / 1000 * SPEED / MM_PER_COUNT

    # The duty cycle is the position error times a constant gain.
    duty = 2*(targetPos - position)
    if (duty > MAX_DUTY_CYCLE):
        duty = MAX_DUTY_CYCLE
    motor.dc(duty)

def driveUntilColor(color):
    hub.light.on(Color.WHITE)
    while sensor.color() != color:
        driveConstantSpeed()

def driveWhileColor(color):
    while sensor.color() == color:
        driveConstantSpeed()

def driveUntilColorPattern(color1, color2):
    hub.light.on(Color.WHITE)
    driveWatch.reset()
    colorPatternMatch = False
    state = 1
    while not colorPatternMatch:
        driveConstantSpeed()
        colorDetected = sensor.color()
        if state == 1 and (colorDetected == color1):
            state = 2
        elif state == 2 and (colorDetected != color1):
            state = 3
        elif state == 3 and (colorDetected == color2):
            colorPatternMatch = True
        else:
            state = 1

def stopTrain():
    motor.stop()
    hub.light.off()

# returns number of laps received through BLE remote control
def waitForNumberOfLaps(maxWaitTime) -> int:
    stopWatch = StopWatch()
    while(stopWatch.time() < maxWaitTime):
        data = hub.ble.observe(BLE_REMOTE_START_CHANNEL)
        if data is None:
            # Broadcasts are only sent every 100 milliseconds
            wait(100)
        elif data >= 1:
            hub.ble.broadcast(BLE_PROTOCOL_TRAIN_DRIVING)
            return data
    return 1;

# Initalization
hub = CityHub(broadcast_channel=BLE_TRAIN_CHANNEL, observe_channels=[BLE_REMOTE_START_CHANNEL])
motor = DCMotor(Port.A)
sensor = ColorDistanceSensor(Port.B)
sensor.detectable_colors([Color.RED, Color.WHITE, Color.GRAY, Color.YELLOW, Color.NONE])
driveWatch = StopWatch()
onRailway = True
position = 0

while True:
    nofLaps = waitForNumberOfLaps(MAX_STATION_STOP_TIME_MS)
    initConstantSpeedControl()
    while nofLaps >= 1:
        driveUntilColor(MARKER_STATION)
        nofLaps -= 1
        if (nofLaps >= 1):
            # drive further until station marker is gone in case it's very long or we drive slowly
            driveWhileColor(MARKER_STATION)
    stopTrain()
    hub.ble.broadcast(BLE_PROTOCOL_TRAIN_STOPPED)
    wait(MIN_STATION_STOP_TIME_MS)