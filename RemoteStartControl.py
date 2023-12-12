from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor, Light, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

BLE_REMOTE_START_CHANNEL = 1
BLE_TRAIN_CHANNEL = 2
BLE_PROTOCOL_TRAIN_DRIVING = 100
BLE_PROTOCOL_TRAIN_STOPPED = 101

hub = TechnicHub(broadcast_channel=BLE_REMOTE_START_CHANNEL, observe_channels=[BLE_TRAIN_CHANNEL])
sensor = ColorDistanceSensor(Port.A)
lightGreen = Light(Port.C)
lightRed = Light(Port.D)

def observeBLE():
    data = hub.ble.observe(BLE_TRAIN_CHANNEL)
    if data is not None:
        if data == BLE_PROTOCOL_TRAIN_DRIVING:
            lightGreen.on()
            lightRed.off()
        elif data == BLE_PROTOCOL_TRAIN_STOPPED:
            lightGreen.off()
            lightRed.on()

def waitAndObserve(waitTime):
    while waitTime > 0:
        observeBLE()
        wait(100)
        waitTime -= 100

# returns number of laps based on color shown to the sensor
def waitForNumberOfLaps() -> int:
    while True:
        detectedColor = sensor.color()
        if detectedColor == Color.GREEN:
            return 1
        elif detectedColor == Color.BLUE:
            return 2
        elif detectedColor == Color.YELLOW:
            return 3
        else:
            observeBLE()
    
while True:
    hub.light.on(Color.WHITE)
    nofLaps = waitForNumberOfLaps()
    print(nofLaps)
    hub.ble.broadcast(nofLaps)
    hub.light.on(Color.BLUE)
    waitAndObserve(5000)
    hub.ble.broadcast(0)