import math
import canopen
import time
from motor402 import Motor
import signal, sys

net = canopen.Network().connect(channel='vcan0', interface='socketcan')

#node = net.add_node(0x10, '../objdict/AGV_CIA402_ProfileVelocity_Barebones.eds')
node = net.add_node(0x10, '../objdict/node16.dcf')

#node.nmt.send_command(129) # Reset node

#time.sleep(3)

#node.tpdo.read()
#node.rpdo.read()

motor = Motor(node, rename_map={
    "controlword": "Controlword",
    "statusword": "Statusword",
    "operating_mode": "Modes of operation",
    "target_position": "Target position",
    "profile_velocity": "Profile velocity",
    "target_velocity": "Target velocity",
    "position_actual_value": "Position actual value",
    "velocity_actual_value": "Velocity actual value",
})

def sigint(sig, frame):
    print("to SWITCH_ON_DISABLED")
    motor.to_switch_on_disabled()
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, sigint)

node.nmt.state = "OPERATIONAL"

time.sleep(0.5)

motor.operating_mode = 'pv'

time.sleep(0.5)
node.sdo['Target velocity'].write(0)
motor.to_operational()

print("OPERATION ENABLED")
time.sleep(2)

for i in range(100):
	try:
		node.sdo['Target velocity'].write(int(i * 40000))
	except:
		pass
	time.sleep(0.1)

for v in [0, 4000000, -4000000, 4000000, -4000000, 0]:
    print(v)
    node.sdo['Target velocity'].write(v)
    time.sleep(1)

for i in range(1000):
	v = math.sin(i * 0.1) * 4000000
	try:
		node.sdo['Target velocity'].write(int(v))
	except:
		pass
	time.sleep(0.1)

motor.to_switch_on_disabled()

