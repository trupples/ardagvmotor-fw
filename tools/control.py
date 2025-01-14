import math
import canopen
import time
from motor402 import Motor

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

node.nmt.state = "OPERATIONAL"

time.sleep(0.5)

motor.operating_mode = 'pv'

time.sleep(0.5)

motor.to_operational()

print("OPERATION ENABLED")
time.sleep(2)

for i in range(1000):
	try:
		node.sdo['Target velocity'].write(int(i * 4000))
	except:
		pass
	time.sleep(0.01)
# node.sdo['Target velocity'].write(987654321) # mAGIC STOP VALUE

for i in range(1000):
	v = math.sin(i * 0.1) * 4000000
	try:
		node.sdo['Target velocity'].write(int(v))
	except:
		pass
	time.sleep(0.01)

motor.to_switch_on_disabled()
