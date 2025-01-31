import canopen
from canopen.profiles.p402 import BaseNode402
import time
import math

node = BaseNode402(0x16, 'ardagvmotor.eds')
network = canopen.Network()
network.connect(channel='can2', interface='socketcan', bitrate=500000)
network.add_node(node)

network.sync.start(0.1)


print("Resetting comms")
node.nmt.state = 'RESET'

node.nmt.wait_for_bootup(5)

node.load_configuration()

node.setup_402_state_machine()

node.nmt.state = 'OPERATIONAL'

print("Enabling motor")

# Walk through the state machine
node.state = 'OPERATION ENABLED'

for i in range(10000):
    v = math.sin(time.time()) * 4e6
    node.rpdo[4]['Controlword'].raw = 0b1111
    node.rpdo[4]['Target velocity'].raw = int(v)
    node.rpdo[4].transmit()

    if i % 100 == 0:
        V = node.sdo['TMC9660 SUPPLY_VOLTAGE'].raw
        print(f"Power supply = {V/10:.1f}")

    time.sleep(0.01)


