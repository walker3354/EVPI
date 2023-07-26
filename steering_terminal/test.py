#!/usr/bin/python3
# sudo ifconfig can0 txqueuelen 10000
# sudo ip link set can0 up type can bitrate 500000
import tomli
import time
import canopen
import socketio
import typing

class CsCanOpen:
    def __init__(self, can_interface, config_file):
        self.steering_id = 0
        self.config = self.load_config(config_file)
        self.can_network = self.canopen_init(can_interface)
        self.load_light_nodes(self.config["steering"])
        self.prox_dict = {'right': 0, 'left': 0}

    def load_config(self, config_file):
        with open(config_file, "rb") as f:
            toml_dict = tomli.load(f)
            return toml_dict

    def canopen_init(self, can_interface="can0"):
        network = canopen.Network()
        network.connect(channel=can_interface, bustype='socketcan')
        return network
    '''
    def load_light_nodes(self, config):
        print('loading light nodes')
        for node in config["nodes"]:
            print("Wait for light_node {0:} ready...".format(node["node_id"]))
            can_node = self.can_network.add_node(
                node["node_id"], config["config_file"])
            can_node.rpdo.read()
            can_node.nmt.wait_for_heartbeat()
            self.light_node_list.append(can_node)
        print("\nCheck all light nodes completed!\tinitila all light nodes\n")
        for light_node in self.light_node_list:
            light_node.nmt.state = 'PRE-OPERATIONAL'
            light_node.rpdo[1][0x6001].phys = 0
            light_node.rpdo[1].start(0.5)
        print("initial complete!")
    '''

    def load_steering_nodes(self, config):
        print('\nloading steering nodes\n')
        for node in config["nodes"]:
            print("Wait for prox_node {0:} ready...".format(node["node_id"]))
            can_node = self.can_network.add_node(
                node["node_id"], config["config_file"])
            can_node.tpdo.read()
            can_node.nmt.wait_for_heartbeat()
            self.porx_node_list.append(can_node)
        print("Check all steering nodes completed!")
        for node in self.porx_node_list:
            node.tpdo[1].add_callback(self.proximity_callback)

    def proximity_callback(self, msg):
        node_id = msg.cob_id - 384
        for var in msg:
            print(node_id, " : ", (var.raw / 1000))

    def disconnect(self):
        self.can_network.disconnect()


def main():
    cs_canopen = CsCanOpen("can0", "./evpi_sensor.toml")
    cs_canopen.swap_event_watcher()


if __name__ == "__main__":
    main()