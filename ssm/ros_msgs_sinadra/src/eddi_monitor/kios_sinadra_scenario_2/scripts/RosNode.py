#!/usr/bin/env python3
import rospy
import time
from EDDIInput import EDDIInput
import bn_processing_functions
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from eddi_messages.msg import BayesianNetworkOutput
from eddi_messages.msg import BayesianNetworkOutput, NodeOutput, OutcomeOutput
from bn_monitor import BayesianNetworkMonitor as EDDIMonitor

class RosNode():

    def __init__(self, monitor):
        rospy.init_node('kios_sinadra_scenario_2')
        self.monitor = monitor
        self.simulator_values = {}
        self.simulator_values_timestamps = {}
        self.init_subscriber()
        self.init_publisher()
        self.eddi_input = EDDIInput()
        self.rate = rospy.Rate(0.2)
        self.simulator_values_thresholds = {  }

    def out_0_callback (self, data):
        self.simulator_values['out_0'] = data

        self.simulator_values_timestamps['out_0'] = time.time()

    def out_1_callback (self, data):
        self.simulator_values['out_1'] = data

        self.simulator_values_timestamps['out_1'] = time.time()

    def out_2_callback (self, data):
        self.simulator_values['out_2'] = data

        self.simulator_values_timestamps['out_2'] = time.time()

    def out_3_callback (self, data):
        self.simulator_values['out_3'] = data

        self.simulator_values_timestamps['out_3'] = time.time()


    def ros_main(self):
        while not rospy.is_shutdown():
            self.check_validity()
            self.eddi_input.processing(self.simulator_values)
            output = self.monitor.execute_step(self.eddi_input)
            self.eddi_out_0_pub.publish(bn_processing_functions.convert_output(output))
            self.rate.sleep()

    def init_publisher(self):
        self.eddi_out_0_pub = rospy.Publisher('eddi/sinadra/human_injury_criticality_in_area', BayesianNetworkOutput)


    def init_subscriber(self):
        rospy.Subscriber('control_station/dense_building_area', Bool, self.out_0_callback)
        rospy.Subscriber('drone/highest_temperature_in_area', Float32, self.out_1_callback)
        rospy.Subscriber('control_station/risk_of_fire_and_explosions_in_area', Bool, self.out_2_callback)
        rospy.Subscriber('area_distance_to_catastrophy_epicenter', Float32, self.out_3_callback)

    def check_validity (self):
        current_time = time.time()


def main():
    monitor = EDDIMonitor()
    node = RosNode(monitor)
    node.ros_main()
    

if __name__ == '__main__':
    main()

