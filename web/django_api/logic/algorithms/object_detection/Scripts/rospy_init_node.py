import rospy

rospy.init_node('dji_input', log_level=rospy.DEBUG, disable_signals=True)
rospy.spin()