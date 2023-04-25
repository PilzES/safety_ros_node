#!/usr/bin/env python
import rospy
from pymodbus.client.sync import ModbusTcpClient as ModbusClient


if __name__ == '__main__':
    rospy.init_node("Safety_msgs")
    client = ModbusClient('192.168.0.1', port=502)
    client.connect()

    while not rospy.is_shutdown():
        response = client.read_coils(1,3)
        print("Safety_OK: ",response.bits[0])
        print("EmStop_OK: ",response.bits[1])
        print("SGate_OK: ",response.bits[2])
        print("-------------------------------")
        rospy.sleep(1)
        

    rospy.on_shutdown(client.close())
    rospy.spin()