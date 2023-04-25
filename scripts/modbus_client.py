#!/usr/bin/env python
import rospy
import datetime
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from ros_pss4000.msg import SafetyEvent, SafetyStatus

class Safety:

    def __init__(self,client):
        self.pub = rospy.Publisher("/safety_event",SafetyEvent, queue_size = 10)
        self.client = client
        self.Status = {}
        self.Events = {}

    def SafetyStatusCallback(self):
        self.response = self.client.read_coils(1,9)
        self.status_list = {"Safety_OK":self.response.bits[0],"Safety_OK_2":self.response.bits[1]}
        for key, val in self.status_list.items():
            if key not in self.Status.keys():
                if bool(val) == False:
                    self.Status[key] = val
                    self.SafetyStatusPublisher(key,val)
            else:
                if bool(val) == True:
                    self.Status.pop(key)
                    self.SafetyStatusPublisher(key,val)
        

    def SafetyEventsCallback(self):
        self.response = self.client.read_coils(1,9)
        self.event_list = {"EmStop1":self.response.bits[6],"EmStop2":self.response.bits[7],"Coupling_IRM_TECNALIA":self.response.bits[8]}
        for key, val in self.event_list.items():
            if key not in self.Events.keys():
                if bool(val) == False:
                    self.Events[key] = val
                    self.SafetyEventPublisher(key,val)
            else:
                if bool(val) == True:
                    self.Events.pop(key)
                    self.SafetyEventPublisher(key,val)

    def SafetyStatusPublisher(self,key,val):
        msg = SafetyStatus()
        msg.Time = datetime.datetime.now()
        msg.Source = key
        msg.Status = val
        msg.Description = "More information"
        if val == False:
            msg.Description = "Reset needed"
        else:
            msg.Description = "PLC OK"
        
        
        rospy.logerr(msg)

    def SafetyEventPublisher(self, event, val):
        info_1 = rospy.get_param("/"+event+"_1")
        info_2 = rospy.get_param("/"+event+"_2")
        if val == False:
            data = info_1
        else:
            data = info_2
        msg = SafetyEvent()
        msg.Event_ID = data[0]
        msg.Time = datetime.datetime.now()
        msg.Source = event
        msg.Description = data[2]

        rospy.logwarn(msg)

if __name__ == '__main__':
    try:
        rospy.init_node("Safety_msgs", anonymous=True)
        rospy.Rate(10) #1Hz
        client = ModbusClient('192.168.0.1', port=502)
        client.connect()
        safety_event = Safety(client)

        while not rospy.is_shutdown():
            safety_event.SafetyStatusCallback()
            safety_event.SafetyEventsCallback()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass