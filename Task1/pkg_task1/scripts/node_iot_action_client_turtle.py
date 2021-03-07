#!/usr/bin/env python

import rospy
import actionlib
import time

from pkg_task1.msg import msgTurtleAction       # Message Class that is used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal         # Message Class that is used for Goal messages

import paho.mqtt.client as mqtt
import requests


def on_publish(client, userdata, mid):
    print("--- Publisher ---")



class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                msgTurtleAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

    # Function to send Goals to Action Servers


    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")

    # Function print result on Goal completion
    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        # rospy.loginfo("Result is : " + str(result))

        rospy.loginfo("result.final_x: " + str(result.final_x))
        rospy.loginfo("result.final_y: " + str(result.final_y))
        rospy.loginfo("result.final_theta: " + str(result.final_theta))

        # PUBLISH TO MQTT SUBSCRIBER

        pub_client = mqtt.Client()
        pub_client.on_publish = on_publish
        pub_client.connect("broker.mqttdashboard.com", 1883)
        
        pub_message = "result.final_x:" + str(result.final_x) + ", result.final_y:" + str(result.final_y) + ", result.final_theta:" + str(result.final_theta)
        pub_client.publish(topic="eyrc/BYgUhTYm/mqtt/ros_to_iot", payload=pub_message, qos=0, retain=False)


        # PRINT ON GOOGLE SHEET

        param_task1 = {"id":"task1", "team_id":"VB_2109", "unique_id":"BYgUhTYm", "turtle_x":result.final_x, "turtle_y":result.final_y, "turtle_theta":result.final_theta}
        param_Sheet2 = {"id":"Sheet2", "turtle_x":result.final_x, "turtle_y":result.final_y, "turtle_theta":result.final_theta}

        URL = "https://script.google.com/macros/s/AKfycbzLTlI2NID1bnj9mAoabJrSzdmnd30nIyHQ3J7zmxwCOzZTau0/exec"

        response_task1 = requests.get(URL, params=param_task1)
        response_Sheet2 = requests.get(URL, params=param_Sheet2)

        print(response_task1.content)
        print(response_Sheet2.content)



    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        pass
        # rospy.loginfo(feedback)



def on_connect(client, userdata, flags, rc):
    print("[INFO] Connected With Result Code: " + str(rc))

def on_message(client, userdata, message):
    print("--- Subscriber ---")
    print("[INFO] Topic: {}".format(message.topic) )
    print("[INFO] Message Recieved: {}".format(message.payload.decode()))
    print("------------")
    
    if(str(message.payload.decode()) == "start"):
         # 2. Create a object for Simple Action Client.
        obj_client = SimpleActionClientTurtle()

        # 3. Send Goals to Draw a Square

        rospy.sleep(20)

        obj_client.send_goal(2, 0)
        rospy.sleep(30)

        obj_client.send_goal(2, 60)
        rospy.sleep(30)

        obj_client.send_goal(2, 60)
        rospy.sleep(30)

        obj_client.send_goal(2, 60)
        rospy.sleep(30)

        obj_client.send_goal(2, 60)
        rospy.sleep(30)
        
        obj_client.send_goal(2, 60)
        rospy.sleep(30)
        
        rospy.spin()

# Main Function
def main():
    
    rospy.init_node('node_simple_action_client_turtle')

    sub_client = mqtt.Client()
    sub_client.on_connect = on_connect
    sub_client.on_message = on_message
    sub_client.connect("broker.mqttdashboard.com", 1883)

    sub_client.subscribe("eyrc/BYgUhTYm/mqtt/iot_to_ros", qos=0)

    sub_client.loop_forever()
     

if __name__ == '__main__':
    main()
