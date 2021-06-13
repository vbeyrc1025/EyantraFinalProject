#! /usr/bin/env python

from multiprocessing.dummy import Pool
import time
import requests

import sys
import paho.mqtt.client as mqtt #import the client1
import time


class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
    
def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    """
    This function starts a thread for subscribing to mqtt topic.
    Parameters : URL of the broker
                 Port of the broker
                 Name of mqtt topic to be subscribed
                 Quality of message published by mqtt publisher(QoS)
    Returning type : Returns 0 on successfull subscription otherwise returns -1
    """
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_pkg_dit, arg_mqtt_qos):
    """
    This function publishes data provided on the mqtt topic.
    Parameters : URL of the broker
                 Port of the broker
                 Name of mqtt topic on which data is to be published
                 A dictionary arg_pkg_dit containing details of the order
                 Quality of message published by mqtt publisher(QoS) 
    """
    try: 

        #parameters = {"id":"Inventory","id1":"Sheet2", "Team Id":"vb_1025", "Unique Id":"RpAaSgSa", "SKU":arg_x, "Items":arg_y, "Cost":arg_theta} 
        #arg_pkg_dit["id"] = "Inventory"
        
        #URL1 =   "https://script.google.com/macros/s/AKfycbzzAm85w5gXOlqZd1ux_whHnM2HG2IrydTSaB_mmwMMwhvS3nucCbsZkg/exec"
        #URL1 =   #"https://script.google.com/macros/s/AKfycbzsGE4Pz5O3x43Q2S8pYZdp6TcWFTUVKwME0tJbnroPf6OT1I1M_o-b/exec"
        URL1 =   "https://script.google.com/macros/s/AKfycbwgnD75BiCDTToXdsY9SiHegTtE-T8BUntqt3pr8JWO3lBah8sTsXYdiQ/exec"
			#https://docs.google.com/spreadsheets/d/1e5kdWTpRGpYrf_LlEzxKefqHUVVYlShIKOXmnn1_hMc/edit#gid=0

        #message= "("+str(arg_x)+" ,"+str(arg_y)+" ,"+str(arg_theta)+")"
        message= str(arg_pkg_dit)
        print("iot")
        print(message)
        print("1")
       # arg_mqtt_topic='/ros_iot_bridge/mqtt/sub'       
        mqtt_client = mqtt.Client("mqtt_pub")
        print("11")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        print("111")
        mqtt_client.loop_start()
        print("1111")

        print("Publishing message to topic", arg_mqtt_topic)
        print("11111")
        mqtt_client.publish(arg_mqtt_topic, message, arg_mqtt_qos)
        time.sleep(0.5) # wait
        print("111111")

        response = requests.get(URL1, params=arg_pkg_dit)
        print("1111111")
        #response = requests.get(URL2, params=parameters)

        print(response.content)
        print("ghya aalo me")



        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1


#URL2="https://docs.google.com/spreadsheets/d/1e5kdWTpRGpYrf_LlEzxKefqHUVVYlShIKOXmnn1_hMc/edit#gid=0"  shell webpages link for spreadsheet

