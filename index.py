import threading
import time
import json
from server import mqtt_client, scale_topic, sensors_topic
from pyniryo import *

# Global variables to store latest data
latest_scale_data = None
latest_sensors_data = None

# Callback function when a message is received
def message_received(client, userdata, message):
    global latest_scale_data, latest_sensors_data
    payload = json.loads(message.payload.decode("utf-8"))
    print("Message received:", payload)
    print("Topic:", message.topic)

    if message.topic == scale_topic:
        latest_scale_data = payload
        #print("Latest Scale Data:", latest_scale_data)
    elif message.topic == sensors_topic:
        latest_sensors_data = payload
        #print("Latest Sensors Data:", latest_sensors_data)

# Function to handle MQTT client connection and subscription
def mqtt_connect_and_subscribe():
    try:
        # Connect to AWS IoT
        mqtt_client.connect()
        print("Connected to AWS IoT")

        # Subscribe to the scale_data topic
        mqtt_client.subscribe(scale_topic, 1, message_received)
        print("Subscribed to topic:", scale_topic)

        # Subscribe to the sensors_data topic
        mqtt_client.subscribe(sensors_topic, 1, message_received)
        print("Subscribed to topic:", sensors_topic)
            
    except Exception as e:
        print("Error:", str(e))

# Creating and starting a new thread for MQTT client connection and subscription
mqtt_thread = threading.Thread(target=mqtt_connect_and_subscribe)
mqtt_thread.daemon = True  # Daemonize the thread so it automatically exits when the main program exits
mqtt_thread.start()

# Function to get the latest scale data
def get_latest_scale_data():
    global latest_scale_data
    return latest_scale_data

# Function to get the latest sensors data
def get_latest_sensors_data():
    global latest_sensors_data
    return latest_sensors_data

robot_ip_address = "192.168.8.102"  # robot ip address
workspace = "workspaceTestCy"

observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.184, y=-0.009, z=0.134,
    roll=-3.046, pitch=1.370, yaw=3.113,
)

drop_pose = PoseObject(
    x=0.251, y=0.206, z=0.056,
    roll=-0.165, pitch=1.504, yaw=0.509,
)

petri_back_from_scale = PoseObject( 
    x=0.453, y=-0.028, z=0.098,
    roll=0.069, pitch=1.451, yaw=-0.050,
)

go_to_shell = PoseObject( 
    x=0.537, y=0.154, z=0.161,
    roll=0.013, pitch=0.115, yaw=0.307,
)

petri_to_scale = PoseObject( 
    x=0.276, y=0.295, z=0.069,
    roll=0.650, pitch=1.484, yaw=1.394,
)

pick_petri_from_spot = PoseObject( 
    x=0.455, y=-0.026, z=0.100,
    roll=0.696, pitch=1.520, yaw=0.647,
)

take_shell_up = PoseObject( 
    x=0.410, y=0.085, z=0.301,
    roll=0.052, pitch=0.445, yaw=0.203,
)

shell_to_petri_spot = PoseObject( 
    x=0.463, y=-0.017, z=0.236,
    roll=-0.025, pitch=0.405, yaw=-0.038,
)

dip_shell_to_petri = PoseObject( 
    x=0.463, y=-0.017, z=0.208,
    roll=-0.025, pitch=0.405, yaw=-0.038,
)

undip_shell_from_petri = PoseObject( 
    x=0.463, y=-0.017, z=0.316,
    roll=-0.025, pitch=0.405, yaw=-0.038,
)

take_shell_down = PoseObject( 
    x=0.425, y=0.121, z=0.236,
    roll=-0.058, pitch=0.534, yaw=0.289,
)

shell_back_to_holder = PoseObject( 
    x=0.541, y=0.155, z=0.176,
    roll=0.110, pitch=0.745, yaw=0.271,
)

sleep_joints = [0.0, 0.55, -1.2, 0.0, 0.0, 0.0]

# In case you use the air vacuum, update of the height offset. Comment if you use the gripper
# z_offset = -0.01

# In case you use the large gripper, height offset is 0 by default. Comment if you use the vacuum pump
z_offset_petri = 0.009
z_offset_shell = 0.03
z_offset_default = 0.02

model = None

# Connecting to robot
client = NiryoRobot(robot_ip_address)
client.set_tcp(0.11, 0, 0, 0, 0, 0)
client.calibrate(CalibrateMode.AUTO)
client.update_tool()
tool_id = client.get_current_tool_id()
if tool_id == 11 or tool_id == 12 or tool_id == 13:  # If it is a gripper
    client.close_gripper()  # close gripper so that workspace is more visible
client.move_pose(*observation_pose.to_list())

# Test observation_pose
client.move_pose(*observation_pose.to_list())

# Start loop
def vapometer_calibration():

    client.set_arm_max_velocity(60)
    client.open_gripper(max_torque_percentage=20)
    client.move_pose(*pick_petri_from_spot.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*petri_to_scale.to_list())
    client.open_gripper(max_torque_percentage=20)
    client.move_pose(*observation_pose.to_list())
    # Publish message to scale_data topic "waiting for first scale measurement"
    mqtt_client.publish(scale_topic, json.dumps({"message": "Waiting for first scale measurement"}), 1)
    # Wait for the first scale measurement
    while True:
        latest_scale_data = get_latest_scale_data()
        if latest_scale_data and "weight" in latest_scale_data:
            break
        time.sleep(0.5)
    # Publish message to scale_data topic "first scale value received"
    mqtt_client.publish(scale_topic, json.dumps({"message": "First scale value received"}), 1)
    time.sleep(1)
    client.move_pose(*petri_to_scale.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*petri_back_from_scale.to_list())
    client.open_gripper(max_torque_percentage=20)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*go_to_shell.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    client.execute_registered_trajectory("remove_shell_from_holder")
    time.sleep(0.3)
    client.move_pose(*take_shell_up.to_list())
    client.set_arm_max_velocity(25)
    time.sleep(0.3)
    client.move_pose(*shell_to_petri_spot.to_list())
    time.sleep(0.3)
    client.move_pose(*dip_shell_to_petri.to_list())
    time.sleep(0.3)
    client.open_gripper(max_torque_percentage=20)
    # Publish message to sensors_data topic "waiting for sensors_data"
    mqtt_client.publish(sensors_topic, json.dumps({"message": "Waiting for sensors data"}), 1)
    # Wait for the sensors data
    while True:
        latest_sensors_data = get_latest_sensors_data()
        if latest_sensors_data and "temperature" in latest_sensors_data:
            break
        time.sleep(0.3)
    # Publish message to sensors_data topic "sensors value received"
    mqtt_client.publish(sensors_topic, json.dumps({"message": "Sensors value received"}), 1)
    time.sleep(0.3)
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    time.sleep(0.3)
    client.move_pose(*undip_shell_from_petri.to_list())
    time.sleep(0.3)
    client.set_arm_max_velocity(60)
    client.move_pose(*take_shell_down.to_list())
    time.sleep(0.3)
    client.move_pose(*shell_back_to_holder.to_list())
    time.sleep(0.6)
    client.execute_registered_trajectory("drop_shell_back")
    time.sleep(0.9)
    client.open_gripper(max_torque_percentage=10)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*pick_petri_from_spot.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*petri_to_scale.to_list())
    client.open_gripper(max_torque_percentage=20)
    client.move_pose(*observation_pose.to_list())
    # Publish message to scale_data topic "waiting for second scale measurement"
    mqtt_client.publish(scale_topic, json.dumps({"message": "Waiting for second scale measurement"}), 1)
    # Wait for the second scale measurement
    while True:
        latest_scale_data = get_latest_scale_data()
        if latest_scale_data and "weight" in latest_scale_data:
            break
        time.sleep(0.5)
    # Publish message to scale_data topic "second scale value received"
    mqtt_client.publish(scale_topic, json.dumps({"message": "Second scale value received"}), 1)
    time.sleep(1)
    client.move_pose(*petri_to_scale.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)
    client.move_pose(*observation_pose.to_list())
    client.move_pose(*petri_back_from_scale.to_list())
    client.open_gripper(max_torque_percentage=20)
    client.move_pose(*observation_pose.to_list())
    client.close_gripper(max_torque_percentage=20, hold_torque_percentage=100)

    mqtt_client.publish(scale_topic, json.dumps({"message": "Calibration finished"}), 1)
    
    while True:
        latest_scale_data = get_latest_scale_data()
        if latest_scale_data and "start" in latest_scale_data:
            vapometer_calibration()

# Call the function to start the loop
vapometer_calibration()
