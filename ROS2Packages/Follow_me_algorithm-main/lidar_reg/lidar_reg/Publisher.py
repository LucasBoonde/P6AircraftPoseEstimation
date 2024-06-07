import paho.mqtt.client as mqtt
import json
import time

# Initialize MQTT client
client = mqtt.Client()

# Connect to the broker
client.connect("broker.hivemq.com", 1883, 60)

# Coordinates to publish
coordinates_list = [
    [0.86, 0.27],  # Point 1
    [1.46, 0.83],  # Point 2
    [1.46, -0.83], # Point 3
    [0.86, -0.27], # Point 4
    [0.0, 0.0]     # Home
]

# Function to publish coordinates
def publish_coordinates(coords):
    payload = json.dumps({"latitude": coords[0], "longitude": coords[1]})
    client.publish("coordinates/topic", payload)
    print(f"Published coordinates: {coords}")

# Publish each coordinate with a delay
for coords in coordinates_list:
    publish_coordinates(coords)
    time.sleep(2)  # Adjust delay as needed

# Disconnect the client
client.disconnect()
