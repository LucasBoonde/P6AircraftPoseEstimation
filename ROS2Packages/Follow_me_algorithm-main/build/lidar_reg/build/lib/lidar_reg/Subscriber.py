import paho.mqtt.client as mqtt

# Callback when a message is received
def on_message(client, userdata, msg):
    try:
        # Print the topic and the message payload
        print(f"Received message on topic {msg.topic}: {msg.payload.decode('utf-8')}")
    except Exception as e:
        print(f"Error processing message: {e}")

# Initialize MQTT client
client = mqtt.Client()
client.on_message = on_message

# Connect to the broker
client.connect("broker.hivemq.com", 1883, 60)

# Subscribe to the topic
client.subscribe("coordinates/topic")

# Start the MQTT client
client.loop_start()

# Main loop to keep the script running and listening for messages
def main():
    try:
        while True:
            # You can perform other tasks here if necessary
            pass  # Infinite loop to keep the script running
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()
if __name__ == '__main__':
    main()
