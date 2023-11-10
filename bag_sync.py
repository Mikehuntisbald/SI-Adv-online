import rosbag2_py
import rclpy
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import set_message

# Initialize ROS 2
rclpy.init()

# Set up the input and output bag file paths
input_bag_path = '/home/niubi'
output_bag_path = '/home/synced_niubi'

# Set up the storage options for reading and writing
storage_options = StorageOptions(uri=input_bag_path, storage_id='sqlite3')
reader = SequentialReader()

# Set up the converter options
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

# Open the input bag
reader.open(storage_options, converter_options)

# Get all topics with types from the input bag
topics_with_types = reader.get_all_topics_and_types()

# Create a new SequentialWriter for the output
writer = SequentialWriter()

# Set up the storage options for output
output_storage_options = StorageOptions(uri=output_bag_path, storage_id='sqlite3')
writer.open(output_storage_options, converter_options)

# Add topics to the writer
for topic_with_type in topics_with_types:
    topic_metadata = TopicMetadata(name=topic_with_type.name, type=topic_with_type.type, serialization_format='cdr')
    writer.create_topic(topic_metadata)

# Prepare a map from topic names to message types
topic_type_map = {t.name: t.type for t in topics_with_types}

# Read all messages, extract their internal timestamps, and write them into the new bag
messages = []

# Collect messages with their internal timestamps
while reader.has_next():
    (topic, data, t) = reader.read_next()
    # Get the message type class from the topic type
    msg_type = get_message(topic_type_map[topic])
    # Deserialize the message
    msg = deserialize_message(data, msg_type)
    # Extract timestamp from the message's header (assumes the message has a header)
    # Check if the message is a TFMessage and handle accordingly
    if msg_type.__name__ == 'TFMessage':
        for transform in msg.transforms:
            internal_timestamp = Time(seconds=transform.header.stamp.sec, nanoseconds=transform.header.stamp.nanosec)
            messages.append((internal_timestamp, topic, data))
    else:
        # For other messages that have a header
        internal_timestamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
        messages.append((internal_timestamp, topic, data))

# Sort messages by their internal timestamps
messages.sort(key=lambda x: x[0].nanoseconds)

# Write messages in the new bag
for internal_timestamp, topic, data in messages:
    # Convert the internal timestamp back to a tuple (sec, nanosec)
    seconds, nanoseconds = internal_timestamp.seconds_nanoseconds()
    timestamp_ns = int(seconds * 1e9) + nanoseconds

    
    writer.write(topic, data, timestamp_ns)

# Shutdown ROS 2
rclpy.shutdown()

