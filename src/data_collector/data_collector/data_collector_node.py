import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
import h5py
import numpy as np
import os
import time
import importlib
from typing import Dict, Any, List
from dataclasses import dataclass
from bisect import bisect_left
import threading
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from data_collector.data_processor import DataProcessor
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def load_topics_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    return config


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        self.config = load_topics_config(config_path)

        self.is_collecting = False
        self.timer = None
        self.collected_dataset_list = []
        self.dataset_cfg_dict = {}

        self.get_logger().info(f"data_dir: {self.config['data_dir']}")
        self.get_logger().info(f"robot_number: {self.config['robot_number']}")
        self.get_logger().info(f"collection_frequency: {self.config['collection_frequency']}")
        self.get_logger().info(f"max_queue_size: {self.config['max_queue_size']}")
        self.get_logger().info(f"datasets to collect: {self.config['datasets'].keys()}")

        self.max_frames = self.config.get('max_frames', 2000)
        self.get_logger().info(f"max_frames: {self.max_frames}")

        self.bridge = CvBridge()
        self.processors: Dict[str, DataProcessor] = {}
        self.buffers: Dict[str, List] = {}
        self.locks: Dict[str, threading.Lock] = {}
        self.setup_processors()
        self.setup_subscriptions()

        self.collecting_status_pub = self.create_publisher(Bool, 'collecting_status', 10)

        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.start_srv = self.create_service(
            Trigger, 'start_collect', self.start_callback,
            callback_group=self.srv_cb_group)
        self.stop_srv = self.create_service(
            Trigger, 'stop_collect', self.stop_callback,
            callback_group=self.srv_cb_group)


    def setup_processors(self):
        datasets = self.config['datasets']
        for dataset_name, datset_value in datasets.items():
            try:
                module_name, class_name = datset_value['processor'].rsplit('.', 1)
                module = importlib.import_module(module_name)
                processor_class = getattr(module, class_name)
                self.processors[dataset_name] = processor_class(
                    datset_value.get('processor_config', {})
                )

                topic_name = datset_value['topic']
                self.buffers[topic_name] = []
                if topic_name not in self.locks.keys():
                    self.locks[topic_name] = threading.Lock()

                dataset_config = self.processors[dataset_name].get_dataset_config()
                self.get_logger().info(f"{dataset_name} config: {dataset_config}")
                self.dataset_cfg_dict[dataset_name] = dataset_config

            except Exception as e:
                self.get_logger().error(f"Failed to load processor for {dataset_name}: {str(e)}")
                raise

    def setup_subscriptions(self):
        topic_names = [ v['topic'] for _, v in self.config['datasets'].items() ]
        topic_configs = {
            topic_name: topic_cfg
            for topic_name, topic_cfg in self.config['topics'].items()
            if topic_name in topic_names
        }

        # Create QoS profile for high-frequency topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        for topic_name, topic_cfg in topic_configs.items():
            msg_module, msg_type = topic_cfg['msg_type'].rsplit('.', 1)
            module = importlib.import_module(msg_module)
            msg_class = getattr(module, msg_type)

            self.create_subscription(
                msg_class,
                topic_name,
                self.create_callback(topic_name),
                qos_profile  # Use the custom QoS profile here
            )

            self.get_logger().info(f"create sub {topic_name}")

    def create_callback(self, topic_name):
        def callback(msg):
            stamp = self.get_clock().now().nanoseconds / 1e9
            with self.locks[topic_name]:
                self.buffers[topic_name].append((stamp, msg))
                max_q = self.config['max_queue_size']
                if len(self.buffers[topic_name]) > max_q:
                    self.buffers[topic_name].pop(0)

        return callback

    def start_callback(self, request, response):
        # request is empty for Trigger
        if self.is_collecting:
            response.success = False
            response.message = "Already collecting data"
            return response

        # Get frequency from parameters instead of request
        frequency = self.config['collection_frequency']
        interval = 1.0 / frequency

        # Clear buffers to discard stale data from previous runs
        for topic_name in self.buffers:
            with self.locks[topic_name]:
                self.buffers[topic_name] = []

        self.is_collecting = True
        self.timer = self.create_timer(interval, self.collect_data)
        self.publish_collecting_status(True)

        response.success = True
        response.message = f"Started collecting at {frequency}Hz"
        return response

    def stop_callback(self, request, response):
        if not self.is_collecting:
            response.success = False
            response.message = "Not currently collecting"
            return response

        self.is_collecting = False
        if self.timer:
            self.timer.cancel()
        self.publish_collecting_status(False)

        if not self.collected_dataset_list:
            self.get_logger().warn("No frames collected, skipping save")
            response.success = False
            response.message = "No frames collected, nothing saved"
            return response

        try:
            start_time = time.time()
            file_path = self.save_data()
            delta_time = time.time() - start_time
            self.get_logger().info(f"save time cost: {delta_time}")
            del self.collected_dataset_list
            self.collected_dataset_list = []
            response.success = True
            response.message = f"Data saved to {file_path}"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def collect_data(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        processed_data_dict = {}
        max_age = self.config['max_age']

        for dataset_name, dataset_cfg in self.config['datasets'].items():
            topic_name = dataset_cfg['topic']
            with self.locks[topic_name]:
                buffer = self.buffers[topic_name]
                if not buffer:
                    self.get_logger().warn(f"No data in {topic_name}, skipping frame")
                    return
                
                stamps = [s for s, _ in buffer]
                idx = bisect_left(stamps, current_time)
                closest = self.find_closest(stamps, idx, current_time)

                if closest is None:
                    self.raise_error(f"No closet data in {topic_name}")

                delta_time = abs(buffer[closest][0] - current_time)
                self.get_logger().debug(f"topic: {topic_name}, delta time: {delta_time}")
                if delta_time > max_age:
                    self.get_logger().warn(f"Stale data in {topic_name}, delta time: {delta_time}, skipping frame")
                    return

                _, msg = buffer[closest]
                processed_data_dict[dataset_name] = self.processors[dataset_name].process(msg)

                message = self.processors[dataset_name].get_message()
                if message != '':
                    self.get_logger().warn('*'*50)
                    self.get_logger().warn(message)
                    self.get_logger().warn('*'*50)

                # always save one frame at least in buffer
                self.buffers[topic_name] = buffer[closest:]

        self.collected_dataset_list.append(processed_data_dict)
        frame_count = len(self.collected_dataset_list)
        self.get_logger().info(f"Collected frame: {frame_count}/{self.max_frames}")

        if frame_count >= self.max_frames:
            self.get_logger().info(f"Reached max frames ({self.max_frames}), auto saving...")
            self.auto_stop_and_save()
            return

        time_cost = self.get_clock().now().nanoseconds / 1e9 - current_time
        time_interval = 1.0 / self.config['collection_frequency']

        if time_cost > time_interval:
            self.get_logger().warn(f"collect_data time cost: {time_cost}, !!!!!costing too much time!!!!!")
        else:
            self.get_logger().info(f"collect_data time cost: {time_cost}")

    def auto_stop_and_save(self):
        self.is_collecting = False
        if self.timer:
            self.timer.cancel()
        self.publish_collecting_status(False)
        try:
            start_time = time.time()
            file_path = self.save_data()
            delta_time = time.time() - start_time
            self.get_logger().info(f"Auto save time cost: {delta_time}")
            self.get_logger().info(f"Auto saved to {file_path}")
            del self.collected_dataset_list
            self.collected_dataset_list = []
        except Exception as e:
            self.get_logger().error(f"Auto save failed: {str(e)}")

    def publish_collecting_status(self, status):
        msg = Bool()
        msg.data = status
        self.collecting_status_pub.publish(msg)

    def save_data(self):
        data_dir = self.config['data_dir']
        os.makedirs(data_dir, exist_ok=True)
        timestamp = int(time.time())
        file_path = os.path.join(data_dir, f'collection_{timestamp}.hdf5')
        self.get_logger().info(f"saving to {file_path}")

        with h5py.File(file_path, 'w') as hf:
            # First create all datasets and store references
            hdf5_dataset_dict = {}
            for dataset_name, dataset_cfg in self.dataset_cfg_dict.items():
                shape = (len(self.collected_dataset_list), *dataset_cfg['shape'])

                self.get_logger().info(f"{dataset_name} save shape: {shape}")

                # Create dataset with full save path
                dataset = hf.create_dataset(
                    dataset_name,
                    shape,
                    dtype=dataset_cfg['dtype'],
                    chunks=(1, *dataset_cfg['shape'])
                )
                dataset.attrs['description'] = dataset_cfg['description']
                hdf5_dataset_dict[dataset_name] = dataset

                self.get_logger().info(f"{dataset_name} dataset: {dataset}")

            # Then write data to pre-created datasets
            for i, collected_dataset_dict in enumerate(self.collected_dataset_list):
                for collected_dataset_name, collected_dataset_data in collected_dataset_dict.items():
                    if i == 0:
                        if hasattr(collected_dataset_data, 'shape'):
                            self.get_logger().info(f"{collected_dataset_name} data shape: {collected_dataset_data.shape}")
                        else:
                            self.get_logger().info(f"{collected_dataset_name} data : {collected_dataset_data}")
                    hdf5_dataset = hdf5_dataset_dict[collected_dataset_name]
                    hdf5_dataset[i] = collected_dataset_data

            hf.create_dataset(
                'collection_frequency',
                data=self.config['collection_frequency'],
                dtype='f'
            )

            hf.create_dataset(
                'robot_number',
                data=self.config['robot_number'],
                dtype='f'
            )

        self.get_logger().info(f"saved to {file_path}")

        return file_path

    def find_closest(self, stamps, idx, target):
        if idx == 0:
            return 0 if stamps else None
        elif idx == len(stamps):
            return idx - 1
        else:
            before = stamps[idx - 1]
            after = stamps[idx]
            return idx - 1 if (target - before) <= (after - target) else idx

    def raise_error(self, msg):
        self.get_logger().error(msg)
        self.is_collecting = False
        if self.timer:
            self.timer.cancel()
        self.publish_collecting_status(False)

        if self.collected_dataset_list:
            self.get_logger().info(f"Error occurred, saving {len(self.collected_dataset_list)} collected frames...")
            try:
                file_path = self.save_data()
                self.get_logger().info(f"Emergency saved to {file_path}")
            except Exception as e:
                self.get_logger().error(f"Emergency save failed: {str(e)}")

        self.collected_dataset_list = []


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
