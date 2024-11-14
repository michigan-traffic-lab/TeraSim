import os
import json
import redis
import time
import threading
from typing import Any, Mapping, List
from loguru import logger

import terasim_cosim.constants as constants
from terasim_cosim.redis_msgs import GeneralMsg


class TeraSimRedisClientGeneral:
    """TeraSimRedisClientGeneral is a wrapper class for Redis client."""

    pubsub_sleep_time = 0.001

    def __init__(
        self,
        key_value_config: dict = {},
        remote_flag: bool = False,
        remote_redis_config: dict = {},
        pub_channels: list = [],
        sub_channels: list = [],
        latency_src_channels: list = [],
    ):
        """Initialize the Redis client.

        Args:
            key_value_config (dict, optional): Configuration of the Redis key and value type. Defaults to {}.
            remote_flag (bool, optional): Whether it is necessary to create a Redis client connected to the remote server. Defaults to False.
            remote_redis_config (dict, optional): Configuration of the remote Redis server. Defaults to {}.
            pub_channels (list, optional): List of channels to publish. Defaults to [].
            sub_channels (list, optional): List of channels to subscribe. Defaults to [].
            latency_src_channels (list, optional): Channel name for latency data source. Defaults to [].
        """
        self.remote_flag = remote_flag

        # Define the Redis client connected to localhost.
        self.local_client = redis.Redis(host="localhost", port=6379, db=0, password="")
        # Define the configuration of the Redis key and value type.
        self.key_value_config = key_value_config

        self.remote_client = None
        self.sub_thread = None
        self.pub_threads = []
        self.stop_pub_threads = False
        if remote_flag:
            # Define the Redis client connected to a remote server.
            redis_host = remote_redis_config.get(
                "host", os.environ.get(constants.ENVIRONMENT_VARIABLE_HOST)
            )
            redis_port = remote_redis_config.get(
                "port", os.environ.get(constants.ENVIRONMENT_VARIABLE_PORT)
            )
            redis_password = remote_redis_config.get(
                "passward", os.environ.get(constants.ENVIRONMENT_VARIABLE_PASSWORD)
            )
            self.remote_client = redis.Redis(
                host=redis_host, port=redis_port, db=0, password=redis_password
            )

            # Create subprocesses for publishing and subscribing messages.
            if pub_channels != []:
                for channel in pub_channels:
                    pub_thread = threading.Thread(
                        target=self.helper_publish_message, args=(channel,)
                    )
                    self.pub_threads.append(pub_thread)
                    pub_thread.start()
            if sub_channels != []:
                self.pubsub = self.remote_client.pubsub(ignore_subscribe_messages=True)
                channels_dict = {}
                for channel in sub_channels:
                    logger.info(f"Subscribing message on channel {channel}")
                    channels_dict[channel] = self.helper_subscribe_message
                self.pubsub.subscribe(**channels_dict)
                self.sub_thread = self.pubsub.run_in_thread(
                    sleep_time=self.pubsub_sleep_time
                )

        self.latency_src_channels = latency_src_channels
        print("Redis client is initialized!")

    def get(self, key: str, mode: str = "local") -> Any:
        """Overwrite the get method of Redis client.

        Args:
            key (str): Redis key.
            mode (str, optional): Where to get the Redis value. Defaults to "local".

        Raises:
            ValueError: When the Redis key is not defined during initialization. we will raise a ValueError.

        Returns:
            Any: String or object of the Redis value.
        """
        client = self.helper_choose_client(mode)
        value = client.get(key)
        if value is None:
            return None
        if key not in self.key_value_config:
            raise ValueError(f"Key {key} is not defined in key_value_config!")
        value_type = self.key_value_config[key]
        if value_type is None:
            logger.info(
                f"Message type is not clearly defined for redis key {key}! Get it as string."
            )
            return value
        return value_type.model_validate_json(value)

    def set(self, key: str, value: Any, mode: str = "local") -> bool:
        """Overwrite the set method of Redis client.

        Args:
            key (str): Redis key.
            value (Any): Redis value.
            mode (str, optional): Where to set the Redis value. Defaults to "local".

        Raises:
            ValueError: When the Redis key is not defined during initialization. we will raise a ValueError.
            ValueError: When the Redis value is not of the correct type, we will raise a ValueError.

        Returns:
            bool: Whether the Redis value is set successfully.
        """
        client = self.helper_choose_client(mode)
        if key not in self.key_value_config:
            raise ValueError(f"Key {key} is not defined in key_value_config!")
        value_type = self.key_value_config[key]
        if value_type is None:
            logger.info(
                f"Message type is not clearly defined for redis key {key}! Set it as string."
            )
            assert isinstance(value, str), f"Value {value} is not of type string!"
            return client.set(key, value)
        if not isinstance(value, value_type):
            raise ValueError(
                f"Value {value} is of type {type(value)}, but we need type {value_type}!"
            )
        value_json = value.model_dump_json()
        return client.set(key, value_json)

    def mget(self, keys: List[str], mode: str = "local") -> List[Any]:
        """Overwrite the mget method of Redis client.

        Args:
            keys (List[str]): List of the Redis keys.
            mode (str, optional): Where to mget the Redis values. Defaults to "local".

        Raises:
            ValueError: When the Redis key is not defined during initialization. we will raise a ValueError.

        Returns:
            List[Any]: List of the Redis values with pre-defined value types (string or object).
        """
        client = self.helper_choose_client(mode)
        values = client.mget(keys)
        final_values = []
        for i in range(len(keys)):
            key = keys[i]
            value = values[i]
            if value is None:
                final_values.append(None)
                continue
            if key not in self.key_value_config:
                raise ValueError(f"Key {key} is not defined in key_value_config!")
            value_type = self.key_value_config[key]
            if value_type is None:
                logger.info(
                    f"Message type is not clearly defined for redis key {key}! Get it as string."
                )
                final_values.append(value)
                continue
            value = value_type.model_validate_json(value)
            final_values.append(value)
        return final_values

    def mset(self, mapping: Mapping[str, Any], mode: str = "local") -> bool:
        """Overwrite the mset method of Redis client.

        Args:
            mapping (Mapping[str, Any]): Mapping of the Redis keys and values with correct types.
            mode (str, optional): Where to mset the Redis values. Defaults to "local".

        Raises:
            ValueError: When the Redis key is not defined during initialization. we will raise a ValueError.
            ValueError: When the Redis value is not of the correct type, we will raise a ValueError.

        Returns:
            bool: Whether the Redis values are set successfully.
        """
        client = self.helper_choose_client(mode)
        mapping_json = {}
        for key in mapping:
            if key not in self.key_value_config:
                raise ValueError(f"Key {key} is not defined in key_value_config!")
            value = mapping[key]
            value_type = self.key_value_config[key]
            if value_type is None:
                logger.info(
                    f"Message type is not clearly defined for redis key {key}! Set it as string."
                )
                assert isinstance(
                    mapping[key], str
                ), f"Value {mapping[key]} is not of type string!"
                mapping_json[key] = mapping[key]
                continue
            if not isinstance(mapping[key], value_type):
                raise ValueError(
                    f"Value {value} is of type {type(value)}, but we need type {value_type}!"
                )
            mapping_json[key] = mapping[key].model_dump_json()
        return client.mset(mapping_json)

    def helper_publish_message(self, channel: str) -> None:
        """Helper function to publish messages on the channel of the remote Redis server.

        Args:
            channel (str): Channel name.
        """
        logger.info(f"Publishing message on channel {channel}")
        prev_timestamp = None
        while not self.stop_pub_threads:
            message = self.local_client.get(channel)
            if message is not None:
                data = json.loads(message)
                assert (
                    "header" in data
                ), f"Key 'header' is not in the value of the Channel '{channel}'"
                if (
                    prev_timestamp is None
                    or prev_timestamp != data["header"]["timestamp"]
                ):
                    prev_timestamp = data["header"]["timestamp"]
                    self.remote_client.publish(channel, message)
                    if channel == "debug":
                        logger.debug(
                            f"Publishing message on channel {channel}. Timestamp: {data['header']['timestamp']}"
                        )
            time.sleep(self.pubsub_sleep_time)

    def helper_subscribe_message(self, message: Any) -> None:
        """Helper function to subscribe messages on the channel of the remote Redis server.

        Args:
            message (Any): Message received from the channel.
        """
        channel = message["channel"].decode()
        message = message["data"].decode()
        self.local_client.set(channel, message)
        if channel == "debug":
            data = json.loads(message)
            latency = time.time() - float(data["header"]["timestamp"])
            logger.debug(f"Received message on channel {channel}. Latency: {latency}")
        if channel in self.latency_src_channels:
            data = json.loads(message)
            latency = time.time() - float(data["header"]["timestamp"])
            logger.debug(f"Received message on channel {channel}. Latency: {latency}")
            latency_msg = GeneralMsg()
            latency_msg.header.timestamp = time.time()
            latency_msg.data = str(latency)
            latency_msg_str = latency_msg.model_dump_json()
            self.local_client.set(constants.TERASIM_LATENCY, latency_msg_str)

    def helper_choose_client(self, mode: str) -> redis.Redis:
        """Helper function to choose the Redis client based on the mode.

        Args:
            mode (str): Location of the Redis server the client should connect to.

        Raises:
            ValueError: When the mode is not supported, we will raise a ValueError.

        Returns:
            redis.Redis: Chosen Redis client.
        """
        if mode == "local":
            return self.local_client
        elif mode == "remote":
            assert (
                self.remote_client is not None
            ), "Remote Redis client is not initialized!"
            return self.remote_client
        else:
            raise ValueError(f"Mode {mode} is not supported!")

    def flushall(self) -> bool:
        """Overwrite the flushall method of Redis client.

        Returns:
            bool: Whether the Redis database is flushed successfully.
        """
        for client in [self.local_client, self.remote_client]:
            if client is not None:
                client.flushall()

    def close(self):
        """Close the publisher and subscriber threads."""
        if self.sub_thread is not None:
            self.sub_thread.stop()
        self.stop_pub_threads = True
        for pub_thread in self.pub_threads:
            pub_thread.join()


def create_redis_client(
    key_value_config={},
    remote_flag=False,
    remote_redis_config={},
    pub_channels=[],
    sub_channels=[],
    latency_src_channels=[],
) -> TeraSimRedisClientGeneral:
    """Helper function to create a TeraSim Redis client.

    Args:
        key_value_config (dict, optional): Configuration of the Redis key and value type. Defaults to {}.
        remote_flag (bool, optional): Whether it is necessary to create a Redis client connected to the remote server. Defaults to False.
        remote_redis_config (dict, optional): Configuration of the remote Redis server. Defaults to {}.
        pub_channels (list, optional): List of channels to publish. Defaults to [].
        sub_channels (list, optional): List of channels to subscribe. Defaults to [].
        latency_src_channels (list, optional): Channel name for latency data source. Defaults to [].

    Returns:
        TeraSimRedisClientGeneral: Redis client for TeraSim co-simulation.
    """
    return TeraSimRedisClientGeneral(
        key_value_config=key_value_config,
        remote_flag=remote_flag,
        remote_redis_config=remote_redis_config,
        pub_channels=pub_channels,
        sub_channels=sub_channels,
        latency_src_channels=latency_src_channels,
    )
