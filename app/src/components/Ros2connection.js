import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';

const Ros2Connection = () => {
  const [connected, setConnected] = useState(false);
  const [messages, setMessages] = useState({});

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.100.152:9090' // URL of the rosbridge server
    });

    ros.on('connection', () => {
      console.log('Connected to ROS Bridge!');
      setConnected(true);
    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
      setConnected(false);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed.');
      setConnected(false);
    });

    // Define the topics to subscribe to
    const topics = [
      { name: '/chatter', messageType: 'std_msgs/String' },
      { name: '/fmu/out/vehicle_local_position', messageType: 'px4_msgs/VehicleLocalPosition' }
    ];

    // Create listeners for each topic
    const listeners = topics.map(topic => {
      const listener = new ROSLIB.Topic({
        ros: ros,
        name: topic.name,
        messageType: topic.messageType
      });

      listener.subscribe((msg) => {
        console.log(`Received message on ${topic.name}:`, msg.data);
        setMessages(prevMessages => ({
          ...prevMessages,
          [topic.name]: msg.data
        }));
      });

      return listener;
    });

    // Clean up on component unmount
    return () => {
      listeners.forEach(listener => listener.unsubscribe());
      ros.close();
    };
  }, []);

  return (
    <div>
      <h1>ROS 2 Connection Status: {connected ? 'Connected' : 'Disconnected'}</h1>
      {Object.keys(messages).map(topic => (
        <h2 key={topic}>Latest Message on {topic}: {messages[topic]}</h2>
      ))}
    </div>
  );
};

export default Ros2Connection;
