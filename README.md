# gece_es

MIRAKSA package to run the ground station node. This package handles all the communication between the UAV and the referee server. 


## Usage
There are already some launch files that can be tried but you would need to run some commands and nodes before running the launch file.

### Simulate teams
This launch file can be used to simulate the competition where there are many teams that will have to fly at once. 
Run this launch file when you need to test the chase algorithm. Assuming that you already build the packages:

```shell
# Run QGC before executing this commands on terminal

# Terminal 1
cd /path/to/dummy_server
docker compose -f docker-compose.main.yaml up -d --build

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
ros2 run ros_gz_image image_bridge /camera

# Terminal 4 
cd /path/to/gece_es
./launch_planes.sh 4

# Terminal 5
ros2 launch gece_es simulateTeams.launch.py

# Terminal 5 (optional)
ros2 run rqt_image_view rqt_image_view
```

After running all those, there should be hundreds of topic that is currently available. since we are running 4 planes using the shell script, we should be able to see 4 devices on the topics.

Run this to check
```shell
ros2 topic list | grep device
```
From each device, they should be able to listen to each other telemetry using requests. The requests is runnning from `processTelemetry.py` node.
## TODO 

- Make the UI for things like visual feedback, air and ground target location, and jamming area. The UI should also be able to send commands and flight mode using ros2 topic. 