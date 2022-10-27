# wall_follower
This repository contains the programs used in The Construct's Rosject 1 - Turtlebot3 Wall follower, parts 1 and 2.

### Under src directory

### 1. find_wall_service_server.py
- This is the script that contains the service server code that performs the behaviour of finding the wall, and moving towards it towards the wall to get ready for the wall following behaviour.

### 2. wall_follow_client.py
- This is the script that contains the service client code that call the find_wall_service_server.
- It also contains the wall follow code. The turtlebot3 will follow the wall once the server behaviour is finished.

### Under launch directory

### 1. main.launch
- This launch file launches both nodes: server and client for the turtlebot3 to find and follow wall.
