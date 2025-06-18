1. Install [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file)
	1. In a terminal, navigate to the location of the extracted simulator file.
	2. Run: ```bash
```
		$ cd <path/to/AutoDRIVE Simulator.x86_64> 
		$ sudo chmod +x AutoDRIVE\ Simulator.x86_64 
```

2. Install [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Devkit-0.3.0)
	1. Dependencies, run:
		1. 
	```
 		$ pip3 install eventlet==0.33.3
		$ pip3 install Flask==1.1.1
		$ pip3 install Flask-SocketIO==4.1.0
		$ pip3 install python-socketio==4.2.0
		$ pip3 install python-engineio==3.13.0
		$ pip3 install greenlet==1.0.0
		$ pip3 install gevent==21.1.2
		$ pip3 install gevent-websocket==0.10.1
		$ pip3 install Jinja2==3.0.3
		$ pip3 install itsdangerous==2.0.1
		$ pip3 install werkzeug==2.0.3
		$ pip3 install attrdict
		$ pip3 install numpy
		$ pip3 install pillow
		$ pip3 install opencv-contrib-python
	```
	2. Clone AutoDRIVE-Devkit and then move the autodrive_ros2 meta-package to source in ros2_ws.
		1. 
``` 
	$ git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git

	$ mv ~/AutoDRIVE-Devkit/autodrive_ros2 ~/ros2_ws/src/
	
	$ cd ~/ros2_ws
	$ colcon build
```
  3. 
