#!/bin/bash

rm -rf out.avi
xterm -e "cd ./webcam_stream && node websocket-relay.js supersecret 8081 8082" &
xterm -e "http-server ./webcam_stream/" &
xterm -e ffmpeg \
	-f v4l2 \
		-framerate 20 -video_size 300x225 -i /dev/video0 \
	-f mpegts \
		-codec:v mpeg1video -s 300x225 -b:v 500k -bf 0 \
	http://localhost:8081/supersecret &

xterm -e "cd ./neato-interface-server && node interface-server.js"
