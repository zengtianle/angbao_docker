docker run -it -P\
	--name angbao_robot \
	-v /etc/localtime:/etc/localtime:ro \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/nvidia/work/autoslam_gm:/root/autoslam_gm \
	-e DISPLAY=unix$DISPLAY \
	--network=host \
	--device=/dev/ttyUSB0 \
	--device=/dev/ttyUSB1 \
	ros-kinetic:v3  /bin/bash
