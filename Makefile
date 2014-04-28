CC=g++


softkinetic: pcl_slam.o 
	${CC} -I /opt/softkinetic/DepthSenseSDK/include/ -L /opt/softkinetic/DepthSenseSDK/lib  src/softkinetic.cpp -lDepthSense -lDepthSensePlugins -o bin/softkinetic

pcl_slam.o:
	${CC}  -c src/pcl_slam.cpp -o pcl_slam.o

clean:
	rm -rf bin/* *.o