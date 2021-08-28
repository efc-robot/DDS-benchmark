# DDS-benchmark
Benchmarking for the Performance of Multi-Robot Networking Techniques and Data Distribution Service in Dynamic Topology
> This work performs tests on:
> - discovery time of four DDS (fast-DDS, openDDS, RTI connext and openSplice) in different ways of networking
> - discovety time of ROS2 with three DDS (fast-DDS, RTI connext and openSplice) in different ways of networking
> - communication performance of DDS and ROS2

# DDS installaion
> introduce how to install DDS in debug mode
## fast-DDS
1. install dependencies
```(shell)
sudo apt install cmake g++ python3-pip wget git
```
2. Download *eProsima_Fast-DDS-2.3.0-Linux.tvz* from the [official website](https://www.eprosima.com/index.php/downloads-all) or [Tsinghua netdisk](https://cloud.tsinghua.edu.cn/f/720c51b015064caba036/?dl=1)

3. modify source code to get broadcast time
- edit file $(FAST_DDS_DIR)/src/fastrtps/src/cpp/rtps/builtin/discovery/participant/PDP.cpp. Find function `announceParticipantState`, uncomment the logInfo line (line 411)
```cpp
logInfo(RTPS_PDP, "Announcing RTPSParticipant State (new change: " << new_change << ")");
```
4. Uncompress the file, enter the directory and edit the **install.sh** file to support debug mode.

- find the following lines to build fastrtps, add flags "**-DCMAKE_BUILD_TYPE=Debug**", "**-DLOG_NO_INFO=OFF**",  "**-DINTERNAL_DEBUG=ON**" when doing cmake.
```shell
mkdir -p build/fastrtps
cd build/fastrtps
cmake ../../src/fastrtps -DCMAKE_BUILD_TYPE=Debug -DLOG_NO_INFO=OFF -DINTERNAL_DEBUG=ON
make -j8 install
cd ../..
```

5. install by run install.sh

```
sudo ./install.sh
```
6. install Fast-DDS-Gen tool
- Download *Fast-DDS-Gen-master* using git or from [Tsinghua netdisk](https://cloud.tsinghua.edu.cn/f/9304affa89644f5bb5f4/?dl=1)
```
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
```
- Download and install *java jdk 16.01* from the [official website](https://www.oracle.com/java/technologies/javase-jdk16-downloads.html) or [Tsinghua netdisk](https://cloud.tsinghua.edu.cn/f/bd5891402dbc4bbf95d6/?dl=1)
- Use gradle to install the tool
```
sudo ./gradlew assemble
```

[reference](https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#fastddsgen-sl)
## OpenSplice
## OpenDDS
## RTI connext
