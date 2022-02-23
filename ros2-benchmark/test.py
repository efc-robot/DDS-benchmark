import os
import subprocess
import numpy as np
import argparse
import time
parser = argparse.ArgumentParser()
parser.add_argument("--dds", required=True, nargs='+', choices=["fastrtps", "cyclone"], help="choose one or more dds for test.")
parser.add_argument("--net", required=True, choices=["adhoc", "wireless", "wired"], help="choose the way of networking")
parser.add_argument("--size", default="64KB", nargs='+', help="choose the size of msg")
parser.add_argument("--count", default=1000, type=int, help="the number of tests")
#freq = {"10B":10, "100B":20, "1K":50, "10K":100, "100K":200, "1M":1000}

args = parser.parse_args()
def ros_run(dds, net, size, count):
    result_dir = os.path.join("/home/ubuntu/ros2_results", net, dds)
    log_filename = "{}.txt".format(size)
    log_file = open(os.path.join(result_dir, log_filename), 'w')
    if dds == "fastrtps":
        talker = subprocess.Popen("RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run latency talker {} {}".format(size, count), stdout=log_file, stderr=subprocess.STDOUT, shell=True)
    elif dds == "cyclone":
        talker = subprocess.Popen("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run latency talker {} {}".format(size, count), stdout=log_file, stderr=subprocess.STDOUT, shell=True)
    subprocess.Popen.wait(talker)
    time.sleep(10)
    log_file.close()
    print("Finish {} test with msg size {}".format(dds, size))
        
def main():
    for dds in args.dds:
        for size in args.size:
            print("Running {} test with msg size {}".format(dds, size))
            ros_run(dds, args.net, size, args.count)
               
if __name__ == "__main__":
    main()

