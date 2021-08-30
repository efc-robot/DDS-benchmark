import os
import argparse
import numpy as np
import time

parser = argparse.ArgumentParser(description="deal with dds output and calculate time")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds)")

args = parser.parse_args()

def extract_time_from_net(file):
    lines = file.readlines()
    return int(float(lines[0])*1000), int(float(lines[1])*1000)


def extract_time_from_sub(file, dds):
    t_sub_find_pub = 0
    t_first_received = 0
    if dds == "fastdds":
        lines = file.readlines()
        for line in lines:
            if "RTPS_PDP_DISCOVERY" in line:
                item = line.split()
                print(item)
                t_sub_find_pub = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
            if "RTPS_HISTORY" in line:
                print(item)
                item = line.split()
                t_first_received = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                break
        return t_sub_find_pub, t_first_received

def extract_time_from_pub(file, dds):
    t_pub_find_sub = 0
    t_first_sent = 0

    if dds == "fastdds":
        lines = file.readlines()
        for line in lines:
            if "RTPS_PDP_DISCOVERY" in line:
                item = line.split()
                print(item)
                t_pub_find_sub = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
            if "RTPS_HISTORY" in line:
                print(item)
                item = line.split()
                t_first_sent = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                break
        return t_pub_find_sub, t_first_sent

def main():
    for dds in args.dds:
        result_dir = os.path.join(dds, "results")
        files = os.listdir(result_dir)
        pub_results = [file for file in files if "pub" in file]
        sub_results = [file for file in files if "sub" in file]
        net_results = [file for file in files if "net" in file]

        t_start = np.zeros(len(pub_results), dtype=int)
        t_ip = np.zeros(len(pub_results), dtype=int)
        t_pub_find_sub = np.zeros(len(pub_results), dtype=int)
        t_sub_find_pub = np.zeros(len(pub_results), dtype=int)
        t_first_sent = np.zeros(len(pub_results), dtype=int)
        t_first_received = np.zeros(len(pub_results), dtype=int)

        for i in range(len(pub_results)):
            with open(os.path.join(result_dir, net_results[i]), 'r') as f:
                t_start[i], t_ip[i] = extract_time_from_net(f)
            with open(os.path.join(result_dir, sub_results[i]), 'r') as f:
                t_sub_find_pub[i], t_first_received[i] = extract_time_from_sub(f, dds)
            with open(os.path.join(result_dir, pub_results[i]), 'r') as f:
                t_pub_find_sub[i], t_first_sent[i] = extract_time_from_pub(f, dds)
        print(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent)

if __name__ == "__main__":
    main()

        