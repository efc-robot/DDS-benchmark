import os
import argparse
import numpy as np
import time
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description="deal with dds output and calculate time")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds)")
parser.add_argument('--net', required=True, choices=['adhoc', 'wired', 'wireless'], help="choose the way of networking")


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
                #print(item)
                t_sub_find_pub = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
            if "HelloWorld: Change" in line:
                #print(item)
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
                #print(item)
                t_pub_find_sub = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
            if "RTPS_HISTORY" in line:
                #print(item)
                item = line.split()
                t_first_sent = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                break
        return t_pub_find_sub, t_first_sent

def plot_figure(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent, dds, net):
    x = range(1,len(t_start)+1)
    plt.plot(x, t_ip-t_start, 's-', label="t_on")
    plt.plot(x, t_sub_find_pub-t_start, 's-', label="t_findpub")
    plt.plot(x, t_pub_find_sub-t_start, 's-', label="t_discovered")
    plt.plot(x, t_first_sent-t_start, 's-', label="t_sent")
    plt.plot(x, t_first_received-t_start, 's-', label="t_received")
    plt.xlabel("Experiment")
    plt.ylabel("delay(ms)")
    plt.axis([1, len(x), 0, 8000])
    plt.title("{} discovery delay under {} network".format(dds, net))
    plt.legend()
    plt.show()
    plt.savefig("./{}/results/{}.png".format(dds, net))

def main():
    for dds in args.dds:
        result_dir = os.path.join(dds, "results")
        num = 100
        pub_results = ["{}_pub_{}_{:d}.txt".format(dds, args.net, x) for x in range(num)]
        sub_results = ["{}_sub_{}_{:d}.txt".format(dds, args.net, x) for x in range(num)]
        net_results = ["{}_{}.txt".format(args.net, x) for x in range(num)]

        t_start = np.zeros(num, dtype=int)
        t_ip = np.zeros(num, dtype=int)
        t_pub_find_sub = np.zeros(num, dtype=int)
        t_sub_find_pub = np.zeros(num, dtype=int)
        t_first_sent = np.zeros(num, dtype=int)
        t_first_received = np.zeros(num, dtype=int)
	
        for i in range(num):
            with open(os.path.join(result_dir, net_results[i]), 'r') as f:
                t_start[i], t_ip[i] = extract_time_from_net(f)
            with open(os.path.join(result_dir, sub_results[i]), 'r') as f:
                t_sub_find_pub[i], t_first_received[i] = extract_time_from_sub(f, dds)
            with open(os.path.join(result_dir, pub_results[i]), 'r') as f:
                t_pub_find_sub[i], t_first_sent[i] = extract_time_from_pub(f, dds)
        print(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent)
        plot_figure(np.array(t_start), np.array(t_ip), np.array(t_pub_find_sub), np.array(t_sub_find_pub), np.array(t_first_received), np.array(t_first_sent), dds, args.net)

if __name__ == "__main__":
    main()

        
