import os
import argparse
import numpy as np
import time
from matplotlib import pyplot as plt
parser = argparse.ArgumentParser(description="deal with dds output and calculate time")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds)")
parser.add_argument('--net', required=True, choices=['adhoc', 'wired', 'wireless'], help="choose the way of networking")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--sub', action='store_true', help="print subscriber and net results")
group.add_argument('--pub', action='store_true', help="print publisher results")
group.add_argument('--all', action='store_true', help="print all results")

args = parser.parse_args()

def plot_figure(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent, dds, net):
    plt.figure()
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
    #plt.show()
    plt.savefig("./{}/results/{}.png".format(dds, net))

def extract_time_from_net(files):
    t_start = np.zeros(len(files), dtype=int)
    t_ip = np.zeros(len(files), dtype=int)
    for i, file in enumerate(files):
        with open(file, 'r') as f:
            lines = f.readlines()
        if len(lines) < 2:
            print("net file {} error!".format(i))
            exit()
        t_start[i], t_ip[i] = int(float(lines[0])*1000), int(float(lines[1])*1000)
    return t_start, t_ip


def extract_time_from_sub(files, dds):
    t_sub_find_pub = np.zeros(len(files), dtype=int)
    t_first_received = np.zeros(len(files), dtype=int)
    if dds == "fastdds":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "RTPS_PDP_DISCOVERY" in line:
                    item = line.split()
                    #print(item)
                    t_sub_find_pub[i] = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                if "HelloWorld: Change" in line:
                    #print(item)
                    item = line.split()
                    t_first_received[i] = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                    break
        return t_sub_find_pub, t_first_received
    if dds == "opendds":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "Spdp::handle_participant_data" in line:
                    item = line.split()
                    #print(item)
                    t_sub_find_pub[i] = int(time.mktime(time.strptime("{} {}".format(item[0], item[1][:8]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][9:12])
                if "DataReaderImpl::data_received: reader" in line:
                    #print(item)
                    item = line.split()
                    t_first_received[i] = int(time.mktime(time.strptime("{} {}".format(item[0], item[1][:8]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][9:12])
                    break
        return t_sub_find_pub, t_first_received
    if dds == "cyclone":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "dq.builtin: SPDP ST0" in line and "192.168.50" in line:
                    item = line.split()
                    t_sub_find_pub[i] = int(float(item[0]*1000))
                if "Msg:{1,\"Hello World\"}" in line:
                    item = line.split()
                    t_first_received[i] = int(float(item[0]*1000))
        return t_sub_find_pub, t_first_received
    if dds == "opensplice":
        pass

def extract_time_from_pub(files, dds):
    t_pub_find_sub = np.zeros(len(files), dtype=int)
    t_first_sent = np.zeros(len(files), dtype=int)

    if dds == "fastdds":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "RTPS_PDP_DISCOVERY" in line:
                    item = line.split()
                    #print(item)
                    t_pub_find_sub[i] = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                if "RTPS_HISTORY" in line:
                    #print(item)
                    item = line.split()
                    t_first_sent[i] = int(time.mktime(time.strptime("{} {}".format(item[0][7:], item[1][:-4]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][-3:])
                    break
        return t_pub_find_sub, t_first_sent

    if dds == "opendds":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "Spdp::handle_participant_data" in line:
                    item = line.split()
                    #print(item)
                    t_pub_find_sub[i] = int(time.mktime(time.strptime("{} {}".format(item[0], item[1][:8]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][9:12])
                if "DataWriterImpl::create_sample_data_message" in line:
                    #print(item)
                    item = line.split()
                    t_first_sent[i] = int(time.mktime(time.strptime("{} {}".format(item[0], item[1][:8]), '%Y-%m-%d %H:%M:%S')))*1000 + int(item[1][9:12])
                    break
        return t_pub_find_sub, t_first_sent
    if dds == "cyclone":
        for i, file in enumerate(files):
            with open(file, 'r') as f:
                lines = f.readlines()
            for line in lines:
                if "dq.builtin: SPDP ST0" in line and "192.168.50" in line:
                    item = line.split()
                    t_pub_find_sub[i] = int(float(item[0]*1000))
                if "Msg:{1,\"Hello World\"}" in line:
                    item = line.split()
                    t_first_sent[i] = int(float(item[0]*1000))
        return t_pub_find_sub, t_first_sent

def main():
    results = {}
    for dds in args.dds:
        result_dir = os.path.join(dds, "results", args.net)
        if not os.path.exists(result_dir):
            print("creating result directory: {}".format(result_dir))
            os.makedirs(result_dir)
        files = os.listdir(result_dir)

        pub_results = [os.path.join(result_dir, file) for file in files if "pub" in file]
        sub_results = [os.path.join(result_dir, file) for file in files if "sub" in file]
        net_results = [os.path.join(result_dir, file) for file in files if file.startswith(args.net)]
        print(len(net_results), len(pub_results), len(sub_results))
        
        pub_results.sort()
        sub_results.sort()
        net_results.sort()

        if args.pub:
            if len(pub_results) == 0:
                print('can not find pub results')
                exit(0)
            t_pub_find_sub, t_first_sent = extract_time_from_pub(pub_results, dds)
            print(t_pub_find_sub, t_first_sent)

        if args.sub:
            if len(sub_results) == 0:
                print('can not find sub results')
                exit(0)
            if len(net_results) == 0:
                print('can not find net results')
                exit(0)
            t_start, t_ip = extract_time_from_net(net_results)
            t_sub_find_pub, t_first_received = extract_time_from_sub(sub_results, dds)
            print(t_start[:5], t_ip[:5], t_sub_find_pub[:5], t_first_received[:5])

        if args.all:
            t_start, t_ip = extract_time_from_net(net_results)
            t_sub_find_pub, t_first_received = extract_time_from_sub(sub_results, dds)
            t_pub_find_sub, t_first_sent = extract_time_from_pub(pub_results, dds)
            latency_discovery = np.maximum(t_sub_find_pub,t_pub_find_sub) - t_ip
            latency_received = t_first_received - t_ip
            latency_discovery_final = np.delete(latency_discovery, np.where(latency_discovery > 10000))
            latency_received_final = np.delete(latency_received, np.where(latency_received > 10000))
            results[dds] = {"discovery":{"mean":np.mean(latency_discovery_final), "std":np.std(latency_discovery_final)}, "sent":{"mean":np.mean(latency_received_final), "std":np.std(latency_received_final)}}
            #print(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent)
            plot_figure(t_start, t_ip, t_pub_find_sub, t_sub_find_pub, t_first_received, t_first_sent, dds, args.net)
    print(results)
if __name__ == "__main__":
    main()

        
