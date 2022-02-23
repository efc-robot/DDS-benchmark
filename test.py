import os
import argparse
import subprocess
import time

parser = argparse.ArgumentParser(description="DDS benchmark")
parser.add_argument('--num', type=int, default=1, help="number of experiments, default:100")
parser.add_argument('--index', type=int, default=0, help="index of experiments, default:0")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds', 'cyclone'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds, cyclone)")
parser.add_argument('--net', required=True, choices=['adhoc', 'wired', 'wireless'], help="choose the way of networking")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--sub', action='store_true', help="serve as a subscriber")
group.add_argument('--pub', action='store_true', help="serve as a publisher")

args = parser.parse_args()


def test_sub(dds, net, index):
    exe_dir = os.path.join(dds, "workspace/build/subscriber")
    output_file = "sub_{:04d}.txt".format(index)
    output_dir = os.path.join(dds, "results", net)
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)
    if dds == "fastdds" or "cyclone":
        cmd = os.path.join(".", exe_dir)
        # first, run subscriber
        subscriber = subprocess.Popen(cmd, stdout=outputfile, stderr=subprocess.STDOUT)
        time.sleep(5)
    
        # second, run network script
        network = subprocess.Popen(['/bin/bash', "./script/{}.sh".format(net), dds, "{:04d}".format(index)], stdout=subprocess.PIPE)
        network.wait()

        # wait for subscriber
        subscriber.wait()
    
    if dds == "opendds":
        cmd = [os.path.join(".", exe_dir), '-DCPSConfigFile', 'opendds/workspace/build/rtps.ini', '-DCPSDebugLevel', '8']
        
        # first, run network script
        network = subprocess.Popen(['/bin/bash', "./script/{}.sh".format(net), dds, "{:04d}".format(index)], stdout=subprocess.PIPE)
        time.sleep(5)
        
        # second, run subscriber
        subscriber = subprocess.Popen(cmd, stdout=outputfile, stderr=subprocess.STDOUT)
        
        network.wait()
        
        subscriber.wait()

def test_pub(dds, net, index):
    exe_dir = os.path.join(dds, "workspace/build/publisher")
    output_file = "pub_{:04d}.txt".format(index)
    output_dir = os.path.join(dds, "results", net)
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)

    if dds == "fastdds" or "cyclone":
        cmd = os.path.join(".", exe_dir)
    if dds == "opendds":
        cmd = [os.path.join(".", exe_dir), '-DCPSConfigFile', 'opendds/workspace/build/rtps.ini', '-DCPSDebugLevel', '4']

    # first, run publisher
    publisher = subprocess.Popen(cmd, stdout=outputfile, stderr=subprocess.STDOUT)

    # wait for publisher
    publisher.wait()
    # time.sleep(30)

    # # kill subprocess
    # if subprocess.Popen.poll(publisher) == None:
    #     subprocess.Popen.terminate(publisher)
    # outputfile.close()

def main():
    for dds in args.dds:
        #for i in range(args.num):
        if args.pub:
            test_pub(dds, args.net, args.index)
        elif args.sub:
            test_sub(dds, args.net, args.index)



if __name__ == "__main__":
    main()
