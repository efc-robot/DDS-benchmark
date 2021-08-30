import os
import argparse
import subprocess
import time

parser = argparse.ArgumentParser(description="DDS benchmark")
parser.add_argument('--num', type=int, default=1, help="number of experiments, default:100")
parser.add_argument('--index', type=int, default=0, help="index of experiments, default:0")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds)")
parser.add_argument('--net', required=True, choices=['adhoc', 'wired', 'wireless'], help="choose the way of networking")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--sub', action='store_true', help="serve as a subscriber")
group.add_argument('--pub', action='store_true', help="serve as a publisher")

args = parser.parse_args()


def test_sub(dds, net, index):
    exe_dir = os.path.join(dds, "workspace/build/DDSHelloWorldSubscriber")
    output_file = "{}_sub_{}_{}.txt".format(dds, net, index)
    output_dir = os.path.join(dds, "results")
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)

    # first, run subscriber
    subscriber = subprocess.Popen(os.path.join(".", exe_dir), stdout=outputfile)
    time.sleep(5)
    
    # second, run network script
    network = subprocess.Popen(['/bin/bash', "./script/{}.sh".format(net), "fastdds", "{:d}".format(index)], stdout=subprocess.PIPE)
    network.wait()
    # wait for publisher
    time.sleep(20)

    # kill subprocess
    if subprocess.Popen.poll(subscriber) == None:
        subprocess.Popen.terminate(subscriber)
    outputfile.close()

def test_pub(dds, net, index):
    exe_dir = os.path.join(dds, "workspace/build/DDSHelloWorldPublisher")
    output_file = "{}_pub_{}_{}.txt".format(dds, net, index)
    output_dir = os.path.join(dds, "results")
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)

    # first, run publisher
    publisher = subprocess.Popen(os.path.join(".", exe_dir), stdout=outputfile)

    # wait for publisher
    time.sleep(30)

    # kill subprocess
    if subprocess.Popen.poll(publisher) == None:
        subprocess.Popen.terminate(publisher)
    outputfile.close()

def main():
    for dds in args.dds:
        #for i in range(args.num):
        if args.pub:
            test_pub(dds, args.net, args.index)
        elif args.sub:
            test_sub(dds, args.net, args.index)



if __name__ == "__main__":
    main()
