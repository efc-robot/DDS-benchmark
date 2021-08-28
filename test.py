import os
import argparse
import subprocess
import time

parser = argparse.ArgumentParser(description="DDS benchmark")
parser.add_argument('--num', type=int, default=100, help="number of experiments, default:100")
parser.add_argument('--dds', required=True, choices=['fastdds', 'opensplice', 'connext', 'opendds'], nargs='+', 
    help="choose one or more DDS for experiments(choices:fastdds, opensplice, connext, opendds)")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--sub', action='store_true', help="serve as a subscriber")
group.add_argument('--pub', action='store_true', help="serve as a publisher")

args = parser.parse_args()


def test_sub(dds, index):
    exe_dir = os.path.join(dds, "workspace/build/DDSHelloWorldSubscriber")
    output_file = "{}_sub_{}.txt".format(dds, index)
    output_dir = os.path.join(dds, "results")
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)

    # first, run subscriber
    subscriber = subprocess.Popen(os.path.join(".", exe_dir), stdout=outputfile)

    # second, run network script
    subprocess.run(['/bin/bash', "./script/network.sh"], stdout=subprocess.PIPE)

    # wait for publisher
    time.sleep(20)

    # kill subprocess
    if subprocess.Popen.poll(subscriber) == None:
        subscriber.terminate()
    outputfile.close()

def test_pub(dds, index):
    exe_dir = os.path.join(dds, "workspace/build/DDSHelloWorldPublisher")
    output_file = "{}_pub_{}.txt".format(dds, index)
    output_dir = os.path.join(dds, "results")
    outputfile = open(os.path.join(output_dir, output_file), 'w')
    if outputfile == None:
        print("open output file error!")
        exit(0)

    # first, run publisher
    publisher = subprocess.run(os.path.join(".", exe_dir), stdout=outputfile)

    # wait for publisher
    time.sleep(20)

    # kill subprocess
    if subprocess.Popen.poll(publisher) == None:
        publisher.terminate()
    outputfile.close()

def main():
    for dds in args.dds:
        for i in range(args.num):
            if args.pub:
                test_pub(dds, i)
            elif args.sub:
                test_sub(dds, i)



if __name__ == "__main__":
    main()
