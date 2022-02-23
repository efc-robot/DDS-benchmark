import os
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--dds", required=True, nargs='+', choices=["fastrtps", "cyclone"], help="choose one or more dds for test.")
parser.add_argument("--net", required=True, choices=["adhoc", "wireless", "wired"], help="choose the way of networking")
parser.add_argument("--size", default="64KB", nargs='+', help="choose the size of msg")
parser.add_argument("--verbose", action="store_true", help="print all results")
parser.add_argument("--plot_origin", action="store_true", help="plot original statistics")
parser.add_argument("--plot_results", action="store_true", help="plot final results")
args = parser.parse_args()

def extract_from(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    pub_dict = {}
    sub_dict = {}
    for line in lines:
        data = line.split()
        if data[0] == "[INFO]" and data[3] == "Publishing:":
            pub_dict[int(data[4][1:-1])] = float(data[1][1:-1])

        elif data[0] == "[INFO]" and data[4] == "heard:":
            sub_dict[int(data[5][1:-1])] = float(data[1][1:-1])
    return pub_dict, sub_dict
    
def matched_latency(pub_dict, sub_dict):
    matched_num = 0
    index = np.zeros(len(sub_dict), dtype=int)
    latency = np.zeros(len(sub_dict), dtype=float)
    for i, key in enumerate(sub_dict.keys()):
        if key in pub_dict:
            index[matched_num] = key
            latency[matched_num] = sub_dict[key] - pub_dict[key]
            matched_num = matched_num + 1
    return index[:matched_num], latency[:matched_num]*1000
    
def plot_origin(index, latency, net, dds, size):
    filename = os.path.join(net, dds, "{}.png".format(size))
    plt.figure()
    plt.scatter(index, latency)
    #plt.axis([0, 2000, 0, 200])
    plt.title("{} {} {}".format(net, dds, size))
    plt.savefig(filename)
    print("Save figure {}".format(filename))
    
def plot_results(results, net):
    plt.figure()
    for i, dds in enumerate(results.keys()):
        fmt = ['co-', 'bs-', '^r-']
        plt.errorbar(x=range(1,len(results[dds].keys())+1), y=[results[dds][s]["mean"] for s in results[dds].keys()], yerr=[results[dds][s]["std"] for s in results[dds].keys()], fmt=fmt[i], label=dds)
    plt.legend()
    plt.show()
    
def main():
    results = {}
    for dds in args.dds:
        result_dir = os.path.join(args.net, dds)
        results[dds] = {}
        for size in args.size:
            log_file = "{}.txt".format(size)
            log_path = os.path.join(result_dir, log_file)
            # pub_data = extract_from(os.path.join(result_dir, pub_filename))
            pub_time, sub_time = extract_from(log_path)
            index, latency = matched_latency(pub_time, sub_time)
            if(args.plot_origin):
                plot_origin(index, latency, args.net, dds, size)
            results[dds][size] = {"mean": np.mean(latency), "std": np.std(latency), "median":np.median(latency)}
    if args.verbose:
        print("\t\tmean\t\tmedian\t\tstd")
        print("--------------------------------------")
        for dds in results.keys():
            print(dds)
            for size in results[dds].keys():
                print("\t{}\t{:03f}\t{:03f}\t{:03f}".format(size, results[dds][size]["mean"], results[dds][size]["median"], results[dds][size]["std"]))
    if args.plot_results:
        plot_results(results, args.net)
            
    
    
if __name__ == "__main__":
    main()
