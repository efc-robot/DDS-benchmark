output=$1/results/network_$2.txt
sudo ifconfig enp4s0 down
#echo "waiting for publisher, press enter to continue"
#read -e -s
sleep 10s
sudo ifconfig enp4s0 up
echo $(date +"%s.%N") > ${output}
while true; do
    if ping -c 1 172.16.0.1 | grep "0%" > /dev/null;then
	    echo $(date +"%s.%N") >> ${output}
	    break
    fi
done
