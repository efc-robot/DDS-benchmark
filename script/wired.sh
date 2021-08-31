output=$1/results/wired/wired_$2.txt
sudo ifconfig eth0 down
#echo "waiting for publisher..."
#read -e -s
sleep 10s
sudo ifconfig eth0 up
echo $(date +"%s.%N") > ${output}
while true; do
    if ping -c 1 192.168.50.1 | grep "0%" > /dev/null;then
	    echo $(date +"%s.%N") >> ${output}
	    break
    fi
done
