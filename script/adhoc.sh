output=$1/results/adhoc/adhoc_$2.txt
sudo ifconfig wlan1 down
#echo "waiting for publisher..."
#read -e -s
sleep 10s
sudo ifconfig wlan1 192.168.5.11
echo $(date +"%s.%N") > ${output}
while true; do
    if ping -c 1 192.168.5.10 | grep "0%" > /dev/null;then
	    echo $(date +"%s.%N") >> ${output}
	    break
    fi
done
