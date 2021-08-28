sudo ifconfig ens33 down
sleep 5s
sudo ifconfig ens33 up
echo $(date +"%s.%N")
while true; do
    if ping -c 1 192.168.246.1 | grep "0%" > /dev/null;then
	    echo $(date +"%s.%N")
	    break
    fi
done
