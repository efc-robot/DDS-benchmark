sudo ifconfig wlan0 down
sudo iwconfig wlan0 mode "ad-hoc"
sudo iwconfig wlan0 essid "MyAdhoc"
sudo iwconfig wlan0 key off
sudo iwconfig wlan0 channel 1
sudo ifconfig wlan0 192.168.5.11


