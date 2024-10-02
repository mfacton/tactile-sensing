# Universal Robotics CB3 UR5 simulator

## One Time Setup
```bash
sudo docker network create --subnet=192.168.56.0/24 ursim_net
```

## Run
Run docker command in this folder

Robot IP is bound to --ip 192.168.56.101

Robot control interface hosted [here](http://192.168.56.101:6080/vnc.html?host=192.168.56.101&port=6080)
```bash
sudo docker run --rm -it -p 30004:30004 -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 -v ./urcaps:/urcaps -v ./programs:/ursim/programs --name ursim universalrobots/ursim_cb3
```
or run bash launcher
```bash
./launch.sh
```

## Safety Settings
Password for safety settings is 'password'
