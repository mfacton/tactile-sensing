# Universal Robotics CB3 UR5 simulator

## Setup
```bash
# One time setup
sudo docker network create --subnet=192.168.56.0/24 ursim_net
```

## Run
```bash
# Run docker command in this folder
# Robot IP is bound to --ip 192.168.56.101
sudo docker run --rm -it -p 30004:30004 -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 -v ./urcaps:/urcaps -v ./programs:/ursim/programs --name ursim universalrobots/ursim_cb3
```

## Safety Settings
Password for safety settings is 'password'
