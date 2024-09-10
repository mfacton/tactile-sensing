sudo docker run --rm -it -p 30004:30004 -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 -v ./urcaps:/urcaps -v ./programs:/ursim/programs --name ursim universalrobots/ursim_cb3
