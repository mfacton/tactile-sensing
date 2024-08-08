from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("192.168.56.101")
rtde_c.moveL([-0.143, 0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)