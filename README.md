# MTI Panda Controllers

This package includes different controllers, which are needed to run PhastaProMPs. 

## Calibrate Friction Parameters

1. get the mac address of the robot you want to calibrate. 

```bash
arp -a panda  | awk '{print $4}' | sed s/://g | tr -d '\n'
```

2. Go to /controller/config/ and open corresponding config file

3. Run controller 

```bash
roslaunch controller_panda_mti test_pdcontroller.launch
```

4. Adjust torque Bias

    a) Check whether there is constant movement in any joint.
    b) Adjust corresponding joint value higher or lower opposing motion.
    c) Reapeat until robot does not move at all. 

5. Adjust torque_stiction

    a) Move every joint by hand and check whether there is to much frictional force opposing the motion.
    b) Increase or decrease corresponding joint value
    c) Repeat until every joint is easy to move by hand. 