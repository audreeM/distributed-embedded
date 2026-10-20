# Lab 3 Pi CAN bring-up (Part 0/1 checkpoint)

```bash
# from the laptop
scp -r lab3_pi/bringup pi@<pi-ip>:~/
ssh pi@<pi-ip>

# on the Pi, once
cd ~/bringup && chmod +x *.sh
sudo ./setup_can.sh 16000000      # crystal frequency printed on the module
sudo reboot

# on the Pi, after every reboot
cd ~/bringup && ./can_up.sh
candump -t d can0                 # terminal 1: bus A
candump -t d can1                 # terminal 2: bus B
./send_test.sh                    # optional: Pi -> STM direction
```
