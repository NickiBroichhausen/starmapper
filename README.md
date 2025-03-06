# StarSift

This repo contains the code to run a star mapping mission on a raspberry Pi for our floatsat mission. For this code to run on the floatsat, following repo is also needed: https://github.com/reshab12/FloatSat.git

## Content

The file `main_rxtx.py` is supposed to be run on the rasbperry pi and connect with the stm board. The file `main.py` can be run localy for testing. 


### PI:

Download an image:
<!-- ```
pscp floatsat-2@192.168.91.92:/home/floatsat-2/starmapper/star_data/image_0.jpg image.jpg
``` -->
```
pscp pi@raspberrypi.local:/home/pi/test.jpg test.jpg
```

### see autostart logs on pi
```
sudo journalctl -u my_script.service -f
```
edit autostart:
```
sudo vi /etc/systemd/system/my_script.service

```

### Xterm bug

```
Xvfb :1 -screen 0 800x600x16 & 
export DISPLAY=:1
```

### Allow serial 
```
sudo usermod -aG dialout $USER
```

### get picam
```
sudo apt update
sudo apt install python3-picamzero
```

