## Navodila za namestitev Ubuntu 18.04 Server + ROS Melodic na RaspberryPi 4 

- Headless uporaba RPI 
- Povezava preko SSH 
- SSH poveza z Visual Studio Code 

 

### Namestitev Ubuntu 18.04 Server 

Uporabi Disc Management in formatiraj 16 Gb SD kartico 

Snemi [ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz](https://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img.xz)

Extrahiraj sliko (7zip) 

Sledi [navodilom za namestitev](https://help.ubuntu.com/community/Installation/FromImgFiles).

Na RaspberryPi priklopi ekran, miško in tipkovnico, se prijavi v ubuntu sistem

```python
username: ubuntu
password: ubuntu
```
in spremeni geslo.

Sledi [navodilom za postavitev mreže in SSH](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet).


Spremeni geslo: 

```python
sudo passwd ubuntu 

```

### Namestitev ROS 

Poveži se preko SSH 

Sledi [navodilom](http://wiki.ros.org/melodic/Installation/Ubuntu), namestiš ROS-Base (ker je Ubuntu server, nima smisla nameščati grafična orodja)

Desni miškin klik je “paste” v cmd. 

Ko namestis, naredi `catkin_ws` 

```
cd
mkdir catkin_ws 
cd catkin_ws 
mkdir src 
cd .. 
catkin_make 
```

Dodaj še source za ROS spremenljivke 

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc 
```

### Nastavitev statičnega IP 

[Vir](https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-18-04/)

S spodnjim ukazom dobi seznam vseh mrežnih naprav:
```
ifconfig
```
Poišči ustrezno mrežno kartico (po navadi je `eth0`).

Postavi se v `/etc/netplan`
```
cd /etc/netplan
```
in odpri `01-netcfg.yaml` (če je ni, preveri z `ll`)

```
sudo nano /etc/netplan/01-netcfg.yaml
```

Dodaj spodnjo kodo, kjer nastaviš ustrezne mrežne nastavitve. Pozorni bodite na ustrezne zamike (uporabite presledke ne tabulator).

```python linenums="1" hl_lines="5 6 7 8 9"
network: 
   ethernets: 
      eth0: 
         dhcp4: false 
         addresses: 
            - 192.168.65.60/24 
         gateway4: 192.168.65.254 
         nameservers: 
            addresses: [192.168.65.14, 193.2.1.66] 
   version: 2 
```

Na koncu ponovno zaženi netplan
```
sudo netplan apply
```
ter preveri, če so nastavitve pravilne z
```
ifconfig
```

### Povezava s SSH 

V Win CMD se povežeš s:  
```
ssh RPI_uporabnisko_ime@RPI_IP 
```
 

### Povezava z VS Code 

Sledi [navodilom](https://www.raspberrypi.org/blog/coding-on-raspberry-pi-remotely-with-visual-studio-code/)

Pozor! Potrebuješ delujočo mrežno povezavo!

## GPIO na RaspberryPi

[rpi.gpio](https://sourceforge.net/projects/raspberry-gpio-python/)


Namestitev:

```
sudo apt-get update
sudo apt-get install python-rpi.gpio
```

Dodaj trenutnega uporabnika v skupino `dialout`:
```
sudo adduser $USER dialout
```
in ponovno zaženi RPi.

Uporaba:

```python linenums="1"
import rospy
import RPi.GPIO as GPIO

# button GPIO
    # button 1 - GPIO 11
    # button 2 - GPIO 12

# LED GPIO
    # Green 1 - GPIO 2
    # Green 2 - GPIO 3
    # Yellow 1 - GPIO 4
    # Yellow 2 - GPIO 5
    # Red 1 - GPIO 6
    # Red 2 - GPIO 7


def resetLed():
     # nastavi in resetiraj vse LED
    for ii in range(2,8):
        # nastavi IO kot izhode
        GPIO.setup(ii,GPIO.OUT)
        # postavi izhode na nizek nivo
        GPIO.output(ii,False)

if __name__ == '__main__':
    # node init
    rospy.init_node('test_gpio_rpi')
    # set GPIO as BCM
    GPIO.setmode(GPIO.BCM)
    # reset LED
    resetLed()
    # set button1 as input
    GPIO.setup(11, GPIO.IN)
    # set loop to 10 Hz
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # read GPIO
        gpio_state = GPIO.input(11)
        """
        interrupt definition
        GPIO.add_event_detect(gpio_num, detected_edge, callback, bouncetime)
        GPIO.add_event_detect(BUTTON_GPIO, GPIO.RISING, callback=button_callback, bouncetime=500)
        """
        GPIO.output(2, False)
        if gpio_state:
            GPIO.output(2, True)
        rate.sleep()

    # clean GPIO settings after stop
    GPIO.cleanup()
```

## SICK NanoScan3

Manual - [wiki](http://wiki.ros.org/sick_safetyscanners)

- 1651 measurements
- angle resolution: 0.002909 rad
- scan angle: 275°

### Install ROS support

```
sudo apt-get install ros-melodic-sick-safetyscanners
```

### Run the driver
```
roslaunch sick_safetyscanners sick_safetyscanners.launch sensor_ip:=<sensor ip> host_ip:=<host ip>
```
### Published topics

Topic: `/sick_safetyscanners/scan`

Message type: `sensor_msgs/LaserScan`

## Linux ukazi

Uporabni ukazi v Linux okolju

Dostop to root direktorija:
```bash
cd /
```
Premik v prejšnjo mapo:
```
cd ..
```
Premik v domačo mapo:
```
cd /home/$user$
cd ~
```
Izpis trenutne poti:
```
pwd
```
Prikaži vsebino mape:
```
ls
```
Prikaži tudi skrito vsebino mape:
```
ls -l
```
Ustvari novo mapo `my_folder`:
```
mkdir my_folder
```
Izbriši prazno mapo `my_folder`:
```
rmdir my_folder
```
Izbriši mapo `my_folder`, ki ni prazna:
```
rm -rf my_folder
```
Spremeni pravice datoteke `my_file` v executable:
```
chmod +x my_file
```
Ustvari novo datoteko `my_file.py`
```
touch my_file.py
```
Odpri datoteko `my_file.py` v Visual Studio Code:
```
code my_file.py
```
Odpri datoteko `my_file.py` v konzolnem urejevalniku besedila `nano`:
```
nano my_file.py
```







