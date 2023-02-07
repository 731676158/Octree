## if before ubuntu 18.04:

### dependencies
```
sudo apt-get install libsdl-image1.2 libsdl-image1.2-dev guile-1.8 \
guile-1.8-dev libsdl1.2debian libart-2.0-dev libaudiofile-dev \
libesd0-dev libdirectfb-dev libdirectfb-extra libfreetype6-dev \
libxext-dev x11proto-xext-dev libfreetype6 libaa1 libaa1-dev \
libslang2-dev libasound2 libasound2-dev build-essential
```
## if after ubuntu 18.04
guile 1.8 and libesd0-dev is cancelled,so
```
sudo gedit /etc/apt/sources.list 
```
add contents below
```
deb http://us.archive.ubuntu.com/ubuntu/ xenial main universe deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main universe 
```
then run
```
sudo apt-get install libsdl-image1.2 libsdl-image1.2-dev guile-2.0 guile-2.0-dev libsdl1.2debian libart-2.0-dev libaudiofile-dev libesd0-dev libdirectfb-dev libdirectfb-extra libfreetype6-dev libxext-dev x11proto-xext-dev libfreetype6 libaa1 libaa1-dev libslang2-dev libasound2 libasound2-dev guile-1.8 guile-1.8-dev
```
## compile and installation
```
./configure
make
sudo make install

```

### error occurs!
if under ubuntu, error with conio.h will occur, execute codes as follows
```
sudo cp conio.h /usr/include/
```
