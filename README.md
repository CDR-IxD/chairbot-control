# chairbot-control
Using OpenCV to navigate chairbots.

Credits to [Abhijeet Agnihotri](https://github.com/abhiagni11) for developing v1.0 for the OpenCV code.

## Setup
Instructions by [Abhijeet Agnihotri](https://github.com/abhiagni11)

Set up computer:

I am working in linux, ubuntu 16.04 .

The code listed here is for opencv 3.1 Install opencv 3.1 by following the steps listed here-: http://rodrigoberriel.com/2014/10/installing-opencv-3-0-0-on-ubuntu-14-04/

After installing opencv, try to run basic opencv applications to check the installation.

I would recommend installing libraries via "Synaptic package manager", you can install individual libraries as well but having a package manager will help you a lot in making sure you do not miss any dependencies.

After opening synaptic, make sure you install all these libs: (also install the other libraries prompted in the synaptics manager when installing one of the below mentioned libraries.)

a) libwebsocketpp

b) libboost

c) libthread

d) libasio

e) libserial

These should work, if while compiling you get some errors , it could be solved by installing these libs. Otherwise look into all the libraries mentioned in CMakeLists.txt in the computer vision program folders in the repositories.

Raspberry pi is already set up with all the libraries and dependencies installed.
