# Some notes of what I did

Sudo apt-get install
===============================
# Install apache server to create a local server to host the website
sudo apt-get install apache2
sudo a2enmod ssl
# Install rosbridge to have the capability to run the web socket server
ros-noetic-rosbridge-suite

# Setup a self-signed certificate to allow ssl communications between browser
# and robot.
cp /etc/ssl/private/ssl-cert-snakeoil.key /etc/ssl/certs/


==================================


pip install:
autobahn
pyOpenSSL
twisted
pymongo
