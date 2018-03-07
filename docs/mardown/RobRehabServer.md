**RobRehabServer** is the application responsible for [[communicating|Clients Protocol]] with external system's clients. 

It doesn't process robots control itself, but exchanges online/real-time data with [[RobRehabControl|RobRehabControl]] application and exposes this information to the remote clients.

<p align="center">
  <img src="https://raw.githubusercontent.com/Bitiquinho/RobRehabSystem/master/img/client_communication.png" width="800"/>
</p>
<p align="center">
  (Communication channels between RobRehabServer and its clients)
</p>

### Command-line arguments

For running it, use the follwing command from project's root folder:

    $ ./RobRehabServer <multicast_ip_address>
    
Where **ip_address** is the local IP address that can be used for [multicast](https://en.wikipedia.org/wiki/Multicast) (UDP) connections inside local networks (**LAN**s)