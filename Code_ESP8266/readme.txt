CODE FOR ESP-8266

This is the code that runs on ESP 8266 chip.

The code mainly focusses on two separate areas on which the function of the bot relies:

Firstly, the establishment of connection between the bots with the help of a server. 

	The bots send a request to the server. 

	The server takes information about the coordinates of the bots with the help of image processing 
	and sends the data to the bots. 

	After receiving the data it reads it and prints them to the serial. 
	From here we extract the information of the bots position and now every bots has been loaded with 
	the position of every other bot. 

	Next, With the help of the mobile app a command is received to make a particular pattern. 
	The command signal loaded on a JSON format is received through the server.

	After this the code uploaded on the bot processes to find which bot should go to which position 
	without the intervention of the server. This part is done independently by the ESP 8266 chip on 
	the bot, which is the central concept in swarm robotics.


Now the second part of the code is to take the bots to the assigned position.

	It does this with the help of pwm. The required position is finally reached.


The code is self-explanatory and important comments have been added at appropriate position.
