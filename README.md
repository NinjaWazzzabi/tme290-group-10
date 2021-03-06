# Starting Kiwi software using online images

Run the following command in the terminal:

	docker-compose -f rover-simulation.yml up

# Starting Kiwi software using local images

To start the simulation for testing on replayviewer, run the start script:

	./start_rover_sim.sh

To start the simulation for testing with the simulator, run the start script:

	./start_rover.sh

If that does not work you might need to set the execution bit:

	chmod +x start_sim.sh

# Updating the Open DLV message set

In the _messages_ folder you will find two files. One contains the standard  message set for _Open DLV_ and the other contains this project's custom message sets.

To add new messages edit the _custom-messages.odvd_. The message ID must be larger than 9000 to not intefere with the standard message IDs and of course must be unique in the custom message set.

To generate the header file and distribute it to all projects at once, run:

	./update-messages.sh

in the project root folder.