# Robot-motion-planning

cs5335_puma.py contains a code to simulate the program.
Command to run the file: python cs5335_puma.py <outputStackOrder> <environmentFile>
		    eg.: python cs5335_puma.py RGBAW cs5335_53.env.xml
		    

All the xml files include environment details for openrave.
File names are given as cs5335_<NumberOfBlocks><config>.env.xml
NumberOfBlocks can be 3,4 or 5
Config : 
	1 : all the blocks are stacked in one column
	2 : all the blocks are placed at different locations
	4 : blocks are stacked in two columns
	
analysis1.py analysis2.py and analysis3.py runs the code for each configuration multiple times adn returns the time taken to simulate the result in an csv file.
