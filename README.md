# Robotic arm simulation: Blocks word problem
• Plan motions/movements for the Puma robotic arm to move blocks to stack them in the desired order using python and openRave
• Implemented kinematic, inverse kinematics, biRRT, and data structure to find a set of collision-free movements of robotic arm
• Enabled the algorithm to find solution for any number of input blocks


cs5335_puma.py contains a code to simulate the program. <br>
<b>Command to run the file:</b> python cs5335_puma.py <outputStackOrder> <environmentFile><br>
	<b>eg.:</b> python cs5335_puma.py RGBAW cs5335_53.env.xml <br>
		    

All the xml files include environment details for openrave.<br>
File names are given as cs5335_\<NumberOfBlocks\>\<config\>.env.xml <br>
<br>NumberOfBlocks can be 3,4 or 5<br>
	<b>Config</b> : <br>	
	1 : all the blocks are stacked in one column <br>
	2 : all the blocks are placed at different locations <br>
	3 : blocks are stacked in two columns<br>
	
<b>analysis1.py analysis2.py</b> and <b>analysis3.py</b> runs the code for each configuration multiple times and returns the time taken to simulate the result in an csv file.<br>
