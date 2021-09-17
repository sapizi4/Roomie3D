This is the Roomie3D registration module, or the kinect jpg to 3d model module.
Usage:
After the use of the kinect to jpg module ("kinect cpp preprocessing") you should have a folder with a 2 folders inside:
one with color images and one with deapth images, both in jpg format.
Open slam.py, and edit the paths in the function lounge_register_2 to match yours.
Make sure the filenames of the jpg's match, and save.

now in cmd:
python slam.py

The registration process should take about an hour, depending on your computer's abilities.
Then tweak the temporary ply result with the tools in the openPointCloud.py, to adjust the final paramameters.
Especially notice the "density" parameter, try to move it down for more details and up for less blobs.
In the end you will have a 3d obj file of your room, ready to enter the final unity module.