visual studio 2019
Visual Studio Installer tool -> add C++ for desktop development

click generateProject.bat
open the sln
run. it will fail with no dll.
run postBuildCommands.bat
run the sln again, now it should work.
*after any deletion of bin, run postBuildCommands

important: the file \Roomie3D\kinect_cpp_preprocessing\bin\Debug-windows-x86_64\kinect_cpp_preprocessing\opencv_world452d.dll
is missing, as it's too big too upload regularly into git. You can find it in here:
https://opencv.org/releases/