# BlenderBulletPhysicsAddon
An Addon that lets you use the ABSOLUTLY INCREDIBLE SOFTBODY AND rigidbody Physics from the bullet physics Library IN BLENDER.
Because as it turns out these can actually RUN CONSIDERABLY FAST 100fps!!! if they aren't coded 
in Python and Blender already uses Bullet Physics for other components so it is questionable why they did not use the 
faster and better bullet physics version.

Requirements: 
-Visual Studio 2017 or higher
-Bullet Physics (Install this preferably with vcpackage)
-CMake
-PyBinds (Just Copy the folder from the github into this directory e.g BlenderBulletPhysicsAddon/PyBinds)

Setup For Building On Windows x64:
you will need to build it because it isnt finisched yet (missing gui only testing)
- Install bullet physics with vcpackage (https://github.com/microsoft/vcpkg , run the command ./vcpkg install bullet3:x64-windows in the directory of vcpkg e.g "mystuff/vcpkg/")
- copy the bullet phyiscs header directory into the current directory where you want to build the addon ("BlenderBulletPhysicsAddon
/") [the directory of vcpkg e.g "mystuff\vcpkg"]\vcpkg\installed\x64-windows\include\bullet)
  (other wise bullet physics wont find the headers and other problems)
  you may also need to copy CommonFileIOInterface.h into the current directory
  (get the library working post errors, questions here or at the bullet library page)
- Copy the pybinds directory into the current directory (https://github.com/pybind/pybind11)
- install CMake (https://cmake.org/)

-create new folder "build"
-run the CMake GUI to build the Visual studio project in the build Folder (select the current directory for where is the source code, and where to build the binaries is the build folder) then click Generate.

-open the Visual Studio project in the build Folder
-and build phys_view for the bullet-physics Demo
-or build mybinds to build the python library

put the Addon_better.py in a zip file with the library you built which can be found in the Debug/Debug folder or the release folders possibly called mybinds.cp310-win_amd64.pyd

And that's it the Zip File is your Addon.

Addon Installation 
In Blender 3.5 go to Edit/Preferences/Addons/Install select the zip file click install Addon.

General IDEA of this Project:
Use the Bullet Physics library Functions For simulating softbodies in Blender, by creating python binds with pybinds11 to create a python library that binds to cpp code that can be put inside an Addon and Run Efficently because its cpp code / And Faster Than the Current softbody system in blender.

# EASY STARTUP/ TEST INSTALLATION
A Simple setup you can try is:
-install the BulletAddonSimple.zip file in blender as an Addon
-download Workingtest.blend
-run the python script thats open

Press Space to Start And Stop the Simulation!

Warning my Crash Blender Allot.
