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

Setup For Windows:
you will need to build it because it isnt finisched yet (missing gui only testing)
- Install bullet physics with vcpackage (https://github.com/microsoft/vcpkg , run the command ./vcpkg install bullet3:x64-windows in the directory of vcpkg e.g "mystuff/vcpkg/")
- copy the bullet phyiscs header directory into the current directory where you want to build the addon ("BlenderBulletPhysicsAddon
/") [the directory of vcpkg e.g "mystuff\vcpkg"]\vcpkg\installed\x64-windows\include\bullet)
  (other wise bullet physics wont find the headers and other problems)
  you may also need to copy CommonFileIOInterface.h into the current directory
- Copy the pybinds directory into the current directory (https://github.com/pybind/pybind11)
- install CMake (https://cmake.org/)

-create new folder "build"
-run the CMake GUI to build the Visual studio project in the build Folder (select the current directory for where is the source code, and where to build the binaries is the build folder) then click Generate.

-open the Visual Studio project in the build Folder
-and build phys_view for the bullet-physics Demo
-or mybinds to build the python library

General IDEA of this Project:
Use the Bullet Physics library Functions For simulating softbodies in Blender, by creating python binds with pybinds11 to create a python library that binds to cpp code that can be put inside an Addon And run efficiently because the functions are not converted to python code just controlled by it.

