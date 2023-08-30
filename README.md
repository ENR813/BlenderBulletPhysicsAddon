# UNFINISHED PROJECT: NEEDS A GUI AND BUG/CRASH FIXES
# Blender Bullet Physics Addon
An addon that improves blenders soft body physics by letting you use the soft body and rigid body physics simulation functions from the bullet physics Library.
Because as it turns out, Soft body Simulations with very large vertex counts if implemented correctly can actually run considerably fast and stable!

# General Technical Idea/Plan of this Project:
Use the Bullet Physics library Functions For simulating soft bodies in Blender, by creating a python library with pybinds11 that binds to the Bullet physics library, which can be put inside a Blender addon.

# EASY STARTUP/ TEST INSTALLATION
A Simple setup you can try is:
- install the BulletAddonSimple.zip file in blender as an addon
- download Workingtest.blend
- run the python script that's open
- press space!

![25fps](https://github.com/ENR813/BlenderBulletPhysicsAddon/assets/50517527/7126508f-ab43-4b2c-9224-b066e387fceb)

Press Space to Start And Stop the Simulation! It will shop up as debug lines of the selected Mesh!

WARNING may Crash Blender.

Works With Blender 3.5

# For Contributors:
- There is some GUI Code Available in BrokenExampleGuiCode.py For A Quick startup/ lazy references, may be helpful for building a GUI in Blender 

# Requirements For Building:
- Visual Studio 2017 or higher
- Bullet Physics library
- CMake
- PyBinds Library
# Setup Tutorial and Tips For Building On Windows x64:
you will need to build it because it isn't finished yet (missing GUI only testing)
- install VC Package e.g. copy https://github.com/microsoft/vcpkg to a directory e.g. mystuff/
- Install bullet physics with vcpackage (run the command ./vcpkg install bullet3:x64-windows in the directory of vcpkg e.g "mystuff\vcpkg\")
- copy the bullet phyiscs header directory found at mystuff\vcpkg\installed\x64-windows\include\bullet into the current directory where you want to build the addon with the files from this page (current directory = e.g "mystuff\BlenderBulletPhysicsAddon\")
  (otherwise bullet physics won't find the headers and other problems) 
  you may also need to copy CommonFileIOInterface.h into the current directory
  (get the library working post errors, questions here or at the bullet library page)
- Copy the pybinds directory into the current directory (https://github.com/pybind/pybind11)
- install CMake (https://cmake.org/)
- create new folder "build"
- run the CMake GUI to build the Visual Studio project in the build Folder (select the current directory for "where is the source code", and for "where to build the binaries" is the build folder) then click Generate.
- open the Visual Studio project in the build Folder
- (build phys_view for the bullet-physics Demo / test if the library works)
- build mybinds to build the python library
- put the Addon_better.py in a zip file with the library you built which can be found in e.g. mystuff/BlenderBulletPhysicsAddon/build/Debug/Debug folder or the Release folders and possibly called mybinds.cp310-win_amd64.pyd

And that's it, the zip file is your addon.

# Addon Installation in Blender Reminder
In Blender 3.5 go to Edit/Preferences/Addons/Install select the zip file then click install addon. Then click the Checkbox next to the addons Name to enable it.
