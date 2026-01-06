# mjbots-cpp
This is a lite c++ project to include both Moteus and Pi3hat libraries from Mjbots Robotics. 

## Why
The original Moteus and Pi3hat libraries are comprehensive projects that include more things than one needs in a custom c++ project. 
The overall idea is to extract the header and source files for plain c++ compilation of Mjbots libraries. 
It is designed for cross compilation for Raspberry Pi OS, both 32-bit and 64-bit. You can also try to compile it directly on a Raspberry Pi. 

## What
The project structure is outlined as

- "/include" stores the header files from both Moteus and Pi3hat. 
- "/src" stores the source files from both Moteus and Pi3hat. 
- "/example" stores the example and test files. 
- "/cmake" stores the .cmake files for dependencies. 
- "/thirdparty" stores the original repositories as submodules. 

## How
To compile the libraries and examples, the easiest way is to run the bash script run_build.sh. 
Otherwise, one can build it as a plain cmake project. 

TODO: show cmake building steps

TODO: show how to setup cross compilation

## Acknowledgement
Star this repository if you like it. 
For the actual codes, please give credits to Mjbots Robotics. 