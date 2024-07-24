Overview

Flight simulation is a crucial technology for both civil and military aviation, and the aerospace industry. Real-time simulation tools extend across all aspects of aircraft development, from aerodynamics to avionics and image generation systems. Knowledge of flight simulation software is vital for professional developers, educators, and students. 
This repository contains the necessary computer tools including open-source code required for anyone to design and develop a flight simulator.

This is a companion repository written by Professor Dave Allerton, a noted expert with decades of experience developing flight simulators in academia. 
The resources will enable readers to develop their own simulations with readily available open-source software. 
The resources are based upon the Flight Simulation Software book, which contains software taken from operational flight simulators and provides step-by-step guidance on software design, computer graphics, aircraft equations of motion, navigation and flight control systems, and more. 
The book is accompanied by a companion website, containing source code for flight simulation software with the link provided below:
www.wiley.com/go/flightsimulationsoftware


Folders

The PC version folder entails a group of files from different systems of the flight simulator including: the engine model, instructor station, navigation flight display, primary flight display etc.
The eng folder contains files required for the engine model, computing the thrust, fuel flow and other parameters for the aircraft engine.
The ios folder contains the instructor operating station files.
The nfd folder contains the files for the navigation and avionics computer, producing navigation display for the flight simulator.
The pfd folder contains the aircraft primary flight display files.
The MATLAB files folder includes code used to obtain an integrated interface to Mathworks MATLAB software. 
The interface enables students to design and develop prototype aircraft flight control code, where the output of the code is transmitted to the flight simulator to override flight control inputs. 
This external interface between the flight simulator and MATLAB allows rapid prototyping of flight control code in the MATLAB environment without the need to access, develop or recompile the flight simulator software. 

Instructions on Instalation from Author: 

This works with msys2, so you will need to download msys2. You can try to build it with mingw but I have not done this and cannot guarantee it will work (although it may).
You will also need to download the libraries using pacman:

    glfw3 OpenGL
    freetype2 - IOS fonts
    ftgl - IOS fonts
    libpng - screen dump
    libZ - needed by ftgl (I think)
    libalut (needed for the sound card)
    libopenal (needed for the sound card)

Just Google 'msys2 glfw' etc to find the libraries and the commands.

The libraries opengl32, glu32 gdi32 should already be installed on your system.
 
Download the folder sim/ to a local directory and the folder sim-include/
Copy the files in sim-include/ to /c/msys64/mingw64/include/SIM/
Install the libraries
in sim/libs/ type 'make clean' then 'make'
in sim/b747/ type 'make clean' then 'make'
to run type './b747'

The simulator should start immediately.

(Make sure the terminal resolution is set to 1920x1080)

I made a couple of minor changes from the Cranfield and my home versions as it runs without any controls:

    To HOLD (freeze) the simulator, press the space bar - press it again to resume
    To set the flaps, go to Settings/Flaps and enter a value between 0 (UP) and 30 (down)
    To set the gear position, go to Settings/Gear and select up or down

To try it out, set the gear down, set flaps 30 and go to Reposition/Restore and select GAT6.sav

In all other senses, it's the same as the Cranfield software. You can set DIAGNOSTICS to 1 in ios.c (line 36) to see how fast it runs on your PC. It may run slightly slowly with lots of graphics on the IOS because FTGL is very slow. This has been radically improved by my recent software using OpenGL shaders.

Find more instructions on INSTRUCTIONS.txt
