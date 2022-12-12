cmucam-viewer
=============

A simple SDL2/OpenGL application for watching a progressive frame dump from the CMUcam 1 [https://www.cs.cmu.edu/~cmucam/downloads.html].

Dependencies
------------
- SDL2
- GLEW
- OpenGL

Building
--------
```
git clone https://github.com/teknoman117/cmucam-viewer
cd cmucam-viewer
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -B build/Release .
cmake --build build/Release -j
build/Release/src/cmucam-viewer -d <path to CMUcam tty>
```

Inspiration
-----------
So why does one write a CMUcam 1 viewer in 2022?

Well, I had a conversion with someone at the last HBRC meeting about "retro" homebrew robotics and decided to pull some of the older components out of my parts bin: CMUcam v1, Basic Stamps, SRF04 sonars, etc.

I attempted to use the 20+ year old Java viewer but I wasn't able to get past opening a serial port and I really didn't try to debug it. It's interesting from a historical standpoint - it predates the Java serial port libraries like RXTX (which itself has been dead for 7 years). It just uses the serial ports via a FileStream and relies on an external program to set up the serial port correctly.

Either way, I don't think I've ever wrote a frame dump viewer for it in the 20 years I've had this camera, so it's just about time I suppose.