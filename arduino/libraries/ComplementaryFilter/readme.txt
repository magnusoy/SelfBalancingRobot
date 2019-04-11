This is a C++ library for Arduino for filtering signals with a Complementary Filter.

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/lib/targets/libraries folder.

When installed, this library should look like:

Arduino/lib/targets/libraries/ComplementaryFilter                             (this library's folder)
Arduino/lib/targets/libraries/ComplementaryFilter/ComplementaryFilter.cpp     (the library implementation file)
Arduino/lib/targets/libraries/ComplementaryFilter/ComplementaryFilter.h       (the library description file)
Arduino/lib/targets/libraries/ComplementaryFilter/keywords.txt                (the syntax coloring file)
Arduino/lib/targets/libraries/ComplementaryFilter/examples                    (the examples in the "open" menu)
Arduino/lib/targets/libraries/ComplementaryFilter/readme.txt                  (this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you just have to start the Arduino application.
You may see a few warning messages as it's built.

To use this library in a sketch, go to the Sketch | Import Library menu and
select KalmanFilter.  This will add a corresponding line to the top of your sketch:
#include <ComplementaryFilter.h>

To stop using this library, delete that line from your sketch.

Geeky information:
After a successful build of this library, a new file named "Test.o" will appear
in "Arduino/lib/targets/libraries/ComplementaryFilter". This file is the built/compiled library
code.

If you choose to modify the code for this library (i.e. "ComplementaryFilter.cpp" or "ComplementaryFilter.h"),
then you must first 'unbuild' this library by deleting the "ComplementaryFilter.o" file. The
new "ComplementaryFilter.o" with your code will appear after the next press of "verify"

