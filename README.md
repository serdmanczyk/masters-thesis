Masters Thesis Source Code
==============

This repositiory contains the source code used to implement the final demonstration system for my Masters of Science in Electrical Engineering thesis I completed at the Unversity of North Carolina at Charlotte.

The full system includes a base station and mobile nodes all communicating using [XBee 802.15.4 radios](http://www.digi.com/products/wireless-wired-embedded-solutions/zigbee-rf-modules/point-multipoint-rfmodules/xbee-series1-module) configured in API mode.

The 'base station' repository contains python code that runs on any computer which can interface with an XBee radio.

The 'node' folder contains C++ code written for a [Renesas Sakura](http://sakuraboard.net/gr-sakura_en.html) board, but adaptable to Arduino.

For more information, read my [full web write up of my work](http://serdmanczyk.github.io/MSEEThesis/) or skip ahead and read the [thesis in its entirety](http://webpages.uncc.edu/~jmconrad/GradStudents/Thesis_Erdmanczyk.pdf)(pdf).

Take note I came into this with an embedded background and was learning python as I was going, so there are definitely more efficient ways to do the things I did in Python.  Oh, if I had it all to do over again....
