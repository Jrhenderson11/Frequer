# Frequer
My personal adaptation of Adafruits Freqshow (https://github.com/adafruit/FreqShow), designed to display SDR outputs, friendly to a raspberry pi touchscreen.

This was kind of a project for a hackthon: I was playing around with having a radio controlled with motion control using a Myo arm thingy. This is why there are a bunch of files called Myo. Most of the time was spent trying to make a horrible frankensteins monster by joining the 2 pieces of software together (ie having a radio in pygame whilst maintaining a connection to, polling and recieving inputs from the controller). It was fairly succesful, I could change the frequency with a motion of my hand, but joining the 2 together made it crash quite a bit.


## DISCLAIMER 
 (well at least I think this is a disclaimer)

 As said above this code (well some) was originally provided by adafruit for viewing software defined radio on a raspberry pi. At a hackathon I picked it apart to see what was what and played around with it, altering the interface and the way it provides data from the model to the interface and added motion control. But quite a lot of the code was not written by me, and I just changed the way it was presented and interacted with.


Original README from Adafruit:

	Raspberry Pi & PiTFT-based RTL-SDR frequency scanning and display tool. 
	See installation and usage instructions in the guide at:
	https://learn.adafruit.com/freq-show-raspberry-pi-rtl-sdr-scanner/


# Technical Stuff


- freqshow.py is my adaptation of FreqShow with no motion control stuff (interface and display changes mostly)

- testdemo.py is a visual debugger made by somebody else for connecting and viewing inputs from the myo armband

- myo_raw.py is my conjunction of the 2, it places the code for drawing the screen of freqshow inside the code that gets the myo data and gives instructions to the model (updating the frequency) upon user hand gestures


