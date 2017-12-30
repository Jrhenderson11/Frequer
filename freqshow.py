# FreqShow main application and configuration.
# Author: Tony DiCola (tony@tonydicola.com)
#
# The MIT License (MIT)
#
# Copyright (c) 2014 Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import os
import time

import pygame

import controller
import model
import ui

import views

#import myo_raw
# Application configuration.
SDR_SAMPLE_SIZE = 2048	# Number of samples to grab from the radio.  Should be
						# larger than the maximum display width.

CLICK_DEBOUNCE  = 0.4	# Number of seconds to wait between clicks events. Set
						# to a few hunded milliseconds to prevent accidental
						# double clicks from hard screen presses.

# Font size configuration.
MAIN_FONT = 33
NUM_FONT  = 50

# Color configuration (RGB tuples, 0 to 255).
MAIN_BG        = (  0,   0,   0) # Black
INPUT_BG       = ( 60, 255, 255) # Cyan-ish
INPUT_FG       = (  0,   0,   0) # Black
CANCEL_BG      = (128,  45,  45) # Dark red
ACCEPT_BG      = ( 45, 128,  45) # Dark green
BUTTON_BG      = ( 60,  60,  60) # Dark gray
BUTTON_FG      = (255, 255, 255) # White
BUTTON_BORDER  = (200, 200, 200) # White/light gray
INSTANT_LINE   = ( 34, 188,  42) # Bright yellow green.
SLIGHTLY_GRAY  = (100, 100, 100)
# Define gradient of colors for the waterfall graph.  Gradient goes from blue to
# yellow to cyan to red.
WATERFALL_GRAD = [(0, 0, 255), (0, 255, 255), (255, 255, 0), (255, 0, 0)]
WATERFALL_GRAD2 = [(0, 0, 0), (0, 0, 50), (0, 0, 200), (0, 200, 255)]

# Configure default UI and button values.
ui.MAIN_FONT = MAIN_FONT
ui.Button.fg_color     = BUTTON_FG
ui.Button.bg_color     = BUTTON_BG
ui.Button.border_color = BUTTON_BORDER
ui.Button.padding_px   = 2
ui.Button.border_px    = 2

def run():
	# Initialize pygame and SDL to use the PiTFT display and touchscreen.
	#os.putenv('SDL_VIDEODRIVER', 'fbcon')
	#os.putenv('SDL_FBDEV'      , '/dev/fb1')
	#os.putenv('SDL_MOUSEDRV'   , 'TSLIB')
	#os.putenv('SDL_MOUSEDEV'   , '/dev/input/touchscreen')
	pygame.display.init()
	pygame.font.init()
	#pygame.mouse.set_visible(False)
	# Get size of screen and create main rendering surface.
	size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
	print "w: " + str(size[0])
	print "h: " + str(size[1])

	screen = pygame.display.set_mode(size, pygame.RESIZABLE)
	# Display splash screen.
	splash = pygame.image.load('freqshow_splash.png')
	screen.fill(MAIN_BG)
	screen.blit(splash, ui.align(splash.get_rect(), (0, 0, size[0], size[1])))
	pygame.display.update()
#	splash_start = time.time()
	# Create model and controller.
	print "build model with " + str(size[0]) + ", " + str(size[0]) 
	fsmodel = model.FreqShowModel(size[0], size[1])
	fscontroller = controller.FreqShowController(fsmodel)
	#time.sleep(2.0)
	# Main loop to process events and render current view.
	lastclick = 0
	while True:
		# Process any events (only mouse events for now).
		for event in pygame.event.get():
			if event.type is pygame.MOUSEBUTTONDOWN \
				and (time.time() - lastclick) >= CLICK_DEBOUNCE:
				lastclick = time.time()
				fscontroller.current().click(pygame.mouse.get_pos())
		
		# Update and render the current view.
		fscontroller.current().render(screen)
		#fscontroller.multiview().render(screen)
		#for x in fscontroller.both_views():
		#	x.render(screen)

		#views.WaterfallSpectrogram(model, fscontroller).render(screen)
		#views.InstantSpectrogram(model, fscontroller).render(screen)
		pygame.display.update()

if __name__ == '__main__':
	
	run()
