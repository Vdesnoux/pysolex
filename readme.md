This repository is a Sand Box on SpectroHeliography imaging acquisition with Sol'EX spectrograph from Christian Buil and Zwo Camera.
it includes also the pdf of the Atelier 3 python Shleyak which refers to a previous version of acquisition without thread - not included anymore.

GUI of script

Preview:  shows the video from camera - to stop, click on stop button

Image:    capture one Image ans save it to png - to exit press any keyboard entry, the only safe way to close the opencv window

Capture:  shows the video from camera, save frame into ser file - to stop press 'q' on the keyboard

Video:    shows the video from camera and the realtime building of the monochromatic image - to stop press 'q' on the keyboard - 
          Disk image will then be displayed in openCV window and is NOT bring to front - to exit, press any key on the keyboard
	
Stop:     Stop the preview, and only the preview mode


Update:   in preview mode, during acquisition, can update the gain, exposure - better to stop to update ROI and relaunch the preview mode


Sometimes ZWO raises some timeout errors, do not understand the root cause, more often when updating ROI

