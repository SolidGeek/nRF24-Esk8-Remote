# nRF24-Esk8-Remote

After building and hacking several together, I ended up with this design of my own based on electronics and software from SolidGeek (available at https://github.com/SolidGeek/nRF24-Esk8-Remote).
 
The remote I designed has the following features:

- A dead man's switch which deactivates the throttle when the remote is not held (safety feature because the trigger can accidentally go off otherwise, ask me how I know)
- A mode switch which swaps between displayed data (Speed, distance, battery voltage, Ah drawn, current draw)
- Ergonomic OLED display position so you don't need to twist your wrist to read it.
- Menu to adjust settings
- Programmable deck selection via menu
- Calibration of throttle via menu
- Ambidextrous design, swap to left hand use by adjusting the script-
- Skate bearing used in trigger mechanism for smoother operation as well just a fun detail considering it's meant for an esk8
- Hall sensor instead of potentiometer to prevent potential wear of a pot

STLs available at https://www.thingiverse.com/thing:2800544

Youtube video available at https://www.youtube.com/watch?v=gQl7mLMAiAs&feature=youtu.be

Part List available at https://docs.google.com/spreadsheets/d/1vXR9ce0m25Ap6XxzlymFo_pfpIeT2RAqWCypW-a-Jes/edit?usp=sharing

PCB Files available at https://easyeda.com/ervinelin/New_Project-35f7bf3537744cda8e1064904f6a78a5

Forum discussion available at https://www.electric-skateboard.builders/t/diy-trigger-style-remote-with-telemetry-complete-guide/48231

Build guide available at https://github.com/ModMiniMan/nRF24-Esk8-Remote/wiki

You will need to have intermediate soldering skills to complete the build as well as knowledge on how to program an Arduino and adjust simple code. Of course if you print this yourself you will need to know how to 3D print.

Caveats:
Remote is experimental and not an off the shelf product that has undergone proper testing. Build and use at your own risk. Please bench test extensively before actual use. Lastly, I highly recommend protective gear when using the remote just in case something doesn't work (many many things can go wrong)
