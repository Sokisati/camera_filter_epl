# Task
Our team will be given a color code (more on this color code later) mid-flight by TEKNOFEST personnel. This code needs to be sent to our flight computer via ground station interface, which will then act accordingly.


# Color code
Color code is a 4 digit string in this format:
- 1st digit: Number between [1-9]. This number indicates the duration of the first filters color's activation.
- 2nd digit: Letter that is either R, G or B. This Letter indicates the first filter color.
- 3rd digit: Number between [1-9]. This number indicates the duration of the second filters color's activation.
- 4th digit: Letter that is either R, G or B. This Letter indicates the second filter color.

![](https://github.com/Sokisati/camera_filter_epl/blob/master/image/pepe.gif)

For example, 6G4B means filer disc will travel to green color, wait for 6 seconds, travel to blue color, wait for 4 seconds and travel back to neutral color.

# Actuator choice

This is an easy task, right? Just attach a step motor (because these kinds of scenarios is just the right ones for using a step motor) and that's it. 
Well, no. It is explicilty written in the competition guidelines that we have to use a servo motor. Okay, no big deal. We have 180 degree servo motor available (MG90) as a lightweight option. We can attach it to a 1:2 gear and we have servo with 360 degrees of movement! 
Yeah, don't wanna be the bearer of bad news but it is also explicitly written in the competition guidelines that servo shall rotate 90 degrees to travel one color and it can move only in clockwise direction.
We can't buy 360 degree servos because those are too expensive and too heavy.

At this point, you might say "what kind of a bullshit is that?" as a sane person and get on with your happy life. But no sir, mom did raise no quitter.
I watched a tutorial on how to hack a 180 servo motor to 360. Only problem is, it's not a "servo" motor at that point, it's more like a DC motor. I can't control position directly, because angle values correspond to speed. 90 means stop and either direction causes it so spin forever. 0 and 180 are max speed values.

So I thought I can derive an equation based on rotation speed and time to rotate it to desired angle. I wrote the code for it but problem with that is it's not consistent at all, because of friction. It's basically an open loop, I needed a close loop to make sure I am actually at the desired angle.

So I asked our teams mechanical guy to put some kind of a notch for every color and put a printer encoder, that way I can detect where filter is; it's essentialy a closed loop. 
So I need to write a program to count the rising or falling edges and stop when it's the right color.

![](https://github.com/Sokisati/camera_filter_epl/blob/master/image/disc_anim.gif)

# Angle to encoder counter
Because it can only move in one direction (get out of my head music group), I need an algorithm to calculate how many encoder signal edge (rising xor falling) I need. It's super simple, it just iterates over a predefined color list, and when it finally finds the desired color it returns the number of steps it needed to find it.


# move_step
This program is just a way for us to test it. It travels x number of steps, where x is the additional argument like python3 move_step.py 4. 
