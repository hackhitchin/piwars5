# Piwars 4.0
Back at it again!

## Aim for this year
We decided this year to go back to grass routes and keep things as simple as possible. This notion lasted about 3 weeks before I saw a funky new distance sensor on kickstarter called the "protractor". It returns the distance and angle to the nearest item in its field of view. Kinda cool!

## Grass Routes
The design is using the basic shape of TITO 1 in that it has 4x wheels which are each powered by a 12v 1000rpm motor. The front two wheels are clamped to an articulated front end that allows one or other side to rise/fall independantly of the rear half.  
  
Having had a similar feature on TITO 1 and subsequently not on TITO 2, we now realise it's importance.  
  
This articulation allows all 4 wheels get best traction on a flat or undulating surface. Without it, small imperfections in tires / 3D printed chassis / or even on the floor equate to a single or multiple wheels not touching the ground and the bot not driving straight. 

## Differences
One of our main issues in past years (excluding idiotic ideas) has been our motor controllers. We have typically used remote control car ESC's due to their low cost and high power ability. What became all too apparent last year was the downside to these wonderful controllers: their fluctuating calibration with temperature, time and apparently whether a butterfly flaps it wings in Australia.  
  
We have spent hours, no, days testing and calibrating motors to drive straight under hard acceleration, only to turn it all off, go home and sleep, then return to "finish off the last little bit" and notice it doing something completely different :(  
  
The new motor controllers (although more expensive) are straight H-Bridges that can handle 12amps continuously or 30amps for a second or two. They hopefully won't suffer wondering calibration issues like we have gotten used too. 

## Install Instructions
```
cd ~  
mkdir Projects  
cd Projects  
git clone https://github.com/hackhitchin/piwars4.git  
sudo apt-get install python-dev python-setuptools  
sudo easy_install -U RPIO  
sudo pip install --upgrade pip enum34  
sudo pip install ordereddict
```  
  
Note: I2C must be enabled in the interfacing options of raspi-config.

## OpenCV ball finding
Before running example_ball, you'll want to white balance for your local lighting conditions.  
  
### Calibrating White Balance
Hold a piece of white paper in front of the camera and run ```whitebalance.py```. This will work out the optimal camera gains and save them locally for other scripts to use.  

### Testing Call Finder Module
```example_ball.py``` should run standalone and detect the coloured balls, I haven't tried it as part of the overall code.  

### Notes
Both calibration and ball finder module may complain if there isn't a display for them to use; run ```export DISPLAY=:0``` to tell them to send output to the Pi desktop.
