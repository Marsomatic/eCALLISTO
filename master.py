import ephem
import time
import sys
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, Angle
from astropy.time import Time
from datetime import datetime, timedelta
import RPi.GPIO as g
from constants import DEG_PER_STEP, RAD_TO_DEG_FACTOR, SLEEP_TIME, DEG_PER_SECOND, PRINT_FREQ, HA_HOME_ABS_POSITION, HOME_HA, DEC_HOME_ABS_POSITION, HOME_DEC, MENU_STRING, LAT, LON, ALTITUDE
from routines import convertToEquatorial, cleanup, initMotors, moveStepper
import pytz

# local timezone
tz = pytz.timezone('Europe/Berlin')

# This is initialized as in the middle of the stepper range (1 revolution is 690 000 steps), needs to be updated when homed
absoluteStepperState = [345_000, 345_000] # for RA and Dec

# Stepper control initialization
# pins of the 2 motors, 4 coils each
motors = [[15, 13, 12, 11], 
          [32, 33, 31, 29]]

# States a stepper motor can be in row --> state; column --> coil nr.
states = [
    [0, 0, 0, 1],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0]
]

# pins of the photointerrupters used for homing. RA and Dec axis respectively
limits = [38, 40]

# Initializing the GPIO library, motor GPIO pins and the optical limit sensors 
g.setmode(g.BOARD)
g.setwarnings(False)
initMotors()
g.setup(limits[0], g.IN)
g.setup(limits[1], g.IN)

# PyEphem variables
pointing = [datetime.now(tz), 0, 0]
observer = ephem.Observer()
observer.lon = str(LON)
observer.lat = str(LAT)
observer.date = datetime.now(tz)
observer.elevation = ALTITUDE
sun = ephem.Sun(observer)

# Astropy variables
loc = EarthLocation(lat = LAT*u.deg, lon = LON*u.deg, height = ALTITUDE*u.m)

# Finding intial pointing
# Use below if mounted
# Pointing format: [Date and time, RA, Dec]
# HomeCoords = convertToEquatorial(30, 180)
# Pointing = [observer.date.datetime(), homeCoords[0], homeCoords[1]]

# Pointing [time, right ascension, declination] (in degrees)
# pointing = [observer.date.datetime(), float(sun.ra) * RAD_TO_DEG_FACTOR, float(sun.dec) * RAD_TO_DEG_FACTOR]

print('           UTC             |   Sun RA      Sun Dec     Sun HA    |    antenna RA     antenna Dec     antenna HA    |     absoluteStepperState     ')

lastPrint = datetime.now(tz)

# Tracking the sun
def trackSun():
    '''
    tracks the sun assuming the antenna has been homed
    '''
    global pointing
    global observer
    global sun
    global loc
    global lastPrint
    global absoluteStepperState
    
    while True:
        try:
            # Update PyEphem variables: time, sun coords
            timenow = datetime.now(tz)
            observer.date = timenow
            sun.compute(observer)
            
            # Update antenna pointing due to earth rotation
            pointing[1] -= (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
            pointing[0] = timenow
            
            # Compute local hour angle of the pointing
            lmst = Time(datetime.now(tz), format = 'datetime', scale='utc')
            lha = (Angle(lmst.sidereal_time('apparent', loc)).degree + (pointing[1]))%360
            sunHourAngle = (Angle(lmst.sidereal_time('apparent', loc)).degree + (float(sun.ra) * RAD_TO_DEG_FACTOR))%360
            
            # Moves ra stepper to track the sun
            if sunHourAngle < lha - DEG_PER_STEP:
                absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
                pointing[1] -= DEG_PER_STEP
                if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                    print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {sunHourAngle} | {round(pointing[1], 9)}, {round(pointing[2], 9)}, {round(lha, 9)} | {absoluteStepperState}')
                    lastPrint = timenow
            elif sunHourAngle> lha + DEG_PER_STEP:
                absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
                pointing[1] += DEG_PER_STEP
                if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                    print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {sunHourAngle} | {round(pointing[1], 9)}, {round(pointing[2], 9)}, {round(lha, 9)} | {absoluteStepperState}')
                    lastPrint = timenow
            
            if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                    print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {sunHourAngle} | {round(pointing[1], 9)}, {round(pointing[2], 9)}, {round(lha, 9)} | {absoluteStepperState}')
                    lastPrint = timenow
            
            cleanup(motors)
            
        except KeyboardInterrupt:
            # goes back to main menu
            cleanup(motors)
            break
            
# Homing
def home():
    '''
    drives the antenna to the home position
    '''
    try:
        global pointing
        global observer
        global sun
        global loc
        global lastPrint
        global absoluteStepperState
        
        # drives RA axis towards home position
        
        print('homing RA...')
        while g.input(38):    
            absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
            time.sleep(SLEEP_TIME)
        print('end stop reached')

        # sets RA in home position
        absoluteStepperState[0] = HA_HOME_ABS_POSITION
        lmst = Time(datetime.utcnow(), format = 'datetime', scale='utc') # local mean sidereal time
        siderealTime = Angle(lmst.sidereal_time('apparent', loc)).degree
        pointing[1] = (siderealTime - HOME_HA)%360
        print('RA homed!')
        
        cleanup(motors)
        
    except KeyboardInterrupt:
        cleanup(motors)
        return
        
def goto(targetRa):
    '''
    goes to a given RA-Dec
    '''
    try:
        global pointing
        global observer
        global sun
        global loc
        global lastPrint
        global absoluteStepperState

        while True:
            # Update PyEphem variables: time, sun coords
            timenow = datetime.now(tz)
            observer.date = timenow
            sun.compute(observer)

            # Update antenna pointing due to earth rotation
            pointing[1] -= (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
            pointing[0] = timenow

            # Compute local hour angle of the pointing
            lmst = Time(datetime.now(tz), format = 'datetime', scale='utc')
            lha = (Angle(lmst.sidereal_time('apparent', loc)).degree + (pointing[1]))%360
            sunHourAngle = (Angle(lmst.sidereal_time('apparent', loc)).degree + (sun.ra))%360

            # Moves ra stepper to go-to/track the target
            if targetRa < pointing[1] - DEG_PER_STEP:
                while targetRa < pointing[1]:
                    absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
                    pointing[0] = timenow
                    pointing[1] -= DEG_PER_STEP
                    timenow = datetime.now(tz)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {float(sunHourAngle)} | {round(pointing[1], 9)}, {round(pointing[2], 9)}, {round(lha, 9)} | {absoluteStepperState}')
                        lastPrint = timenow

            elif targetRa > pointing[1] + DEG_PER_STEP:
                while targetRa > pointing[1]:
                    absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
                    pointing[0] = timenow
                    pointing[1] += DEG_PER_STEP
                    timenow = datetime.now(tz)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {float(sunHourAngle)} | {round(pointing[1], 9)}, {round(pointing[2], 9)}, {round(lha, 9)} | {absoluteStepperState}')
                        lastPrint = timenow

            cleanup(motors)

    except KeyboardInterrupt:
        cleanup(motors)
        return pointing[1]

def manual(raSteps, decSteps):
    '''
    manually moves motors by a given amount of steps (direction is indicated with positive or negative values)
    '''
    try:
        global pointing
        global observer
        global sun
        global loc
        global lastPrint
        global absoluteStepperState

        if raSteps < 0:
            try:
                print(f'brrrrrrrrrrrrrrrrrrrrr {absoluteStepperState}')
                absoluteStepperState = moveStepper(0, raSteps, -1, absoluteStepperState)
            except KeyboardInterrupt:
                cleanup(motors)
                return
        if raSteps > 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                absoluteStepperState = moveStepper(0, raSteps, 1, absoluteStepperState)
            except KeyboardInterrupt:
                cleanup(motors)
                return

        if decSteps < 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                absoluteStepperState = moveStepper(1, decSteps, -1, absoluteStepperState)
            except KeyboardInterrupt:
                cleanup(motors)
                return
        if decSteps > 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                absoluteStepperState = moveStepper(1, decSteps, 1, absoluteStepperState)
            except KeyboardInterrupt:
                cleanup(motors)
                return
    except KeyboardInterrupt:
        cleanup(motors)
        return
    
def coords():
    '''
    prints out current pointing of the antenna along with the sun coordinates
    '''
    global pointing
    global observer
    global sun
    global loc
    global lastPrint
    global absoluteStepperState

    # Update PyEphem variables: time, sun coords
    timenow = datetime.now(tz)
    observer.date = timenow
    sun.compute(observer)

    # Update antenna pointing due to earth rotation
    pointing[1] -= (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
    pointing[0] = timenow

    # Compute local hour angle of the pointing
    lmst = Time(timenow, format = 'datetime', scale='utc')
    lha = (Angle(lmst.sidereal_time('apparent', loc)).degree + (pointing[1]))%360
    sunHourAngle = (Angle(lmst.sidereal_time('apparent', loc)).degree + (sun.ra))%360

    print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {float(sunHourAngle) * RAD_TO_DEG_FACTOR} | {round(pointing[1], 9)}, {round(pointing[2], 9)} {round(lha, 9)} | {absoluteStepperState}')

# Main loop
while True:
    cleanup(motors)
    continuation = input(MENU_STRING)
    if continuation == 't':
        observer.date = datetime.now(tz)
        sun.compute(observer)
        if sun.alt > 0:
            print('Starting tracking')
            trackSun()
        else:
            print(f'the sun is below horizon: {sun.alt}')
    elif continuation == 'h':
        home()
    elif continuation == 'goto':
        coords()
        ra = float(input('target RA (in deg): '))
        print(ra)
        goto(ra)
    elif continuation == 'm':
        raSteps = int(input('RA steps: '))
        decSteps = int(input('DEC steps: '))
        manual(raSteps, decSteps)
        print('Done!')
    elif continuation == 'coords':
        coords()
    else:
        confirmation = input('Are you sure about that? [y/n]\n>>> ')
        if confirmation == 'y':
            break
        else:
            continue