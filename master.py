"""
Created on Jul 31 2024
Updated on Jul 31 2024
main control program for the callisto station Visnjan

@author: M. Markovic
"""

import ephem
import time
import astropy.units as u
from astropy.coordinates import EarthLocation, Angle
from astropy.time import Time
from datetime import datetime, timedelta
import RPi.GPIO as g
from constants import *
from routines import *
import pytz
try:
    from src.TMC_2209.TMC_2209_StepperDriver import *
    from src.TMC_2209._TMC_2209_GPIO_board import Board
except ModuleNotFoundError:
    from TMC_2209.TMC_2209_StepperDriver import *
    from TMC_2209._TMC_2209_GPIO_board import Board

# local timezone
tz = pytz.timezone('Europe/Berlin')

# This is initialized as in the middle of the stepper range (1 revolution is 660 000 steps), needs to be updated when homed
absoluteStepperState = [STEPS_PER_ROT/2, STEPS_PER_ROT/2] # for RA and Dec
# Format: time; RA; DEC
pointing = [datetime.now(tz), 0, 0]

# Initializing the motor GPIO pins and the optical limit sensors 
# all in GPIO notation, not physical
# initiate and setup the TMC_2209 class
enPin = 21
dirPin = 20
stepPin = 16
limit = 17

tmc1 = TMC_2209(enPin, stepPin, dirPin)
g.setup(limit, g.IN)
setupTMC(tmc1)

# PyEphem variables
observer = ephem.Observer()
observer.lon = str(LON)
observer.lat = str(LAT)
observer.date = datetime.now(tz)
observer.elevation = ALTITUDE
sun = ephem.Sun(observer)

# Astropy variables
loc = EarthLocation(lat = LAT*u.deg, lon = LON*u.deg, height = ALTITUDE*u.m)

print('           UTC             |   Sun RA      Sun Dec     Sun HA    |    antenna RA     antenna Dec     antenna HA    |     absoluteStepperState     ')

lastPrint = datetime.now(tz)

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

    observer.date = datetime.now(tz)
    sun.compute(observer)
    
    waitForSchedule()
    print("Sun: ", sun.ra * RAD_TO_DEG_FACTOR, "Antenna: ", pointing[1])
    while sun.alt < 0:
        observer.date = datetime.now(tz)
        sun.compute(observer)
        time.sleep(15)
    goto(sun.ra * RAD_TO_DEG_FACTOR, True)
    print('tracking')
    
    obsEndTime = datetime.now().replace(hour=STOP_TIME_HOUR, minute=STOP_TIME_MINUTE, second=0, microsecond=0)
    
    try:
        while True:
            # Update PyEphem variables: time, sun coords
            timenow = datetime.now(tz)
            observer.date = timenow
            sun.compute(observer)
            
            # Update antenna pointing due to earth rotation
            pointing[1] += (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
            pointing[0] = timenow
            
            # Compute local hour angle of the pointing
            lmst = Time(datetime.now(tz), format = 'datetime', scale='utc')
            siderealTime = observer.sidereal_time()
            lha = (siderealTime * RAD_TO_DEG_FACTOR - (pointing[1]))%360
            sunHourAngle = (Angle(lmst.sidereal_time('apparent', loc)).degree - (float(sun.ra) * RAD_TO_DEG_FACTOR))%360
            
            if sun.alt > 0:
                # Moves ra stepper to track the sun
                if sunHourAngle < lha - DEG_PER_STEP and lha - sunHourAngle < 180:
                    # absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
                    # pointing[1] += DEG_PER_STEP
                    goto(sun.ra * RAD_TO_DEG_FACTOR, True)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
                elif sunHourAngle > lha + DEG_PER_STEP and sunHourAngle - lha < 180:
                    # absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
                    # pointing[1] -= DEG_PER_STEP
                    goto(sun.ra * RAD_TO_DEG_FACTOR, True)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
                elif sunHourAngle < lha - DEG_PER_STEP and lha - sunHourAngle > 180:
                    # absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
                    # pointing[1] -= DEG_PER_STEP
                    goto(sun.ra * RAD_TO_DEG_FACTOR, True)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
                elif sunHourAngle > lha + DEG_PER_STEP and sunHourAngle - lha > 180:
                    # absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
                    # pointing[1] += DEG_PER_STEP
                    goto(sun.ra * RAD_TO_DEG_FACTOR, True)
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
            cleanup(tmc1)
            if timenow.timestamp() > obsEndTime.timestamp():
                    home()
                    trackSun()
            
            time.sleep(1)
            
            
    except KeyboardInterrupt:
        # goes back to main menu
        cleanup(tmc1)
        return

def home():
    '''
    drives the antenna to the home position
    '''
    try:
        global pointing
        global observer
        global sun
        global loc
        global absoluteStepperState
        global ser
        
        # drives RA axis towards home position
        print('homing RA...')
        while g.input(38):
            absoluteStepperState = moveStepper(tmc1, 1)
            time.sleep(SLEEP_TIME)
        print('end stop reached')
        
        timenow = datetime.now(tz)
        sun.compute(observer)

        # sets RA in home position
        absoluteStepperState[0] = HA_HOME_ABS_POSITION
        siderealTime = observer.sidereal_time()
        pointing[1] = (siderealTime * RAD_TO_DEG_FACTOR - HOME_HA)%360
        pointing[0] = timenow
        print('RA homed!')
        
        cleanup(tmc1)
        coords()
        
    except KeyboardInterrupt:
        cleanup(tmc1)
        return
        
def goto(targetRa, tracking):
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
            pointing[1] += (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
            pointing[0] = timenow

            # Compute local hour angle of the pointing
            lmst = Time(datetime.now(tz), format = 'datetime', scale='utc')
            siderealTime = observer.sidereal_time()
            lha = (siderealTime * RAD_TO_DEG_FACTOR - (pointing[1]))%360
            sunHourAngle = (Angle(lmst.sidereal_time('apparent', loc)).degree - (float(sun.ra) * RAD_TO_DEG_FACTOR))%360

            # Moves ra stepper to go-to/track the target
            if targetRa < pointing[1] - DEG_PER_STEP and pointing[1] - targetRa < 180:
                while targetRa < pointing[1]:
                    absoluteStepperState = moveStepper(tmc1, -1)
                    pointing[0] = timenow
                    pointing[1] -= DEG_PER_STEP
                    if pointing[1] < 0:
                        pointing[1] = 360 + pointing[1]
                    timenow = datetime.now(tz)
                    if tracking:
                        if abs(pointing[1] - targetRa) < 0.01:
                            return
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow

            elif targetRa > pointing[1] + DEG_PER_STEP and targetRa - pointing[1] < 180:
                while targetRa > pointing[1]:
                    absoluteStepperState = moveStepper(tmc1, 1)
                    pointing[0] = timenow
                    pointing[1] += DEG_PER_STEP
                    if pointing[1] > 360:
                        pointing[1] = pointing[1] - 360
                    timenow = datetime.now(tz)
                    if tracking:
                        if abs(targetRa - pointing[1]) < 0.01:
                            return
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
            
            elif targetRa < pointing[1] - DEG_PER_STEP and pointing[1] - targetRa > 180:
                while targetRa < pointing[1]:
                    absoluteStepperState = moveStepper(tmc1, 1)
                    pointing[0] = timenow
                    pointing[1] += DEG_PER_STEP
                    if pointing[1] > 360:
                        pointing[1] = pointing[1] - 360
                    timenow = datetime.now(tz)
                    if tracking:
                        if abs(targetRa - pointing[1]) < 0.01:
                            return
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow
                        
            elif targetRa > pointing[1] + DEG_PER_STEP and targetRa - pointing[1] > 180:
                while targetRa > pointing[1]:
                    absoluteStepperState = moveStepper(tmc1, -1)
                    pointing[0] = timenow
                    pointing[1] -= DEG_PER_STEP
                    if pointing[1] < 0:
                        pointing[1] = 360 + pointing[1]
                    timenow = datetime.now(tz)
                    if tracking:
                        if abs(pointing[1] - targetRa) < 0.01:
                            return
                    if (timenow - lastPrint).total_seconds() >= PRINT_FREQ:
                        printAllCoords(sunHourAngle, lha)
                        lastPrint = timenow

            cleanup(tmc1)

    except KeyboardInterrupt:
        cleanup(tmc1)
        return pointing[1]

def gotoZenith():
    '''
    goes to zenith assuming antenna is at home position
    '''
    global absoluteStepperState
    now = datetime.utcnow()
    print(f'{now}going to zenith, this will take approx. 3min')
    zenithSteps = STEPS_PER_ROT / 4
    absoluteStepperState = moveStepper(tmc1, -int(zenithSteps))
    now = datetime.utcnow()
    print(f'arrived at zenith at {now}(UTC)')
    
def manual(raSteps, decSteps):
    """ manually moves the two axes by the specified amount of steps

    Args:
        raSteps (int): positive or negative hour axis steps
        decSteps (int): postitive or negative declination axis steps
    """
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
                moveStepper(tmc1, -raSteps)
            except KeyboardInterrupt:
                cleanup(tmc1)
                return
        if raSteps > 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                moveStepper(tmc1, raSteps)
            except KeyboardInterrupt:
                cleanup(tmc1)
                return

        if decSteps < 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                moveStepper(tmc1, -decSteps)
            except KeyboardInterrupt:
                cleanup(tmc1)
                return
        if decSteps > 0:
            try:
                print('brrrrrrrrrrrrrrrrrrrrr')
                moveStepper(tmc1, decSteps)
            except KeyboardInterrupt:
                cleanup(tmc1)
                return
    except KeyboardInterrupt:
        cleanup(tmc1)
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
    pointing[1] += (timenow - pointing[0]).total_seconds() * DEG_PER_SECOND
    pointing[0] = timenow

    # Compute local hour angle of the pointing
    siderealTime = observer.sidereal_time()
    lha = (siderealTime * RAD_TO_DEG_FACTOR - (pointing[1]))%360
    sunHourAngle = (siderealTime - (float(sun.ra) * RAD_TO_DEG_FACTOR))%360

    printAllCoords(sunHourAngle, lha)

def printAllCoords(sunHourAngle, lha):
    print(f'{pointing[0]} | {float(sun.ra) * RAD_TO_DEG_FACTOR}, {float(sun.dec) * RAD_TO_DEG_FACTOR}, {sunHourAngle} | {round(pointing[1], 9)}, {round(pointing[2], 9)} {round(lha, 9)} | {absoluteStepperState}')
    
def waitForSunrise():
    print('Waiting for sunrise')
    while True:
        observer.date = datetime.now(tz)
        sun.compute(observer)
        if sun.alt > 0:
            print('Good morning world')
            break
        time.sleep(30)
    return

def waitForSchedule():
    print('Waiting for next scheduled event')

    while True:
        starttime = datetime.now(tz).replace(hour=START_TIME_HOUR, minute=START_TIME_MINUTE, second=0, microsecond=0)
        ovstime = datetime.now(tz).replace(hour=OVS_TIMEH, minute=OVS_TIMEM, second=0, microsecond=0)
        obsEndTime = datetime.now().replace(hour=STOP_TIME_HOUR, minute=STOP_TIME_MINUTE, second=0, microsecond=0)
        timenow = datetime.now(tz)
        observer.date = timenow
        sun.compute(observer)
        if timenow >= (starttime + timedelta(hours=-1)) and timenow.timestamp() <= (obsEndTime).timestamp():
            print(f'{timenow}: good morning world')
            break
        if timenow > ovstime + timedelta(minutes=-15) and timenow < ovstime + timedelta(minutes=15):
            gotoZenith()
            time.sleep(1800)
            print(f'{timenow}: going back home')
            home()
        time.sleep(30)
    return
# ===== Main loop manual control =====
# if __name__ == '__main__':
#     try:
#         while True:
#             cleanup(tmc1)
#             continuation = input(MENU_STRING)
#             if continuation == 't':
#                 trackSun()
#             elif continuation == 'h':
#                 home()
#             elif continuation == 'goto':
#                 coords()
#                 ra = float(input('target RA (in deg): '))
#                 print(ra)
#                 goto(ra, False)
#             elif continuation == 'm':
#                 raSteps = int(input('RA steps: '))
#                 decSteps = int(input('DEC steps: '))
#                 moveStepper(tmc1, raSteps)
#                 # manual(raSteps, decSteps)
#                 print('Done!')
#             elif continuation == 'coords':
#                 coords()
#             elif continuation == 'clean':
#                 cleanup(tmc1)
#             else:
#                 confirmation = input('Are you sure about that? [y/n]\n>>> ')
#                 if confirmation == 'y':
#                     break
#                 else:
#                     continue
#     except KeyboardInterrupt:
#         fWrite = open('lastPos.txt', 'w')
#         fWrite.write(f'{pointing[1]} {pointing[2]} {absoluteStepperState[0]}')
#         fWrite.close()
#         tmc1.set_motor_enabled(False)
#         del tmc1

# ===== Main loop auto control =====
if __name__ == '__main__':
    try:
        cleanup(motors)
        home()
        trackSun()
            
    except KeyboardInterrupt:
        fWrite = open('lastPos.txt', 'w')
        fWrite.write(f'{pointing[1]} {pointing[2]} {absoluteStepperState[0]}')
        fWrite.close()
