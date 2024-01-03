import ephem
import time
import astropy.units as u
from astropy.coordinates import EarthLocation, Angle
from astropy.time import Time
from datetime import datetime
import RPi.GPIO as g
from constants import *
from routines import cleanup, initMotors, moveStepper
import pytz

# local timezone
tz = pytz.timezone('Europe/Berlin')

# This is initialized as in the middle of the stepper range (1 revolution is 660 000 steps), needs to be updated when homed
absoluteStepperState = [330_000, 330_000] # for RA and Dec
# Format: time; RA; DEC
pointing = [datetime.now(tz), 0, 0]

# Stepper control initialization
# pins of the 2 motors, 4 coils each
motors = [[15, 13, 12, 11], 
          [32, 33, 31, 29]]

# pins of the photointerrupters used for homing. RA and Dec axis respectively
limits = [38, 40]

# Initializing the GPIO library, motor GPIO pins and the optical limit sensors 
g.setmode(g.BOARD)
g.setwarnings(False)
initMotors()
g.setup(limits[0], g.IN)
g.setup(limits[1], g.IN)

# Open file
fRead = open('lastPos.txt', 'r')
line = fRead.readline()
filelist = line.split()

# Pointing format: [Date and time, RA, Dec]
if len(line) != 0:
    pointing = [datetime.now(tz), float(filelist[0]), float(filelist[1])]
    absoluteStepperState = [int(filelist[2]), 330000]
fRead.close()

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
    if sun.alt > 0:
        goto(sun.ra * RAD_TO_DEG_FACTOR, True)
    print('tracking')
    
    
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
            cleanup(motors)
            if timenow.hour >= STOP_TIME_HOUR and timenow.minute >= STOP_TIME_MINUTE:
                    home()
                    trackSun()
            
            time.sleep(1)
            
            
    except KeyboardInterrupt:
        # goes back to main menu
        cleanup(motors)
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
        
        # drives RA axis towards home position
        print('homing RA...')
        while g.input(38):    
            absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
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
        
        cleanup(motors)
        coords()
        
    except KeyboardInterrupt:
        cleanup(motors)
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
                    absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
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
                    absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
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
                    absoluteStepperState = moveStepper(0, 1, 1, absoluteStepperState)
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
                    absoluteStepperState = moveStepper(0, 1, -1, absoluteStepperState)
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

            cleanup(motors)

    except KeyboardInterrupt:
        cleanup(motors)
        return pointing[1]

def gotoZenith():
    home()
    zenithSteps = STEPS_PER_ROT / 4
    moveStepper(0, zenithSteps, 1, absoluteStepperState)

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
        timenow = datetime.now(tz)
        observer.date = timenow
        sun.compute(observer)
        if timenow.hour >= START_TIME_HOUR - 1 and sun.alt > 0:
            print('good morning world')
            break
        if timenow.hour >= OVS_TIMEH - 1 and timenow.hour < OVS_TIMEH + 1:
            gotoZenith()
            if timenow.hour > OVS_TIMEH:
                home()
        time.sleep(30)
    return        
# ===== Main loop manual control =====
# try:
#     while True:
#         cleanup(motors)
#         continuation = input(MENU_STRING)
#         if continuation == 't':
#             trackSun()
#         elif continuation == 'h':
#             home()
#         elif continuation == 'goto':
#             coords()
#             ra = float(input('target RA (in deg): '))
#             print(ra)
#             goto(ra, False)
#         elif continuation == 'm':
#             raSteps = int(input('RA steps: '))
#             decSteps = int(input('DEC steps: '))
#             manual(raSteps, decSteps)
#             print('Done!')
#         elif continuation == 'coords':
#             coords()
#         else:
#             confirmation = input('Are you sure about that? [y/n]\n>>> ')
#             if confirmation == 'y':
#                 break
#             else:
#                 continue
# except KeyboardInterrupt:
#     fWrite = open('lastPos.txt', 'w')
#     fWrite.write(f'{pointing[1]} {pointing[2]} {absoluteStepperState[0]}')
#     fWrite.close()

# ===== Main loop auto control =====
try:
    cleanup(motors)
    home()
    trackSun()
        
except KeyboardInterrupt:
    fWrite = open('lastPos.txt', 'w')
    fWrite.write(f'{pointing[1]} {pointing[2]} {absoluteStepperState[0]}')
    fWrite.close()