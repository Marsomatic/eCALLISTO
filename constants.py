from math import pi

# Location of the antenna and altitude above sea level
LAT = 45.276055
LON = 13.721878
ALTITUDE = 226

# control pins
enPin = 21
dirPin = 20
stepPin = 16
limit = 17

# microstepping
MICROSTEPS = 2

# FULL Steps in a rotation
STEPS_PER_ROT = 10000

# Transmission ratio stepsperrev/200
TRANSMISSION_RATIO = STEPS_PER_ROT / 200

# Steps of the SM per 1 degree rotation on the output shaft
STEPS_PER_DEG = int(STEPS_PER_ROT / 360)

# Inverse of the above, used to update the pointing after every step of a SM
DEG_PER_STEP = 1/STEPS_PER_DEG

# Used to convert angles from radians to degrees
RAD_TO_DEG_FACTOR = 180 / pi

# Time in seconds that the program sleeps for after every step of a SM (no sleep time causes the motor to skip steps)
SLEEP_TIME = 0.002

# Rotation rate of the earth in degrees/second
DEG_PER_SECOND = 1/240

# Frequency (in seconds) of updating the position in console
PRINT_FREQ = 5

# Absolute RA SM position when in home position
HA_HOME_ABS_POSITION = int(STEPS_PER_ROT * (3/4))

# Hour angle when in home position
HOME_HA = 270 + 2.73

# Absolute Dec SM position when in home position
DEC_HOME_ABS_POSITION = STEPS_PER_ROT / 2

# Declination when in home positon
HOME_DEC = -45

# Main menu output
MENU_STRING = '===== eCALLISTO Master v1.0 =====\nt = track sun\nh = home\ngoto = GoTo\nm = manual control (RA and Dec)\ncoords = print current coords\nclean = release motors\n>>> '

#TODO: turn these into lists
# Scheduler nominal start time hour and minute in local timezone
START_TIME_HOUR = 7
START_TIME_MINUTE = 30

# Scheduler nominal stop time hour and minute in local timezone
STOP_TIME_HOUR = 19
STOP_TIME_MINUTE = 30

# Time when callisto does a spectral overview hour:0..23. Each column is 1 OVS time
OVS_TIMEH = [3, 18]
OVS_TIMEM = [30, 15]

# max current in mA
MAX_CURRENT = 1000

# default max acceleration step/s2
MAX_ACCEL = 1000

# default max speed in steps/s (from experience, stepper skips steps above 1000)
MAX_SPEED = 1000
