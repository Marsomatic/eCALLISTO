from math import pi

# a.k.a. the transmission ratio SM - output shaft
STEPS_PER_DEG = 3450

# Inverse of the above, used to update the pointing after every step of a SM
DEG_PER_STEP = 1/STEPS_PER_DEG

# Used to convert angles from radians to degrees
RAD_TO_DEG_FACTOR = 180 / pi

# Time in seconds that the program sleeps for after every step of a SM (probably optional)
SLEEP_TIME = 0.001

# Rotation rate of the earth in degrees/second
DEG_PER_SECOND = 1/240

# Frequency (in seconds) of updating the position in console
PRINT_FREQ = 1

# Absolute RA SM position when in home position
HA_HOME_ABS_POSITION = 172_500

# Hour angle when in home position
HOME_HA = -90

# Absolute Dec SM position when in home position
DEC_HOME_ABS_POSITION = 345_000

# Declination when in home positon
HOME_DEC = -45

# menu string
MENU_STRING = 'eCALLISTO Master v0.1\nt = track sun\nh = home\ngoto = GoTo\nm = manual control (RA and Dec)\n>>>'