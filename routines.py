import time
from constants import *
# https://github.com/Chr157i4n/TMC2209_Raspberry_Pi/tree/main
try:
    from src.TMC_2209.TMC_2209_StepperDriver import *
    from src.TMC_2209._TMC_2209_GPIO_board import Board
except ModuleNotFoundError:
    from TMC_2209.TMC_2209_StepperDriver import *
    from TMC_2209._TMC_2209_GPIO_board import Board

def setupTMC(tmc):
    """ initializes the settings in the register of the TMC driver

    Args:
        tmc (TMC_2209): TMC object from the TMC_2209 stepper driver library
    """
    # set the loglevel of the libary (currently only printed)
    # set whether the movement should be relative or absolute
    # both optional
    tmc.tmc_logger.set_loglevel(Loglevel.DEBUG)
    tmc.set_movement_abs_rel(MovementAbsRel.ABSOLUTE)

    # these functions change settings in the TMC register
    tmc.set_direction_reg(False)
    tmc.set_current(MAX_CURRENT)
    tmc.set_interpolation(True)
    tmc.set_spreadcycle(False)
    tmc.set_microstepping_resolution(MICROSTEPS)
    tmc.set_internal_rsense(False)
    tmc.set_motor_enabled(True)

    tmc.set_acceleration_fullstep(MAX_ACCEL)
    tmc.set_max_speed_fullstep(MAX_SPEED)

def moveStepper(tmc, steps):
    """ all the stepper movements are controlled here

    Args:
        tmc (TMC_2209): TMC driver object
        steps (int): positive or negative value. steps, including microsteps, to move the stepper 
    """
    tmc.set_motor_enabled(True)
    tmc.run_to_position_steps(steps, MovementAbsRel.RELATIVE)
    cleanup(tmc)

def cleanup(tmc):
    """ pulls the enable pin high to disable the driver output. This is the only safe way to power off the motor. Sudden loss of power can damage the driver

    Args:
        tmc (TMC_2209): TMC driver object
    """
    tmc.set_motor_enabled(False)
