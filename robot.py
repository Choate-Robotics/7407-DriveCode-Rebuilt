#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import wpilib
from wpilib import DriverStation
import commands2
from ntcore import NetworkTableInstance
from autos import AutoRoutine

from robotcontainer import RobotContainer
from utils.alliance_flip_util import get_alliance
from sensors import FieldOdometry
from utils import shooter_utils


class MyRobot(wpilib.TimedRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        self.robot = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()

        self.nt_inst = NetworkTableInstance.getDefault()
        self.time_table = self.nt_inst.getTable("Timing")
        self.time_pub = self.time_table.getDoubleTopic("Loop time").publish()
        self.time = 0

        if wpilib.RobotBase.isSimulation():
            DriverStation.silenceJoystickConnectionWarning(True)

        self.robot.telemetrize_drivetrain()
        self.distance_pub = self.nt_inst.getTable("Shot Tuner").getDoubleTopic("distance to hub").publish()

        # self.robot.field_odometry.disable()

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        self.scheduler.run()

        current_time = wpilib.Timer.getFPGATimestamp()
        self.time_pub.set(current_time - self.time)
        self.time = current_time


        self.robot.field_odometry.update()
        
        self.distance_pub.set(shooter_utils.shot_distance_from_pose(self.robot.drivetrain.get_pose()))
        

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        auto: AutoRoutine = self.robot.getAutonomousCommand()(self.robot)
        starting_pose = get_alliance(auto.start_pose)
        self.robot.drivetrain.reset_pose(starting_pose)
        self.scheduler.schedule(commands2.SequentialCommandGroup(
            commands2.InstantCommand(lambda: self.robot.drivetrain.seed_field_centric(get_alliance(starting_pose.rotation()))),
            auto.command,
        ))
        
    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def autonomousExit(self):
        self.scheduler.cancelAll()

    def teleopInit(self) -> None:
        self.robot.configureButtonBindings()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        self.scheduler.cancelAll()
