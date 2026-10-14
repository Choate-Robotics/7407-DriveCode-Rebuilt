#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time

import wpilib
from wpilib import DriverStation, RobotController
import commands2
from ntcore import NetworkTableInstance
import wpinet
from autos import AutoRoutine

from robotcontainer import RobotContainer
from utils.alliance_flip_util import get_alliance
from utils import alliance_flip_util
from sensors import FieldOdometry
from utils import shooter_utils
from utils.local_logger import start_log_status
from utils.match_info import start_match_info


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
        RobotController.setBrownoutVoltage(6.0)

        self.robot = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()

        self.scheduler._watchdog.disable()

        self.nt_inst = NetworkTableInstance.getDefault()
        self.time_table = self.nt_inst.getTable("Timing")
        self.time_pub = self.time_table.getDoubleTopic("Loop time").publish()
        # Where robotPeriodic's time goes (ms), to find what spikes on the real robot.
        # "Scheduler" = subsystem periodics, button triggers and running commands.
        self.scheduler_ms_pub = self.time_table.getDoubleTopic("Scheduler ms").publish()
        self.odometry_ms_pub = self.time_table.getDoubleTopic("Field Odometry ms").publish()
        self.periodic_ms_pub = self.time_table.getDoubleTopic("robotPeriodic ms").publish()
        # How long autonomousInit holds up the first auto loop (pose/gyro resets, scheduling)
        self.auto_init_ms_pub = self.time_table.getDoubleTopic("autonomousInit ms").publish()
        self.time = 0

        if wpilib.RobotBase.isSimulation():
            DriverStation.silenceJoystickConnectionWarning(True)

        self.robot.telemetrize_drivetrain()

        # self.robot.field_odometry.disable()
        
        self.robot.configureButtonBindings()

        # Logging/ health on the dashboard (USB mounted, where logs go, still writing)
        start_log_status()

        # Match/ on the dashboard: hub active, shift, match and shift timers (FMS game data)
        start_match_info()

        # Serve deploy/ on port 5800 so Elastic's "Download layout from robot" gets
        # deploy/elastic-layout.json (the dashboard layout, versioned with the code)
        wpinet.WebServer.getInstance().start(5800, wpilib.getDeployDirectory())

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        t_start = time.perf_counter()

        # Per-loop reads shared by all the commands and triggers below
        alliance_flip_util.refresh()
        self.robot.refresh_driver_only_mode()

        t_sched = time.perf_counter()
        self.scheduler.run()
        t_sched_done = time.perf_counter()

        current_time = wpilib.Timer.getFPGATimestamp()
        self.time_pub.set(current_time - self.time)
        self.time = current_time

        t_odom = time.perf_counter()
        self.robot.field_odometry.update()
        t_odom_done = time.perf_counter()

        self.time_table.putBoolean("gyro broken", self.robot.drivetrain.gyro_broken)

        self.robot.device_health.update()

        self.scheduler_ms_pub.set((t_sched_done - t_sched) * 1000)
        self.odometry_ms_pub.set((t_odom_done - t_odom) * 1000)
        self.periodic_ms_pub.set((time.perf_counter() - t_start) * 1000)
        

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        t_start = time.perf_counter()
        auto = self.robot.getAutonomousCommand()
        starting_pose = get_alliance(auto.start_pose)
        self.robot.drivetrain.reset_pose(starting_pose)
        self.robot.drivetrain.seed_field_centric(get_alliance(starting_pose.rotation()))
        self.robot.backup_gyro.set_yaw(starting_pose.rotation().degrees())
        # Scheduled as-is: wrapping it in a new group here crashed the code the second time
        # the same auto was enabled (a command can only ever be added to one group).
        self.scheduler.schedule(auto.command)
        self.auto_init_ms_pub.set((time.perf_counter() - t_start) * 1000)
        
    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def autonomousExit(self):
        self.scheduler.cancelAll()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        self.scheduler.cancelAll()
