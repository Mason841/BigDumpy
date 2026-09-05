// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRack;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelights;

public class Robot extends LoggedRobot {

    private final Drivetrain drivetrain;
    private final Hood hood;
    private final Indexer indexer;
    private final IntakeRack intakeRack;
    private final IntakeRoller intakeRoller;
    private final RollerFloor rollerFloor;
    private final Shooter shooter;

    private final Autoaim autoaim;

    private final VisionIO visionIO;
    private final Vision vision;

    private final SubsystemManager subsystemManager;

    private final RobotContainer robotContainer;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    private CommandXboxController joystick = new CommandXboxController(0);

    public Robot() {

        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher()); 

        DriverStation.silenceJoystickConnectionWarning(true);

        SignalLogger.enableAutoLogging(false);

        drivetrain = new Drivetrain(
            joystick,
            TunerConstants.DrivetrainConstants, 
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight);

        autoaim = new Autoaim(drivetrain);

        hood = new Hood(autoaim::getHoodPositionToTargetWhileMoving);
        indexer = new Indexer();
        intakeRack = new IntakeRack();
        intakeRoller = new IntakeRoller();
        rollerFloor = new RollerFloor();
        shooter = new Shooter(autoaim::getShooterSpeedToTargetWhileMoving);

        this.visionIO = new VisionIOLimelights();
        this.vision = new Vision(visionIO, drivetrain);

        subsystemManager = new SubsystemManager(autoaim, drivetrain, hood, indexer, intakeRack, intakeRoller, rollerFloor, shooter);

        robotContainer = new RobotContainer(joystick, autoaim, subsystemManager, drivetrain, hood, indexer, intakeRack, intakeRoller, rollerFloor, shooter);

        this.autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );
        
        autoChooser = new AutoChooser();
        
        SmartDashboard.putData("AutoChooser", autoChooser);

        Threads.setCurrentThreadPriority(true, 5);

        Logger.start();

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 

        var latestShiftInfo = HubShiftUtil.getOfficialShiftInfo();
        Logger.recordOutput("HubShift/Official", latestShiftInfo);
        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());

        SmartDashboard.putString("Current Shift", latestShiftInfo.currentShift().toString());
        // SmartDashboard.putNumber("Elapsed Time", latestShiftInfo.elapsedTime());
        SmartDashboard.putNumber("Remaining Time", latestShiftInfo.remainingTime());
        SmartDashboard.putBoolean("Shift Active", latestShiftInfo.active());
    }

    @Override
    public void disabledInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        HubShiftUtil.initialize();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
