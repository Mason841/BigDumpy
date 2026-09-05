// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.SubsystemManager.RobotState;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick;
    public final CommandXboxController cojoystick = new CommandXboxController(1);

    public final Drivetrain drivetrain;
    public final Hood hood;
    public final Indexer indexer;
    public final IntakeRack intakeRack;
    public final IntakeRoller intakeRoller;
    public final RollerFloor rollerFloor;
    public final Shooter shooter;

    public final Autoaim autoaim;

    public final SubsystemManager subsystemManager;
    
    public enum RobotMode {
    }

    public RobotContainer(
        CommandXboxController joystick,
        Autoaim autoaim,
        SubsystemManager subsystemManager,
        Drivetrain drivetrain, 
        Hood hood, 
        Indexer indexer, 
        IntakeRack intakeRack, 
        IntakeRoller intakeRoller, 
        RollerFloor rollerFloor, 
        Shooter shooter) {

        this.joystick = joystick;

        this.autoaim = autoaim;

        this.subsystemManager = subsystemManager;

        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.intakeRack = intakeRack;
        this.intakeRoller = intakeRoller;
        this.rollerFloor = rollerFloor;
        this.shooter = shooter;
        
        configureBindings();
    }

    private void configureBindings() {    

        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager)
        );

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORINGMOVING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager)
        );

        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.leftBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_REVERSEINTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );
       
        // Zero drivebase relative to field
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Initialize/restart inactive/active shift information
        cojoystick.start().onTrue(new InstantCommand(HubShiftUtil::initialize));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
