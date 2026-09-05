package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRack;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemManager;

public class Autos {

    private final AutoFactory autoFactory;

    private final SubsystemManager subsystemManager;

    private final Autoaim autoaim;

    private final Drivetrain drivetrain;
    private final Hood hood;
    private final Indexer indexer;
    private final IntakeRack intakeRack;
    private final IntakeRoller intakeRoller;
    private final RollerFloor rollerFloor;
    private final Shooter shooter;

    public Autos(AutoFactory autoFactory, SubsystemManager subsystemManager, Autoaim autoaim,
            Drivetrain drivetrain, Hood hood, Indexer indexer, IntakeRack intakeRack, IntakeRoller intakeRoller,
            RollerFloor rollerFloor, Shooter shooter) {

        this.autoFactory = autoFactory;

        this.subsystemManager = subsystemManager;

        this.autoaim = autoaim;

        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.intakeRack = intakeRack;
        this.intakeRoller = intakeRoller;
        this.rollerFloor = rollerFloor;
        this.shooter = shooter;
    }

}
