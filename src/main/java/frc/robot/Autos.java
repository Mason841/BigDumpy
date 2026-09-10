package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SubsystemManager.RobotState;

public class Autos {

    private final AutoFactory autoFactory;

    private final SubsystemManager subsystemManager;

    public Autos(AutoFactory autoFactory, SubsystemManager subsystemManager) {

        this.autoFactory = autoFactory;

        this.subsystemManager = subsystemManager;
    }

    public AutoRoutine LT_DoubleSweepReturn() {
        
        AutoRoutine routine = autoFactory.newRoutine("LT_DoubleSweepReturn");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1");
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2");
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3");

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RT_DoubleSweepReturn() {
        
        AutoRoutine routine = autoFactory.newRoutine("RT_DoubleSweepReturn");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1").mirrorY();
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2").mirrorY();
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3").mirrorY();

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager),
                path3.cmd()
            )
        );

        return routine;
    }

    private void assignStandardCommandsToTrajectory(AutoTrajectory path) {
        path.atTime("Intake").onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager));
        path.atTime("IntakeStop").onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager));
    }
}
