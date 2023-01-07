// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.TargetDrive;
import frc.robot.commands.shoot.AutoShootForTwoBallAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBallAutoCommand extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoCommand. */
  public TwoBallAutoCommand(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight,
      String pathName,
      String autoName) {

    PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath(pathName, 1, 2);

    Command pathCommand = new PathCommand(driveSubsystem, pathPlannerTrajectory);

    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    setName(autoName);

    addCommands(
        new BeginAutoCommand(
            driveSubsystem, hopperSubsystem, pathPlannerTrajectory.getInitialState(), autoName),
        pathCommand,
        new InstantCommand(() -> hopperSubsystem.retractIntake()),
        new InstantCommand(() -> driveSubsystem.stop()),
        new WaitCommand(1.4),
        new LogCommand("Starting shoot cycle"),
        new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
        new InstantCommand(hopperSubsystem::reverseVertical)
            .andThen(
                new WaitCommand(0.1).andThen(new InstantCommand(hopperSubsystem::stopVertical))),
        new ParallelRaceGroup(
            targetDrive.withTimeout(5.0),
            new SequentialCommandGroup(
                new WaitUntilCommand(
                    () -> {
                      return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                    }),
                new AutoShootForTwoBallAutoCommand(shooterSubsystem, hopperSubsystem, limeLight),
                new LogCommand("Done shooting"))),
        new LogCommand("Ending shoot cycle"),
        new InstantCommand(
            () -> {
              hopperSubsystem.setIntakeSensor(true);
              hopperSubsystem.stopBallOne();
              hopperSubsystem.stopIntake();
              driveSubsystem.stop();
              driveSubsystem.headingOffest(
                  pathPlannerTrajectory.getInitialState().holonomicRotation.getDegrees());
            }));
  }
}
