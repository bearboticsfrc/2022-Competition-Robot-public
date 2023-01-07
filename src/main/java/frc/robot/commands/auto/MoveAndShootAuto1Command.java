// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.TargetDrive;
import frc.robot.commands.shoot.AutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Move off the line and shoot the preloaded ball. NO intake extension or pickup.
 *
 * @param driveSubsystem
 * @param hopperSubsystem
 * @param shooterSubsystem
 * @param limelightSubsystem
 * @return
 */
public class MoveAndShootAuto1Command extends SequentialCommandGroup {
  /** Creates a new MoveAndShootAuto1Command. */
  public MoveAndShootAuto1Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {

    PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("auto1", 1, 2);
    Command pathCommand = new PathCommand(driveSubsystem, pathPlannerTrajectory);
    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    addCommands(
        new InstantCommand(
            () -> driveSubsystem.resetOdometry(pathPlannerTrajectory.getInitialState())),
        pathCommand,
        new ParallelCommandGroup(
            targetDrive.withTimeout(5),
            new SequentialCommandGroup(
                new WaitUntilCommand(
                    () -> {
                      return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                    }),
                new AutoShootCommand(shooterSubsystem, hopperSubsystem, limeLight))),
        new InstantCommand(() -> hopperSubsystem.stopBallOne()),
        new InstantCommand(() -> driveSubsystem.stop()),
        new InstantCommand(
            () ->
                driveSubsystem.headingOffest(
                    pathPlannerTrajectory.getInitialState().holonomicRotation.getDegrees())));
    setName("Auto1");
  }
}
