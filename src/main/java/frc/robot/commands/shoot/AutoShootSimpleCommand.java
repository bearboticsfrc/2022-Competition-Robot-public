// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The AutoShootSimpleCommand will shoot to the high hub, assumes that the shooter is aimed and the
 * targetRPM is set from the targetDrive command. No fancy logic, just waits 3 seconds to feed the
 * cargo.
 */
public class AutoShootSimpleCommand extends SequentialCommandGroup {
  public AutoShootSimpleCommand(
      ShooterSubsystem shooter, HopperSubsystem hopper, LimelightSubsystem limelight) {
    addCommands(
        new WaitUntilCommand(shooter::atTargetVelocity).withTimeout(3.0),
        new LogCommand("Started feeding shooter"),
        new InstantCommand(hopper::feedShooter),
        new WaitCommand(3.0),
        new LogCommand("Stopping shooter and feeder"),
        new InstantCommand(shooter::stop),
        new InstantCommand(hopper::stopFeed));

    addRequirements(shooter, hopper);
  }
}
