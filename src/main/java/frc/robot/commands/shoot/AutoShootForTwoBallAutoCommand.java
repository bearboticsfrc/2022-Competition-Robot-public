// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The AutoShootCommand will shoot to the high hub, assumes that the shooter is aimed and the
 * targetRPM is set from the targetDrive command.
 */
public class AutoShootForTwoBallAutoCommand extends SequentialCommandGroup {
  public AutoShootForTwoBallAutoCommand(
      ShooterSubsystem shooter, HopperSubsystem hopper, LimelightSubsystem limelight) {
    addCommands(
        new InstantCommand(() -> hopper.setIntakeSensor(true)),
        new WaitUntilCommand(shooter::atTargetVelocity).withTimeout(3.0),
        new InstantCommand(hopper::feedShooter),
        new LogCommand("Started feeding shooter"),
        new WaitCommand(0.1),
        new WaitUntilCommand(
                () -> {
                  return shooter.getTargetRPM() - shooter.getVelocity() > 200;
                })
            .withTimeout(2.0),
        new LogCommand(
            String.format(
                "After RPM Dropped %s / %s vertSensor = %s ",
                shooter.getVelocity(), shooter.getTargetRPM(), hopper.getVerticalSensor())),
        // new InstantCommand(hopper::stopFeed),
        new WaitCommand(0.1),
        new ConditionalCommand(
            new InstantCommand(),
            new SequentialCommandGroup(
                new LogCommand("Shooting second cargo"),
                new WaitUntilCommand(shooter::atTargetVelocity).withTimeout(2.0),
                new InstantCommand(hopper::feedShooter),
                new WaitUntilCommand(
                        () -> {
                          return Math.abs(shooter.getVelocity() - shooter.getTargetRPM()) > 200;
                        })
                    .withTimeout(1.0)),
            hopper::getVerticalSensor),
        new WaitUntilCommand(
                () -> {
                  return hopper.getVerticalSensor() && hopper.getHopperSensor();
                })
            .withTimeout(2.0),
        new LogCommand("Stopping shooter and feeder"),
        new SequentialCommandGroup(
            new InstantCommand(shooter::stop), new InstantCommand(hopper::stopFeed)));

    addRequirements(shooter, hopper);
  }
}
