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
import frc.robot.subsystems.ShooterSubsystem;

/** Command sequence to shoot one cycle */
public class SimpleShootCommand extends SequentialCommandGroup {

  public SimpleShootCommand(ShooterSubsystem shooter, HopperSubsystem hopper, double shootrpm) {
    addCommands(
        new InstantCommand(hopper::reverseVertical)
            .andThen(new WaitCommand(0.1).andThen(new InstantCommand(hopper::stopVertical))),
        new InstantCommand(() -> shooter.setTargetRPM(shootrpm)),
        new WaitUntilCommand(shooter::atTargetVelocity).withTimeout(3.0),
        new InstantCommand(hopper::feedLow),
        new LogCommand("Started feeding shooter"),
        new WaitCommand(1.0),
        // feed motor is still running, so wait until both line break sensors are clear (or a
        // second)
        new WaitUntilCommand(
                () -> {
                  return hopper.getVerticalSensor() && hopper.getHopperSensor();
                })
            .withTimeout(1.0),
        new LogCommand("Stopping shooter and feeder"),
        new InstantCommand(shooter::stop),
        new InstantCommand(hopper::stopFeed));
    addRequirements(shooter, hopper);
  }
}
