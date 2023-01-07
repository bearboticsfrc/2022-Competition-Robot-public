// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ShooterSettings;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Command Sequence to shoot using the manual RPM in the shooter settings */
public class ManualShootCommand extends SequentialCommandGroup {

  public ManualShootCommand(
      ShooterSubsystem shooter, HopperSubsystem hopper, ShooterSettings shooterSettings) {
    addCommands(
        new InstantCommand(hopper::reverseVertical)
            .andThen(new WaitCommand(0.1).andThen(new InstantCommand(hopper::stopVertical))),
        new InstantCommand(() -> shooter.setTargetRPM(shooterSettings.getManualRPM())),
        new WaitCommand(1.0),
        new WaitUntilCommand(shooter::atTargetVelocity),
        new InstantCommand(hopper::feedShooter),
        new WaitCommand(3.0),
        new InstantCommand(shooter::stop),
        new InstantCommand(hopper::stopFeed));

    addRequirements(shooter, hopper);
  }
}
