// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;

/** Command sequence to extend the climber to the next rung */
public class ClimbCommand extends SequentialCommandGroup {
  /** Creates a new ClimbCommand. */
  public ClimbCommand(ClimberSubsystem climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // elevator Up some
        new InstantCommand(() -> climber.extendBeforeRotate()),
        new WaitUntilCommand(climber::atSetPoint),

        // rotate out
        new InstantCommand(() -> climber.extendTraverse()),
        // wait for x seconds to make sure the pistons are at full extend
        new WaitCommand(3.0), // was 1.4
        // elevator Up to full extension
        new InstantCommand(() -> climber.extendToBar()),
        new WaitUntilCommand(climber::atSetPoint),

        // rotate in
        new InstantCommand(() -> climber.retractTraverse()));
    addRequirements(climber);
  }
}
