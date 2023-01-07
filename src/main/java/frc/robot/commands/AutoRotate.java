// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** Command to rotate the robot by a given number of degrees */
public class AutoRotate extends CommandBase {
  private DriveSubsystem driveSubsystem;

  private final PIDController turnPIDController = new PIDController(0.005, .00001, 0);

  private double offset = 0.0;

  private double turnDegrees = 0.0;

  private double setPoint = 0.0;

  private boolean kDebug = false;

  public AutoRotate(DriveSubsystem driveSubsystem, double turnDegrees) {
    this.driveSubsystem = driveSubsystem;
    this.turnDegrees = turnDegrees;

    if (kDebug) {
      ShuffleboardTab tab = Shuffleboard.getTab("Auto Rotate");
      tab.addNumber("SetPoint", () -> this.getSetPoint());
      tab.addNumber("Error", () -> this.getError());
      // tab.addNumber("Degrees Off", () -> {
      //  return driveSubsystem.getHeading() - this.getSetPoint();
      // });
      tab.addNumber(
          "Heading",
          () -> {
            return driveSubsystem.getHeading();
          });
    }
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    offset = driveSubsystem.getHeading();
    setPoint = getSetPoint();
  }

  public double getSetPoint() {
    double temp = turnDegrees + offset;
    temp -= Math.floor(temp / 360.0) * 360.0;
    return temp;
  }

  public double getError() {
    return turnPIDController.calculate(driveSubsystem.getHeading(), getSetPoint());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnOutput = getError();

    driveSubsystem.drive(0.0, 0.0, turnOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getHeading() - setPoint) < 2.0;
  }
}
