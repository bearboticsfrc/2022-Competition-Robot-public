// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class TargetDrive extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private LimelightSubsystem limeLight;
  private ShooterSubsystem shooter;
  private double ShooterOffsetDistance = 0;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private double degreesToTurn = 0.0;

  private double defaultRPM = 0.0;

  private boolean m_firstCycle = true;
  ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  // private AutoShootCommand m_command;

  private final PIDController m_turnPIDController =
      new PIDController(
          AutoConstants.kPTargetTurn, AutoConstants.kITargetTurn, AutoConstants.kDTargetTurn);

  /** Creates a new TargetDrive. */
  public TargetDrive(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limeLight,
      ShooterSubsystem shooter,
      DoubleSupplier ySupplier,
      DoubleSupplier xSupplier) {

    this.driveSubsystem = driveSubsystem;
    this.limeLight = limeLight;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    defaultRPM = 0.0;

    addRequirements(driveSubsystem);
  }

  /** Creates a new TargetDrive. */
  public TargetDrive(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limeLight,
      ShooterSubsystem shooter,
      DoubleSupplier ySupplier,
      DoubleSupplier xSupplier,
      double defaultRPM) {

    this.driveSubsystem = driveSubsystem;
    this.limeLight = limeLight;
    this.shooter = shooter;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.defaultRPM = defaultRPM;

    addRequirements(driveSubsystem);
  }

  /** Creates a new TargetDrive. */
  public TargetDrive(
      DriveSubsystem driveSubsystem, LimelightSubsystem limeLight, ShooterSubsystem shooter) {

    this.driveSubsystem = driveSubsystem;
    this.limeLight = limeLight;
    this.shooter = shooter;
    xSupplier =
        () -> {
          return 0.0;
        };
    ySupplier =
        () -> {
          return 0.0;
        };
    defaultRPM = 0.0;

    addRequirements(driveSubsystem);
  }

  /** Creates a new TargetDrive. */
  public TargetDrive(
      DriveSubsystem driveSubsystem,
      LimelightSubsystem limeLight,
      ShooterSubsystem shooter,
      double defaultRPM) {

    this.driveSubsystem = driveSubsystem;
    this.limeLight = limeLight;
    this.shooter = shooter;
    xSupplier =
        () -> {
          return 0.0;
        };
    ySupplier =
        () -> {
          return 0.0;
        };
    this.defaultRPM = defaultRPM;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLight.enableLED();
    m_firstCycle = true;
  }

  public double getDegreesToTurn() {
    return degreesToTurn;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    degreesToTurn =
        limeLight.getX()
            + (Math.atan(ShooterOffsetDistance / (limeLight.getDistance() + 24)) * 180 / Math.PI);

    degreesToTurn = (Math.abs(degreesToTurn) < 0.5) ? 0.0 : degreesToTurn;

    double setPoint = driveSubsystem.getHeading() + degreesToTurn;

    double turnOutput = m_turnPIDController.calculate(driveSubsystem.getHeading(), setPoint);

    double distance = limeLight.getDistance();
    if (distance == 0.0) distance = 25;

    double targetRPM = (defaultRPM != 0.0 ? defaultRPM : shooter.getRPM(distance));
    shooter.setTargetRPM(targetRPM);
    if (m_firstCycle) {
      DataLogManager.log("TargetDrive: distance=" + distance + " targetRPM=" + targetRPM);
      m_firstCycle = false;
    }

    driveSubsystem.drive(ySupplier.getAsDouble(), xSupplier.getAsDouble(), -turnOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limeLight.disableLED();
    shooter.setTargetRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
