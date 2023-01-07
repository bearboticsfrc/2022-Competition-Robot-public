// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.RevUtil.checkRevError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimberSettings;
import frc.robot.Constants.ClimberConstants;

/** The motors, sensors, and pneumatics for the climber mechanism */
public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax leftclimbermotor =
      new CANSparkMax(ClimberConstants.kLeftClimbMotorPort, MotorType.kBrushless);
  private final CANSparkMax rightclimbermotor =
      new CANSparkMax(ClimberConstants.kRightClimbMotorPort, MotorType.kBrushless);

  private final Solenoid climbsolenoid;
  private final PneumaticHub pneumatichub;

  private SparkMaxPIDController climbpidcontroller;

  private double climbsetpoint;

  private RelativeEncoder climbencoder;

  private double kP = 0.4;
  private double kI = 0.0; // 0.00001;
  private double kD = 0.0;
  private double kIz = 0.0;
  private double kFF = 0.0;
  private double kMaxOutput = 0.60;
  private double kMinOutput = -0.60;

  private DoubleLogEntry positionLog;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(PneumaticHub pneumaticHub) {
    DataLog log = DataLogManager.getLog();
    positionLog = new DoubleLogEntry(log, "/climber/position");

    pneumatichub = pneumaticHub;

    climbsolenoid = pneumatichub.makeSolenoid(ClimberConstants.kCLimbSolenoidPort);

    climbsolenoid.set(false);

    climbencoder = leftclimbermotor.getEncoder();
    climbpidcontroller = leftclimbermotor.getPIDController();

    climbpidcontroller.setP(kP);
    climbpidcontroller.setI(kI);
    climbpidcontroller.setD(kD);
    climbpidcontroller.setIZone(kIz);
    climbpidcontroller.setFF(kFF);
    climbpidcontroller.setOutputRange(kMinOutput, kMaxOutput);
    ShuffleboardTab tab = Shuffleboard.getTab("Climb");
    tab.addNumber("left climber current", leftclimbermotor::getOutputCurrent);
    tab.addNumber("right climber current", rightclimbermotor::getOutputCurrent);
    tab.addNumber("climber", this::getClimbSetPoint);
    tab.addNumber("position", this::getPosition);
    tab.add(
        "Extend",
        new InstantCommand(() -> this.setClimberPosition(ClimberSettings.getManualPosition())));

    checkRevError(
        rightclimbermotor.setIdleMode(IdleMode.kBrake),
        "Error setting idle mode for right climber motor");
    checkRevError(
        leftclimbermotor.setIdleMode(IdleMode.kBrake),
        "Error setting idle mode for left climber motor");
    checkRevError(
        rightclimbermotor.follow(leftclimbermotor, true),
        "Error setting follower for left climber motor");

    checkRevError(
        rightclimbermotor.setSmartCurrentLimit(80),
        "Failed to set current limit for right climber");
    checkRevError(
        leftclimbermotor.setSmartCurrentLimit(80), "Failed to set current limit for left climber");

    checkRevError(
        rightclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20),
        "Failed to set periodic status frame 0 rate for right climber motor");
    checkRevError(
        rightclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate for right climber motor");
    checkRevError(
        rightclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100),
        "Failed to set periodic status frame 2 rate for right climber motor");
    checkRevError(
        rightclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100),
        "Failed to set periodic status frame 3 rate for right climber motor");

    checkRevError(
        leftclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10),
        "Failed to set periodic status frame 0 rate for left climber motor");
    checkRevError(
        leftclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate for left climber motor");
    checkRevError(
        leftclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10),
        "Failed to set periodic status frame 2 rate for left climber motor");
    checkRevError(
        leftclimbermotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100),
        "Failed to set periodic status frame 3 rate for left climber motor");
  }

  public void extendElevator(double rate) {
    rate = MathUtil.clamp(rate, -0.8, 0.8);

    if (getPosition() >= ClimberConstants.kMaxHeight
        && getPosition() <= ClimberConstants.kMinHeight + 0.3) {
      // rightclimbermotor.set(rate);
      leftclimbermotor.set(rate);
    } else {
      // rightclimbermotor.set(0.0);
      leftclimbermotor.set(0.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    positionLog.append(getPosition());
  }

  public double getClimbSetPoint() {
    return climbsetpoint;
  }

  public void setClimberPosition(double position) {
    climbsetpoint =
        MathUtil.clamp(position, ClimberConstants.kMaxHeight, ClimberConstants.kMinHeight);
    climbpidcontroller.setReference(climbsetpoint, CANSparkMax.ControlType.kPosition);
  }

  public void extendClimberMid() {
    climbpidcontroller.setOutputRange(kMinOutput, kMaxOutput);

    setClimberPosition(ClimberConstants.kMidClimb);
  }

  public void extendClimberLow() {
    setClimberPosition(ClimberConstants.kLowBar);
  }

  public void retractClimber() {
    setClimberPosition(ClimberConstants.kMinHeight);
  }

  public void extendToBar() {
    setClimberPosition(ClimberConstants.kMaxHeight);
  }

  public void extendBeforeRotate() {
    setClimberPosition(ClimberConstants.kBeforeExtendHeight);
  }

  public void retractOnBar() {
    setClimberPosition(ClimberConstants.kOnNextBar);
  }

  public double getPosition() {
    return climbencoder.getPosition();
  }

  public boolean atSetPoint() {
    return Math.abs(getPosition() - climbsetpoint) < 0.5;
  }

  public void extendTraverse() {
    climbsolenoid.set(true);
  }

  public void retractTraverse() {
    climbsolenoid.set(false);
  }
}
