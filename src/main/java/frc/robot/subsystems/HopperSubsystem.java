// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.RevUtil.checkRevError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Robot;
import frc.robot.util.RevUtil;

/** The Hopper is all the motors and sensors to pick up the ball and move it to the shooter */
public class HopperSubsystem extends SubsystemBase {
  // motors
  private final CANSparkMax intakemotor =
      new CANSparkMax(HopperConstants.kIntakeMotorPort, MotorType.kBrushless);
  private final CANSparkMax hoppermotor =
      new CANSparkMax(HopperConstants.kHopperMotorPort, MotorType.kBrushless);
  private final CANSparkMax vertmotor =
      new CANSparkMax(HopperConstants.kVerticalMotorPort, MotorType.kBrushless);
  private RelativeEncoder intakemotorEncoder = intakemotor.getEncoder();
  private RelativeEncoder hoppermotorEncoder = hoppermotor.getEncoder();
  private RelativeEncoder vertmotorEncoder = vertmotor.getEncoder();

  // sensors
  private DigitalInput hopperlinebreaksensor =
      new DigitalInput(HopperConstants.kHopperLineBreakSensor);
  private DigitalInput verticallinebreaksensor =
      new DigitalInput(HopperConstants.kVerticalLineBreakSensor);

  private final Solenoid intakeSolenoid;
  private final PneumaticHub pneumatichub;
  private final BooleanLogEntry intakeStateLog;

  // booleans
  public boolean isBallOneRunning = false;
  public boolean isBallTwoRunning = false;

  private boolean intakeSensorValue = true;

  private BooleanLogEntry intakeSensorLog;
  private BooleanLogEntry hopperSensorLog;
  private BooleanLogEntry verticalSensorLog;
  private DoubleLogEntry intakemotorLog;
  private DoubleLogEntry hoppermotorLog;
  private DoubleLogEntry vertmotorLog;

  /**
   * Creates a new HopperSubsystem.
   *
   * @param Shuffleboard
   */
  public HopperSubsystem(PneumaticHub pneumaticHub) {
    DataLog log = DataLogManager.getLog();
    intakeSensorLog = new BooleanLogEntry(log, "/hopper/intake_sensor");
    hopperSensorLog = new BooleanLogEntry(log, "/hopper/hopper_sensor");
    verticalSensorLog = new BooleanLogEntry(log, "/hopper/vertical_sensor");
    intakemotorLog = new DoubleLogEntry(log, "/hopper/intake_velocity");
    hoppermotorLog = new DoubleLogEntry(log, "/hopper/hopper_velocity");
    vertmotorLog = new DoubleLogEntry(log, "/hopper/vertical_velocity");
    intakeStateLog = new BooleanLogEntry(log, "/hopper/extended");
    pneumatichub = pneumaticHub;
    intakeSolenoid = pneumatichub.makeSolenoid(HopperConstants.kIntakeChannel);

    RevUtil.setPeriodicFramePeriodLow(intakemotor, "intake motor");
    RevUtil.setPeriodicFramePeriodLow(hoppermotor, "hopper motor");
    RevUtil.setPeriodicFramePeriodLow(vertmotor, "vertical motor");

    checkRevError(
        intakemotor.setIdleMode(IdleMode.kBrake), "Failed to set idle mode for intake motor.");
    checkRevError(
        hoppermotor.setIdleMode(IdleMode.kBrake), "Failed to set idle mode for hopper motor.");
    checkRevError(
        vertmotor.setIdleMode(IdleMode.kBrake), "Failed to set idle mode for vert motor.");

    stopBallOne();
    StopBallTwo();
  }

  private boolean testMode = false;

  public int ballCount() {

    int ballCount = 0;

    if (!getVerticalSensor()) {
      ballCount++;
    }
    if (!getHopperSensor()) {
      ballCount++;
    }
    return ballCount;
  }

  @Override
  public void periodic() {
    // System.out.println("vertical sensor="+getVerticalSensor());
    if (testMode) return;
    if (!Robot.isRobotEnabled()) return;

    // This method will be called once per scheduler run
    if (!getIntakeSensor() && getVerticalSensor()) {
      startBallOne();
    }
    if (!getVerticalSensor() && isBallOneRunning) {
      stopBallOne();
    }
    if (!getIntakeSensor() && !getVerticalSensor() && getHopperSensor()) {
      startBallTwo();
    }
    if (!getHopperSensor() && isBallTwoRunning) {
      StopBallTwo();
    }

    intakeSensorLog.append(getIntakeSensor());
    hopperSensorLog.append(getHopperSensor());
    verticalSensorLog.append(getVerticalSensor());
    intakemotorLog.append(intakemotorEncoder.getVelocity());
    hoppermotorLog.append(hoppermotorEncoder.getVelocity());
    vertmotorLog.append(vertmotorEncoder.getVelocity());
    intakeStateLog.append(intakeSolenoid.get());
  }

  public void startIntake() {
    intakemotor.set(HopperConstants.kIntakeSpeed);
  }

  public void reverseIntake() {
    intakemotor.set(-0.9);
  }

  public void stopIntake() {
    intakemotor.stopMotor();
  }

  public void extendIntake() {
    intakeSolenoid.set(true);
  }

  public void retractIntake() {
    intakeSolenoid.set(false);
  }

  public void reverseVertical() {
    vertmotor.set(-0.3);
  }

  public void stopVertical() {
    vertmotor.set(0.0);
  }

  public void startBallOne() {
    DataLogManager.log("startBallOne");
    hoppermotor.set(HopperConstants.kHopperSpeed);
    if (Robot.inAuto()) {
      DataLogManager.log("Vertical Motor set to " + HopperConstants.kAutoVert);
      vertmotor.set(HopperConstants.kAutoVert);
    } else {
      DataLogManager.log("Vertical Motor set to " + HopperConstants.kVerticalSpeed);
      vertmotor.set(HopperConstants.kVerticalSpeed);
    }
    isBallOneRunning = true;
  }

  public void stopBallOne() {
    DataLogManager.log("stopBallOne");
    hoppermotor.set(0.0);
    DataLogManager.log("Vertical Motor stop");
    vertmotor.set(0.0);
    isBallOneRunning = false;
  }

  public void startBallTwo() {
    DataLogManager.log("startBallTwo");
    hoppermotor.set(HopperConstants.kHopperSpeed);
    isBallTwoRunning = true;
  }

  public void StopBallTwo() {
    DataLogManager.log("StopBallTwo");
    hoppermotor.set(0.0);
    isBallTwoRunning = false;
  }

  public void feedShooter() {
    DataLogManager.log("Vertical Motor set to " + HopperConstants.kFeedSpeed);
    vertmotor.set(HopperConstants.kFeedSpeed);
    hoppermotor.set(HopperConstants.kHopperSpeed);
  }

  public void feedLow() {
    DataLogManager.log("Vertical Motor set to " + HopperConstants.kLowFeed);
    vertmotor.set(HopperConstants.kLowFeed);
    hoppermotor.set(-HopperConstants.kLowFeed);
  }

  public void stopFeed() {
    DataLogManager.log("Vertical Motor set to 0");
    vertmotor.set(0.0);
    hoppermotor.set(0.0);
  }

  public boolean getIntakeSensor() {
    return intakeSensorValue;
  }

  public boolean getHopperSensor() {
    return hopperlinebreaksensor.get();
  }

  public boolean getVerticalSensor() {
    return verticallinebreaksensor.get();
  }

  public void setIntakeSensor(boolean value) {
    intakeSensorValue = value;
  }
}
