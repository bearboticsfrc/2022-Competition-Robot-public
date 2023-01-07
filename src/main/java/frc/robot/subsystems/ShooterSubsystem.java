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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

/** ShooterSubsystem to encapsulate the flywheel control and sensor feedback */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotorOne;
  private final CANSparkMax shooterMotorTwo;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder shooterMotorEncoder;

  // PID coefficients
  // private final double kP = 0.000055; //0.0001; //0.00004; // was 0.00006;
  // private final double kI = 0.0000005;
  // private final double kD = 0.0;
  // private final double kIz = 0.0;
  // private final double kFF = 0.0001745; // 0.000182; // 0.00017618; // 1/5676

  private final double kP = 0.0001; // 0.0001; //0.00004; // was 0.00006;
  private final double kI = 0.0000007;
  private final double kD = 0.00000008;
  private final double kIz = 400;
  private final double kFF = 0.000168; // 0.000182; // 0.00017618; // 1/5676

  private final double kMaxOutput = 1;
  private final double kMinOutput = -1;
  private final double kMaxRPM = 5676;
  private double m_targetRPM;
  private double m_targetRPM_compensation = 0.0;

  private LinearFilter filter = LinearFilter.movingAverage(20);
  private double m_filteredVelocity = 0.0;

  // private Filter m_rpmFilter;

  private DoubleSupplier m_ySupplier;

  private DoubleLogEntry m_velocityLog;
  private DoubleLogEntry m_targetRPMLog;

  /** Creates a new Shooter. */
  public ShooterSubsystem(DoubleSupplier yAxisSupplier) {
    DataLog log = DataLogManager.getLog();
    m_velocityLog = new DoubleLogEntry(log, "/shooter/velocity");
    m_targetRPMLog = new DoubleLogEntry(log, "/shooter/targetRPM");

    shooterMotorOne = new CANSparkMax(ShooterConstants.kShooterMotorOnePort, MotorType.kBrushless);
    shooterMotorTwo = new CANSparkMax(ShooterConstants.kShooterMotorTwoPort, MotorType.kBrushless);
    shooterMotorEncoder = shooterMotorOne.getEncoder();

    checkRevError(
        shooterMotorOne.enableVoltageCompensation(12.0),
        "Could not enableVoltageCompensation for shooter motor one.");
    checkRevError(
        shooterMotorTwo.enableVoltageCompensation(12.0),
        "Could not enableVoltageCompensation for shooter motor two.");
    checkRevError(
        shooterMotorOne.setIdleMode(IdleMode.kCoast),
        "Could not set idle mode for shooter motor one.");
    checkRevError(
        shooterMotorTwo.setIdleMode(IdleMode.kCoast),
        "Could not set idle mode for shooter motor two.");

    checkRevError(
        shooterMotorTwo.follow(shooterMotorOne, true),
        "Could not set follower on shooter motor two.");

    checkRevError(
        shooterMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10),
        "Failed to set periodic status frame 0 rate");
    checkRevError(
        shooterMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10),
        "Failed to set periodic status frame 1 rate");
    checkRevError(
        shooterMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100),
        "Failed to set periodic status frame 2 rate");
    checkRevError(
        shooterMotorOne.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100),
        "Failed to set periodic status frame 3 rate");

    checkRevError(
        shooterMotorTwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20),
        "Failed to set periodic status frame 0 rate");
    checkRevError(
        shooterMotorTwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    checkRevError(
        shooterMotorTwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100),
        "Failed to set periodic status frame 2 rate");
    checkRevError(
        shooterMotorTwo.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100),
        "Failed to set periodic status frame 3 rate");

    pidController = shooterMotorOne.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    shooterMotorOne.setClosedLoopRampRate(0.005); // 0 to full in 5ms ??
    m_targetRPM = 0;

    m_ySupplier = yAxisSupplier;

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
    tab.addBoolean("At velocity", this::atTargetVelocity).withPosition(0, 2);
    tab.addNumber("Velocity", this::getVelocity).withPosition(1, 2);
    tab.addNumber("target rpm", this::getTargetRPM).withPosition(2, 2);
  }

  public void shoot() {
    // shoots
  }

  public void stop() {
    // do a .set(0) so it coasts and turns off pid?
    shooterMotorOne.set(0.0);
    m_targetRPM = 0;
  }

  @Override
  public void periodic() {
    m_filteredVelocity = filter.calculate(shooterMotorEncoder.getVelocity());
    m_targetRPM_compensation = m_ySupplier.getAsDouble();

    m_targetRPMLog.append(m_targetRPM);
    m_velocityLog.append(getVelocity());
  }

  public void setTargetRPM(double RPM) {
    if (RPM > kMaxRPM) DataLogManager.log("Clamping RPM[" + RPM + "] to [" + kMaxRPM + "]");
    RPM = MathUtil.clamp(RPM, 0.0, kMaxRPM);
    DataLogManager.log("@@@@@@@@@@@ TARGET RPM = " + RPM + " velocity = " + getVelocity());
    m_targetRPM = RPM + (m_targetRPM_compensation / 10.0 * RPM);
    pidController.setReference(m_targetRPM, CANSparkMax.ControlType.kVelocity);
  }

  public double getVelocity() {
    return m_filteredVelocity;
    // return shooterMotorEncoder.getVelocity();
  }

  public double getTargetRPM() {
    return m_targetRPM;
  }

  public boolean atTargetVelocity() {
    return m_targetRPM != 0.0 && Math.abs(getVelocity() - m_targetRPM) < 10; // 20;
  }

  public double getRPM(double Distance) { // Returns RPM based on Limelight Distance
    return (-81.80711
        + (48.30247 * Math.pow(Distance, 1))
        - (0.5783141 * Math.pow(Distance, 2))
        + (0.003585951 * Math.pow(Distance, 3))
        - (0.00001110918 * Math.pow(Distance, 4))
        + (1.436287 * Math.pow(10, -8) * Math.pow(Distance, 5)));
    // y = -1287.519 + 181.4057*x - 3.331198*x^2 + 0.02970816*x^3 - 0.0001251564*x^4
    // +
    // 2.006972e-7*x^5 e-7
    // return (5.839707 * Distance) + 1960.358;
  }

  public void setTargetRPMCompensation(double value) {
    m_targetRPM_compensation = value;
  }
}
