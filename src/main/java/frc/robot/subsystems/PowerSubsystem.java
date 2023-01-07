// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/** Monitor and log the power distribution hub current usage. */
public class PowerSubsystem extends SubsystemBase {
  private final PowerDistribution m_powerDistribution;

  private DoubleLogEntry m_totalCurrent;

  /** Creates a new PowerSubsystem. */
  public PowerSubsystem() {
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    DataLog log = DataLogManager.getLog();
    m_totalCurrent = new DoubleLogEntry(log, "/power/total_current");

    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    compTab.addNumber("Compressor V", this::getCompressorVoltage).withPosition(6, 2).withSize(4, 2);
    compTab.addNumber("total current", this::getTotalCurrent).withPosition(6, 0);
    compTab.addNumber("voltage", this::getVoltage).withPosition(6, 1);
  }

  public double getCompressorVoltage() {
    return m_powerDistribution.getCurrent(20);
  }

  public double getTotalCurrent() {
    return m_powerDistribution.getTotalCurrent();
  }

  public double getVoltage() {
    return m_powerDistribution.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_totalCurrent.append(m_powerDistribution.getTotalCurrent());
    if (Robot.isRobotEnabled()) {
      m_totalCurrent.append(m_powerDistribution.getVoltage());
    }
  }
}
