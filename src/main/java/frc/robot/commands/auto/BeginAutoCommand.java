// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutonomousCommandHelper;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

/**
 * An instant command to initialize the operations at the start of Autonomous. This includes Logging
 * the start, reset odometry, extend the intake, start the intake, and start running the hopper.
 */
public class BeginAutoCommand extends InstantCommand {
  private final DriveSubsystem m_driveSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final PathPlannerState m_initialState;
  private final String m_autoName;

  public BeginAutoCommand(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      PathPlannerState initialState,
      String autoName) {
    m_driveSubsystem = driveSubsystem;
    m_hopperSubsystem = hopperSubsystem;
    m_initialState = initialState;
    m_autoName = autoName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Starting " + m_autoName);
    m_driveSubsystem.resetOdometry(m_initialState);
    AutonomousCommandHelper.initialState = m_initialState;
    m_hopperSubsystem.extendIntake();
    m_hopperSubsystem.startIntake();
    m_hopperSubsystem.setIntakeSensor(false);
  }
}
