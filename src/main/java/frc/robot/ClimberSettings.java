// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ClimberConstants;
import java.util.Map;

/** Dashboard settings for climber testing */
public class ClimberSettings {

  static ShuffleboardTab tab = Shuffleboard.getTab("Climb");

  static GenericEntry ClimbManualEntry =
      tab.add("Manual climber", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(2, 1)
          .withPosition(0, 0)
          .withProperties(
              Map.of("min", ClimberConstants.kMinHeight, "max", ClimberConstants.kMaxHeight))
          .getEntry();

  public static double getManualPosition() {
    return ClimbManualEntry.getDouble(0.0);
  }
}
