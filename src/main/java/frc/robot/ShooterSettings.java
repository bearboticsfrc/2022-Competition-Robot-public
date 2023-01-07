package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;

/**
 * Settings for shooter testing. Puts three sliders on the dashboard to be adjustable during
 * testing.
 */
public class ShooterSettings {
  private double manualRPMSetting = 1500.0;
  private double upperRPMSetting = 2500.0;
  private double lowerRPMSetting = 1000.0;

  ShuffleboardTab tab = Shuffleboard.getTab("Shoot");

  GenericEntry rpmManualEntry =
      tab.add("RPM Setting", manualRPMSetting)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(4, 1)
          .withPosition(2, 0)
          .withProperties(Map.of("min", 0.0, "max", 5000))
          .getEntry();
  GenericEntry rpmUpperEntry =
      tab.add("Upper RPM", upperRPMSetting)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(4, 1)
          .withPosition(2, 1)
          .withProperties(Map.of("min", 0.0, "max", 5000))
          .getEntry();
  GenericEntry rpmLowerEntry =
      tab.add("Lower RPM", lowerRPMSetting)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(4, 1)
          .withPosition(2, 2)
          .withProperties(Map.of("min", 0.0, "max", 5000))
          .getEntry();

  public double getManualRPM() {
    double value = rpmManualEntry.getDouble(0.0);
    System.out.println("Manual RPM = " + value);
    return value;
  }

  public double getUpperRPM() {
    return rpmUpperEntry.getDouble(0.0);
  }

  public double getLowerRPM() {
    return rpmLowerEntry.getDouble(0.0);
  }
}
