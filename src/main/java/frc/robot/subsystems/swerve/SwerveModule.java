package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.DriveConstants;

/** A SwerveModules consists of a drive motor and a steer motor */
public class SwerveModule {
  private final DriveController driveController;
  private final SteerController steerController;

  public SwerveModule(
      String name,
      ShuffleboardContainer dashboardContainer,
      DriveController driveController,
      SteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
    dashboardContainer.addNumber(
        name + "Current Angle", () -> Math.toDegrees(steerController.getStateAngle()));
    dashboardContainer.addNumber(
        name + "Target Angle", () -> Math.toDegrees(steerController.getReferenceAngle()));
    dashboardContainer.addNumber(
        name + "Absolute Encoder Angle", () -> Math.toDegrees(steerController.getAbsoluteAngle()));
    dashboardContainer.addNumber(name + "Current Velocity", driveController::getStateVelocity);
  }

  public SwerveModule(DriveController driveController, SteerController steerController) {
    this.driveController = driveController;
    this.steerController = steerController;
  }

  /** The current velocity in meters per second */
  public double getDriveVelocity() {
    return driveController.getStateVelocity();
  }

  /** The current steer angle in degrees */
  public double getSteerAngle() {
    return steerController.getStateAngle();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition( driveController.getDistance(), new Rotation2d(getSteerAngle()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  public void set(double driveVoltage, double steerAngle) {
    if (driveVoltage == 0.0 && steerAngle != Math.toRadians(DriveConstants.kXModeAngle)) {
      driveController.setReferenceVoltage(driveVoltage);
      return;
    }
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    double difference = steerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
    if (difference >= Math.PI) {
      steerAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      steerAngle += 2.0 * Math.PI;
    }
    difference = steerAngle - getSteerAngle(); // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so
    // the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into the range [0,
      // 2pi)
      steerAngle += Math.PI;
      driveVoltage *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    driveController.setReferenceVoltage(driveVoltage);
    steerController.setReferenceAngle(steerAngle);
  }
}
