package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** LimelightSubsystem to manage reading from the network tables for the limelight telemetry */
public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable table;

  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry leds;
  private NetworkTableEntry booleanLeds;

  private double x;
  private double filteredX;
  private double y;
  private double area; // area is between 0 and 100. Calculated as a percentage of image
  private boolean target;
  private boolean ledStatus; // true = ON
  private double filteredArea;

  private LinearFilter x_iir;
  private LinearFilter area_iir;

  private double filterTC = 0.8; // seconds, cutoff 1.25Hz

  private static final double kLimelightMountAngleDegrees = 30.0;
  private static final double kLimelightLensHeightInches = 28.0;
  private static final double kGoalHeightInches = 104.0;

  private DoubleLogEntry m_xLog;
  private DoubleLogEntry m_yLog;
  private DoubleLogEntry m_distanceLog;

  public LimelightSubsystem() {
    DataLog log = DataLogManager.getLog();
    m_xLog = new DoubleLogEntry(log, "/limelight/x");
    m_yLog = new DoubleLogEntry(log, "/limelight/y");
    m_distanceLog = new DoubleLogEntry(log, "/limelight/distance");

    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx"); // -27 degrees to 27 degrees
    ty = table.getEntry("ty"); // -20.5 to 20.5 degrees
    ta = table.getEntry("ta");
    tv = table.getEntry("tv"); // target validity (1 or 0)
    leds = table.getEntry("ledMode");
    booleanLeds = table.getEntry("booleanLeds");
    disableLED();
  }

  @Override
  public void periodic() {
    // updates global variables
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = (tv.getDouble(0) == 0) ? (false) : (true);
    filteredX = x_iir.calculate(x);
    filteredArea = area_iir.calculate(area);
    ledStatus = (leds.getDouble(0) == 3) ? (true) : (false);

    m_xLog.append(x);
    m_yLog.append(y);
    m_distanceLog.append(getDistance());
  }

  public double getX() {
    return x;
  }

  public double getFilteredX() {
    return filteredX;
  }

  public double getFilteredArea() {
    return filteredArea;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public boolean getTarget() {
    return target;
  }

  public boolean getLEDStatus() {
    return ledStatus;
  }

  public void disableLED() {
    leds.setNumber(1);
    ledStatus = false;
    booleanLeds.setBoolean(ledStatus);
  }

  public void enableLED() {
    leds.setNumber(3);
    ledStatus = true;
    booleanLeds.setBoolean(ledStatus);
  }

  public void toggleLEDs() {
    if (ledStatus) disableLED();
    else enableLED();
  }

  /** true if the limelight has found a target */
  public boolean valid() {
    return target;
  }

  /** Do the solvePnP to determine the distance to the target */
  public double getDistance() {
    // If there is no target recognized, return 0 distance
    if (!valid()) return 0.0;

    // distance from the target to the floor
    double angleToGoalDegrees = kLimelightMountAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (kGoalHeightInches - kLimelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }
}
