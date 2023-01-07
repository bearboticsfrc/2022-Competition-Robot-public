// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LEDconstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.LogCommand;
import frc.robot.commands.TargetDrive;
import frc.robot.commands.auto.MoveAndShootAuto1Command;
import frc.robot.commands.auto.TwoBallAutoCommand;
import frc.robot.commands.shoot.AutoShootCommand;
import frc.robot.commands.shoot.SimpleShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Color;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  public static RobotContainer instance = null;

  public final LEDSubsystem led = new LEDSubsystem(LEDconstants.port_number);

  private final PneumaticHub pneumatichub = new PneumaticHub(HopperConstants.kPneumaticHubPort);

  private final DriveSubsystem robotDrive = new DriveSubsystem();

  // private final PowerSubsystem m_power = new PowerSubsystem();

  private final LimelightSubsystem limeLight = new LimelightSubsystem();

  private final ClimberSubsystem climber = new ClimberSubsystem(pneumatichub);

  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  // XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final HopperSubsystem hopper = new HopperSubsystem(pneumatichub);

  //  private final ShooterSubsystem shooter = new ShooterSubsystem( () -> { return
  // -MathUtil.applyDeadband(operatorController.getRightY(), 0.1); } );
  private final ShooterSubsystem shooter =
      new ShooterSubsystem(
          () -> {
            return 0.0;
          });

  private final ShooterSettings shooterSettings = new ShooterSettings();

  private final TargetDrive targetDrive =
      new TargetDrive(
          robotDrive,
          limeLight,
          shooter,
          () -> {
            return -MathUtil.applyDeadband(driverController.getLeftY(), 0.1);
          },
          () -> {
            return -MathUtil.applyDeadband(driverController.getLeftX(), 0.1);
          });

  private List<Command> autonomousList = new ArrayList<Command>();

  private AutonomousSelector autonomousSelector = new AutonomousSelector();

  // A chooser for autonomous commands
  private SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    // Configure the button bindings
    configureButtonBindings();

    RobotController.setBrownoutVoltage(6.0);
    DataLogManager.log(
        ">>>>>>>>>>>>>>>>>Brownout voltage = " + RobotController.getBrownoutVoltage());

    DataLogManager.log(
        "DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = "
            + DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    pneumatichub.enableCompressorDigital();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                    -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
            robotDrive));

    // ShuffleboardTab driveSystemTab = Shuffleboard.getTab("Drive System");
    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
    // ShuffleboardTab testTab = Shuffleboard.getTab("Test");

    compTab.addBoolean("Target Mode", targetDrive::isScheduled).withPosition(0, 0);
    compTab.addBoolean("Target Lock", limeLight::valid).withPosition(1, 0);
    // testTab.addNumber("Battery Voltage", RobotController::getBatteryVoltage)
    //   .withPosition(2,0)
    //   .withSize(5,2)
    //   .withWidget(BuiltInWidgets.kGraph);

    compTab.addNumber("Distance", limeLight::getDistance).withPosition(3, 2);

    compTab.addBoolean("Vertical Sensor", hopper::getVerticalSensor).withPosition(2, 0);
    compTab.addBoolean("Intake LSensor", hopper::getIntakeSensor).withPosition(3, 0);
    compTab.addBoolean("Hopper Sensor", hopper::getHopperSensor).withPosition(4, 0);
    compTab.addBoolean("LimeLight LED", limeLight::getLEDStatus).withPosition(5, 0);

    limeLight.disableLED();

    buildAutonomousList();
    configureAutonomousSelector(compTab);
    configureAutonomousChooser(compTab);

    compTab.add("Auto Override", chooser).withPosition(1, 1);
    compTab
        .addString("Auto to run", () -> this.getAutonomousCommand().getName())
        .withPosition(0, 1);
    compTab
        .addString("Switch Auto", () -> autonomousSelector.getSelected().getName())
        .withPosition(2, 1);

    // compTab.addNumber("Time Left", () -> 150 -DriverStation.getMatchTime()
    // ).withPosition(6,2).withSize(2,2);

    climber.setDefaultCommand(
        new RunCommand(
            () -> {
              if (operatorController.getLeftTriggerAxis() > 0.95) {
                climber.extendElevator(-MathUtil.applyDeadband(operatorController.getLeftY(), 0.1));
              }
            },
            climber));
  }

  public DriveSubsystem getDriveSubsystem() {
    return robotDrive;
  }

  private void buildAutonomousList() {
    autonomousList.add(new MoveAndShootAuto1Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        new TwoBallAutoCommand(robotDrive, hopper, shooter, limeLight, "auto2", "Auto2"));
    autonomousList.add(
        new TwoBallAutoCommand(robotDrive, hopper, shooter, limeLight, "auto3", "Auto3"));
    autonomousList.add(
        new TwoBallAutoCommand(robotDrive, hopper, shooter, limeLight, "auto4", "Auto4"));
    autonomousList.add(
        AutonomousCommandHelper.getAuto5Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto6Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto7Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto8Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto9Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto10Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto11Command(robotDrive, hopper, shooter, limeLight));
    autonomousList.add(
        AutonomousCommandHelper.getAuto12Command(robotDrive, hopper, shooter, limeLight));
  }

  private void configureAutonomousChooser(ShuffleboardTab tab) {
    chooser.setDefaultOption("Switch", new InstantCommand().withName("Switch"));

    for (Command command : autonomousList) {
      chooser.addOption(command.getName(), command);
    }
  }

  private void configureAutonomousSelector(ShuffleboardTab tab) {
    autonomousSelector.setCommands(autonomousList.toArray(new Command[0]));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> robotDrive.zeroHeading()));

    new JoystickButton(driverController, Button.kB.value)
        .onTrue(new InstantCommand(() -> hopper.setIntakeSensor(false)))
        .onFalse(new InstantCommand(() -> hopper.setIntakeSensor(true)));

    new JoystickButton(driverController, Button.kY.value)
        .onTrue(new InstantCommand(hopper::reverseIntake))
        .onFalse(new InstantCommand(hopper::stopIntake));

    new JoystickButton(driverController, Button.kX.value)
        .onTrue(
            new ParallelRaceGroup(
                targetDrive.withTimeout(5.0),
                new SequentialCommandGroup(
                    new LogCommand("entering shoot cycle"),
                    new WaitUntilCommand(
                            () -> {
                              return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                            })
                        .withTimeout(2.0),
                    new LogCommand("continuing shoot cycle"),
                    new AutoShootCommand(shooter, hopper, limeLight),
                    new LogCommand("Done shooting"))));

    new JoystickButton(driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> robotDrive.setFieldRelative(false)))
        .onFalse(new InstantCommand(() -> robotDrive.setFieldRelative(true)));

    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.95)
        .onTrue(new InstantCommand(() -> robotDrive.setTurboMode(true)))
        .onFalse(new InstantCommand(() -> robotDrive.setTurboMode(false)));

    /*
        new Trigger(() -> !hopper.getIntakeSensor())
            .whenActive(new ParallelCommandGroup(new InstantCommand(() -> driverController.setRumble(RumbleType.kLeftRumble, 1)),
                                                 new InstantCommand(() -> driverController.setRumble(RumbleType.kRightRumble, 1))))
            .whenInactive(new ParallelCommandGroup(new InstantCommand(() -> driverController.setRumble(RumbleType.kLeftRumble, 0)),
                                                   new InstantCommand(() -> driverController.setRumble(RumbleType.kRightRumble, 0))));
    */

    /*  new Trigger(() -> { return targetDrive.isScheduled() && !limeLight.valid();} )
            .whenActive(() -> {driverController.setRumble(RumbleType.kLeftRumble, 0.1);
                               driverController.setRumble(RumbleType.kRightRumble, 0.1); })
            .whenInactive(() -> {driverController.setRumble(RumbleType.kLeftRumble, 0);
                                 driverController.setRumble(RumbleType.kRightRumble, 0); });

        new Trigger(() -> { return !Robot.inAuto() && shooter.atTargetVelocity();} )
            .whenActive(() -> {operatorController.setRumble(RumbleType.kLeftRumble, 0.1);
                               operatorController.setRumble(RumbleType.kRightRumble, 0.1); })
            .whenInactive(() -> {operatorController.setRumble(RumbleType.kLeftRumble, 0);
                                 operatorController.setRumble(RumbleType.kRightRumble, 0); });
    */

    new JoystickButton(driverController, Button.kRightBumper.value)
        .toggleOnTrue(
            new StartEndCommand(hopper::extendIntake, hopper::retractIntake, hopper));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.75)
        .onTrue(new InstantCommand(hopper::startIntake))
        .onFalse(new InstantCommand(hopper::stopIntake));

    /*new Trigger(() -> { return driverController.getPOV() == 0;} )
    .whenActive(
        new ParallelRaceGroup( targetDrive.withTimeout(5.0),
                               new SequentialCommandGroup(
                                    new LogCommand("entering shoot cycle"),
                                    new WaitUntilCommand(() -> { return (Math.abs(targetDrive.getDegreesToTurn()) <1.0);}).withTimeout(2.0),
                                    new LogCommand("continuing shoot cycle"),
                                    new AutoShootCommand(shooter, hopper, limeLight),
                                    new LogCommand("Done shooting")))
    ); */

    new Trigger(
            () -> {
              return driverController.getPOV() == 0;
            })
        .onTrue(new SimpleShootCommand(shooter, hopper, shooterSettings.getUpperRPM()));

    new Trigger(
            () -> {
              return driverController.getPOV() == 180;
            })
        .onTrue(new SimpleShootCommand(shooter, hopper, ShooterConstants.kLowPortRPM));
    // .toggleWhenActive(new SequentialCommandGroup(
    // new InstantCommand(hopper::reverseVertical).andThen(new WaitCommand(0.1).andThen(new
    // InstantCommand(hopper::stopVertical))),
    //    targetDrive
    // ));

    new Trigger(
            () -> {
              return driverController.getPOV() == 270;
            })
        .onTrue(new InstantCommand(hopper::stopBallOne));

    /*    new JoystickButton(driverController, Button.kB.value)
            .whenPressed(hopper::reverseIntake)
            .whenReleased(hopper::stopIntake);
    */

    // Operator Controller Mappings
    /*
        new JoystickButton(operatorController, Button.kLeftBumper.value)
            .whenPressed(() -> hopper.setIntakeSensor(false))
            .whenReleased(() -> hopper.setIntakeSensor(true));

        new JoystickButton(operatorController, Button.kA.value)
            .whenPressed(new SimpleShootCommand(shooter, hopper, ShooterConstants.kLowPortRPM));

        new JoystickButton(operatorController, Button.kB.value)
            .whenPressed(new ManualShootCommand(shooter, hopper, shooterSettings));

        new JoystickButton(operatorController, Button.kX.value)
            .whenPressed(new SimpleShootCommand(shooter, hopper, shooterSettings.getUpperRPM()));

        new JoystickButton(operatorController, Button.kY.value)
            .whenPressed(new AutoShootCommand(shooter, hopper, limeLight));
    */

    new JoystickButton(operatorController, Button.kBack.value)
        .onTrue(new ClimbCommand(climber));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 0;
            })
        .onTrue(new InstantCommand(climber::extendClimberMid));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 90;
            })
        .onTrue(new InstantCommand(climber::extendClimberLow));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 180;
            })
        .onTrue(new InstantCommand(climber::retractClimber));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 270;
            })
        .onTrue(new InstantCommand(() -> climber.setClimberPosition(ClimberConstants.kMidClimb)));

    new Trigger(
            () -> {
              return operatorController.getStartButton()
                  && operatorController.getRightBumperPressed();
            })
        .onTrue(new InstantCommand(() -> climber.extendTraverse()));

    new Trigger(
            () -> {
              return operatorController.getStartButton()
                  && operatorController.getRightTriggerAxis() > 0.95;
            })
        .onTrue(new InstantCommand(() -> climber.retractTraverse()));

    configureButtonBindingsClimb(operatorController);
  }

  private void configureButtonBindingsClimb(XboxController operatorController) {

    // Operator Controller Mappings

    new JoystickButton(operatorController, Button.kBack.value)
        .onTrue(new ClimbCommand(climber));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 0;
            })
        .onTrue(new InstantCommand(climber::extendClimberMid));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 90;
            })
        .onTrue(new InstantCommand(climber::extendClimberLow));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 180;
            })
        .onTrue(new InstantCommand(climber::retractClimber));

    new Trigger(
            () -> {
              return operatorController.getPOV() == 270;
            })
        .onTrue(new InstantCommand(() -> climber.setClimberPosition(ClimberConstants.kMidClimb)));

    new Trigger(
            () -> {
              return operatorController.getStartButton()
                  && operatorController.getRightBumperPressed();
            })
        .onTrue(new InstantCommand(() -> climber.extendTraverse()));

    new Trigger(
            () -> {
              return operatorController.getStartButton()
                  && operatorController.getRightTriggerAxis() > 0.95;
            })
        .onTrue(new InstantCommand(() -> climber.retractTraverse()));
  }

  public HopperSubsystem getHopperSubsystem() {
    return hopper;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = chooser.getSelected();
    if (command.getName().equals("Switch")) {
      command = autonomousSelector.getSelected();
    }
    return command;
  }

  public void periodic() {

    if (hopper.ballCount() == 1) {
      led.set(Color.GOLD);
    }
    if (hopper.ballCount() == 2) {
      led.set(Color.GREEN);
    }
    if (hopper.ballCount()==0) {
      led.set(Color.RED);
    }
  }
}
