package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.LogCommand;
import frc.robot.commands.TargetDrive;
import frc.robot.commands.auto.BeginAutoCommand;
import frc.robot.commands.auto.PathCommand;
import frc.robot.commands.shoot.AutoShootCommand;
import frc.robot.commands.shoot.AutoShootForTwoBallAutoCommand;
import frc.robot.commands.shoot.SimpleShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Helper class to build autonomous routines. */
public class AutonomousCommandHelper {
  public static PathPlannerState initialState = null;

  public static Command getAuto5Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      // HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {

    String autoName = "Auto5";
    PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.loadPath("auto5A", 1, 2);
    PathPlannerTrajectory pathPlannerTrajectory2 = PathPlanner.loadPath("auto5b", 1, 2);

    Command pathCommand = new PathCommand(driveSubsystem, pathPlannerTrajectory);
    Command pathCommand2 = new PathCommand(driveSubsystem, pathPlannerTrajectory2);
    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
            new BeginAutoCommand(
                driveSubsystem, hopperSubsystem, pathPlannerTrajectory.getInitialState(), autoName),
            pathCommand,
            new InstantCommand(() -> hopperSubsystem.retractIntake()),
            targetDrive.raceWith(
                new WaitUntilCommand(
                    () -> {
                      return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                    })),
            new AutoShootCommand(shooterSubsystem, hopperSubsystem, limeLight),
            new InstantCommand(() -> hopperSubsystem.extendIntake()),
            pathCommand2,
            new AutoShootCommand(shooterSubsystem, hopperSubsystem, limeLight),
            new InstantCommand(() -> hopperSubsystem.retractIntake()),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new InstantCommand(() -> hopperSubsystem.stopBallOne()),
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> driveSubsystem.stop()))
        .withName(autoName);
  }

  public static Command getAuto6Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      // HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "Auto6";
    PathPlannerTrajectory pathPlannerTrajectory1 = PathPlanner.loadPath("auto6a", 1, 1);
    // PathPlannerTrajectory pathPlannerTrajectory2 = PathPlanner.loadPath("auto6b", 1, 2);
    PathPlannerTrajectory pathPlannerTrajectory3 = PathPlanner.loadPath("auto6c", 2.3, 2);
    PathPlannerTrajectory pathPlannerTrajectory4 = PathPlanner.loadPath("auto6d", 2.4, 2);
    PathPlannerTrajectory pathPlannerTrajectory5 = PathPlanner.loadPath("auto6e", 2.8, 4);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    // Command pathCommand2 = getPathCommand(driveSubsystem, pathPlannerTrajectory2);
    Command pathCommand3 = new PathCommand(driveSubsystem, pathPlannerTrajectory3);
    Command pathCommand4 = new PathCommand(driveSubsystem, pathPlannerTrajectory4);
    Command pathCommand5 = new PathCommand(driveSubsystem, pathPlannerTrajectory5);

    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);
    TargetDrive targetDrive2 = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);
    TargetDrive targetDrive3 = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    return new SequentialCommandGroup(
            new BeginAutoCommand(
                driveSubsystem,
                hopperSubsystem,
                pathPlannerTrajectory1.getInitialState(),
                autoName),
            pathCommand1,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            // pathCommand2,
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new WaitCommand(0.4),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(() -> hopperSubsystem.startIntake()),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(false)),
            pathCommand3,
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new WaitCommand(0.4),
            new LogCommand("Shooting second ball"),
            new ParallelRaceGroup(
                targetDrive2.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(() -> hopperSubsystem.startIntake()),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(false)),
            pathCommand4,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new WaitCommand(0.5),
            pathCommand5,
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> hopperSubsystem.retractIntake()),
            new WaitCommand(0.3),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive3.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(
                () -> {
                  hopperSubsystem.setIntakeSensor(true);
                  hopperSubsystem.stopBallOne();
                  hopperSubsystem.stopIntake();
                  driveSubsystem.stop();
                  driveSubsystem.headingOffest(
                      pathPlannerTrajectory1.getInitialState().holonomicRotation.getDegrees());
                }))
        .withName(autoName);
  }

  public static Command getAuto7Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      // HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "Auto7";
    PathPlannerTrajectory pathPlannerTrajectory1 = PathPlanner.loadPath("auto7a", 1, 1);
    // PathPlannerTrajectory pathPlannerTrajectory2 = PathPlanner.loadPath("auto6b", 1, 2);
    PathPlannerTrajectory pathPlannerTrajectory3 = PathPlanner.loadPath("auto7c", 2.3, 2);
    // PathPlannerTrajectory pathPlannerTrajectory4 = PathPlanner.loadPath("auto7d", 2.4, 2);
    PathPlannerTrajectory pathPlannerTrajectory5 = PathPlanner.loadPath("auto7e", 3.2, 4);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    // Command pathCommand2 = getPathCommand(driveSubsystem, pathPlannerTrajectory2);
    Command pathCommand3 = new PathCommand(driveSubsystem, pathPlannerTrajectory3);
    // Command pathCommand4 = new PathCommand(driveSubsystem, pathPlannerTrajectory4);
    Command pathCommand5 = new PathCommand(driveSubsystem, pathPlannerTrajectory5);

    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);
    // TargetDrive targetDrive2 = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);
    TargetDrive targetDrive3 = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    return new SequentialCommandGroup(
            new BeginAutoCommand(
                driveSubsystem,
                hopperSubsystem,
                pathPlannerTrajectory1.getInitialState(),
                autoName),
            pathCommand1,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            // pathCommand2,
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new WaitCommand(0.4),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(() -> hopperSubsystem.startIntake()),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(false)),
            pathCommand3,
            new InstantCommand(driveSubsystem::stop),
            // new InstantCommand(() -> hopperSubsystem.retractIntake()).andThen(new
            // ScheduleCommand(new WaitCommand(0.5).andThen(new InstantCommand(() ->
            // hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new WaitCommand(0.4),
            pathCommand5,
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> hopperSubsystem.retractIntake()),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive3.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(
                () -> {
                  hopperSubsystem.setIntakeSensor(true);
                  hopperSubsystem.stopBallOne();
                  hopperSubsystem.stopIntake();
                  driveSubsystem.stop();
                  driveSubsystem.headingOffest(
                      pathPlannerTrajectory1.getInitialState().holonomicRotation.getDegrees());
                }))
        .withName(autoName);
  }

  /**
   * Four ball auto, with pickup of human player ball
   *
   * @param driveSubsystem
   * @param hopperSubsystem
   * @param shooterSubsystem
   * @param limeLight
   * @return
   */
  public static Command getAuto8Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "Auto8";
    PathPlannerTrajectory pathPlannerTrajectory1 = PathPlanner.loadPath("auto8FourBall", 1, 1);
    PathPlannerTrajectory pathPlannerTrajectory2 = PathPlanner.loadPath("auto8FourBall2", 1.5, 2);
    PathPlannerTrajectory pathPlannerTrajectory3 = PathPlanner.loadPath("auto8FourBall3", 2.3, 2);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    Command pathCommand2 = new PathCommand(driveSubsystem, pathPlannerTrajectory2);
    Command pathCommand3 = new PathCommand(driveSubsystem, pathPlannerTrajectory3);

    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);
    TargetDrive targetDrive2 = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    return new SequentialCommandGroup(
            new BeginAutoCommand(
                driveSubsystem,
                hopperSubsystem,
                pathPlannerTrajectory1.getInitialState(),
                autoName),
            pathCommand1,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new WaitCommand(0.2),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))),
            new InstantCommand(() -> hopperSubsystem.startIntake()),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(false)),
            pathCommand2,
            new InstantCommand(driveSubsystem::stop),
            new WaitCommand(1.0),
            pathCommand3,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new InstantCommand(() -> driveSubsystem.stop()), // remove me
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new InstantCommand(() -> hopperSubsystem.retractIntake()),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive2.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))),
            new InstantCommand(
                () -> {
                  hopperSubsystem.setIntakeSensor(true);
                  hopperSubsystem.stopBallOne();
                  hopperSubsystem.stopIntake();
                  driveSubsystem.stop();
                  driveSubsystem.headingOffest(
                      pathPlannerTrajectory1.getInitialState().holonomicRotation.getDegrees());
                }))
        .withName(autoName);
  }

  /**
   * Four ball auto, with pickup of human player ball
   *
   * @param driveSubsystem
   * @return
   */
  public static Command getAuto9Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "FigureEight";
    PathPlannerTrajectory pathPlannerTrajectory1 =
        PathPlanner.loadPath("figureEightRotate2", 1.5, 4);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    return new SequentialCommandGroup(
            new InstantCommand(() -> DataLogManager.log("Starting " + autoName)),
            new InstantCommand(
                () -> driveSubsystem.resetOdometry(pathPlannerTrajectory1.getInitialState())),
            new InstantCommand(
                () ->
                    AutonomousCommandHelper.initialState =
                        pathPlannerTrajectory1.getInitialState()),
            pathCommand1,
            new InstantCommand(() -> driveSubsystem.stop()),
            new WaitCommand(0.4),
            new LogCommand("Starting shoot cycle"),
            new ParallelRaceGroup(
                targetDrive.withTimeout(5.0),
                new SequentialCommandGroup(
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootForTwoBallAutoCommand(
                        shooterSubsystem, hopperSubsystem, limeLight))
                // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                // ShooterConstants.kLowPortRPM)
                ),
            new InstantCommand(
                () -> {
                  hopperSubsystem.setIntakeSensor(true);
                  hopperSubsystem.stopBallOne();
                  hopperSubsystem.stopIntake();
                }),
            new InstantCommand(
                () ->
                    driveSubsystem.headingOffest(
                        driveSubsystem.getHeading()
                            - pathPlannerTrajectory1
                                .getInitialState()
                                .holonomicRotation
                                .getDegrees())))
        .withName(autoName);
  }

  public static Command getAuto10Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "SimpleFigureEight";
    PathPlannerTrajectory pathPlannerTrajectory1 =
        PathPlanner.loadPath("figureEightRotate2", 1.5, 4);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    // TargetDrive targetDrive = new TargetDrive(driveSubsystem, limeLight, shooterSubsystem);

    return new SequentialCommandGroup(
            new InstantCommand(() -> DataLogManager.log("Starting " + autoName)),
            new InstantCommand(
                () -> driveSubsystem.resetOdometry(pathPlannerTrajectory1.getInitialState())),
            new InstantCommand(
                () ->
                    AutonomousCommandHelper.initialState =
                        pathPlannerTrajectory1.getInitialState()),
            pathCommand1,
            new InstantCommand(() -> driveSubsystem.stop()),
            new InstantCommand(
                () ->
                    driveSubsystem.headingOffest(
                        driveSubsystem.getHeading()
                            - pathPlannerTrajectory1
                                .getInitialState()
                                .holonomicRotation
                                .getDegrees())))
        .withName(autoName);
  }

  public static Command getAuto11Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "Auto11";

    return new SequentialCommandGroup(
            new WaitCommand(0.4),
            new SimpleShootCommand(shooterSubsystem, hopperSubsystem, ShooterConstants.kLowPortRPM))
        .withName(autoName);
  }

  public static Command getAuto12Command(
      DriveSubsystem driveSubsystem,
      HopperSubsystem hopperSubsystem,
      ShooterSubsystem shooterSubsystem,
      LimelightSubsystem limeLight) {
    String autoName = "Auto12";

    PathPlannerTrajectory pathPlannerTrajectory1 = PathPlanner.loadPath("auto12a", 1, 2);
    PathPlannerTrajectory pathPlannerTrajectory2 = PathPlanner.loadPath("auto12e", 1.5, 2);
    // PathPlannerTrajectory pathPlannerTrajectory3 = PathPlanner.loadPath("auto12c", 1.5, 2);
    // PathPlannerTrajectory pathPlannerTrajectory4 = PathPlanner.loadPath("auto12d", 1.5, 2);

    Command pathCommand1 = new PathCommand(driveSubsystem, pathPlannerTrajectory1);
    Command pathCommand2 = new PathCommand(driveSubsystem, pathPlannerTrajectory2);
    // Command pathCommand3 = new PathCommand(driveSubsystem, pathPlannerTrajectory3);
    // Command pathCommand4 = new PathCommand(driveSubsystem, pathPlannerTrajectory4);

    TargetDrive targetDrive =
        new TargetDrive(
            driveSubsystem,
            limeLight,
            shooterSubsystem,
            () -> {
              return 0.0;
            },
            () -> {
              return 0.0;
            },
            ShooterConstants.kLowPortRPM);

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
            new BeginAutoCommand(
                driveSubsystem,
                hopperSubsystem,
                pathPlannerTrajectory1.getInitialState(),
                autoName),
            pathCommand1,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new InstantCommand(() -> hopperSubsystem.stopIntake()),
            new WaitCommand(1.0),
            new InstantCommand(() -> hopperSubsystem.setIntakeSensor(true)),
            new ParallelRaceGroup(
                targetDrive.withTimeout(2),
                new SequentialCommandGroup(
                    new InstantCommand(
                        () -> {
                          DataLogManager.log("Waiting for degrees to turn < 1.0");
                        }),
                    new WaitUntilCommand(
                        () -> {
                          return (Math.abs(targetDrive.getDegreesToTurn()) < 1.0);
                        }),
                    new AutoShootCommand(shooterSubsystem, hopperSubsystem, limeLight),
                    // new SimpleShootCommand(shooterSubsystem, hopperSubsystem,
                    // ShooterConstants.kLowPortRPM),
                    new InstantCommand(
                        () -> {
                          DataLogManager.log("Finished shooting cycle.");
                        }),
                    new InstantCommand(() -> hopperSubsystem.stopBallOne()))),
            new InstantCommand(
                () -> {
                  hopperSubsystem.extendIntake();
                  hopperSubsystem.startIntake();
                  hopperSubsystem.setIntakeSensor(false);
                  DataLogManager.log(
                      String.format(
                          "After shooting, rotation is %03.2f , should be %03.2f",
                          driveSubsystem.getHeading(),
                          pathPlannerTrajectory2.getInitialState().holonomicRotation.getDegrees()));
                }),
            // rotate to start of new path ?
            new AutoRotate(
                driveSubsystem,
                driveSubsystem.getHeading()
                    - pathPlannerTrajectory2.getInitialState().holonomicRotation.getDegrees()),
            pathCommand2,
            new InstantCommand(() -> hopperSubsystem.retractIntake())
                .andThen(
                    new ScheduleCommand(
                        new WaitCommand(0.5)
                            .andThen(new InstantCommand(() -> hopperSubsystem.extendIntake())))),
            new SimpleShootCommand(shooterSubsystem, hopperSubsystem, ShooterConstants.kLowPortRPM),
            new InstantCommand(
                () -> {
                  hopperSubsystem.stopBallOne();
                  driveSubsystem.stop();
                }),
            new InstantCommand(
                () ->
                    driveSubsystem.headingOffest(
                        pathPlannerTrajectory1.getInitialState().holonomicRotation.getDegrees())))
        .withName(autoName);
  }
}
