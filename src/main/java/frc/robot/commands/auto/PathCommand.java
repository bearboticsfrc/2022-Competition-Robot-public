// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.PathPlannerDebugCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/** A command to run a given trajectory */
public class PathCommand extends SequentialCommandGroup {

  public static final boolean kDebugMode = true;
  public static final boolean kDebugMode2 = false;

  /** Creates a new PathCommand. */
  public PathCommand(DriveSubsystem driveSubsystem, PathPlannerTrajectory pathPlannerTrajectory) {

    if (kDebugMode2) {
      double trajectoryLength = pathPlannerTrajectory.getTotalTimeSeconds();
      System.out.println(
          "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory total time = " + trajectoryLength);
      System.out.println(
          "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Trajectory states size = "
              + pathPlannerTrajectory.getStates().size());
      List<State> states = pathPlannerTrajectory.getStates();

      for (State state : states) {
        PathPlannerState pState = (PathPlannerState) state;
        System.out.println(
            "%%%%%%%%%%%%%%%%%%%%%%%%%%% holonomicRotation: "
                + pState.holonomicRotation
                + " state: "
                + pState.toString());
      }
    }

    var thetaController =
        new PIDController(
            AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand =
        new PPSwerveControllerCommand(
            pathPlannerTrajectory,
            driveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveSubsystem::setModuleStatesForAuto,
            driveSubsystem);

    Command autoCommand = swerveControllerCommand;

    if (kDebugMode) {
      PathPlannerDebugCommand pathDebugCommand =
          new PathPlannerDebugCommand(pathPlannerTrajectory, driveSubsystem::getPose);
      ParallelCommandGroup parallelCommandGroup =
          new ParallelCommandGroup(swerveControllerCommand, pathDebugCommand);
      autoCommand = parallelCommandGroup;
    }

    addCommands(autoCommand);
  }
}
