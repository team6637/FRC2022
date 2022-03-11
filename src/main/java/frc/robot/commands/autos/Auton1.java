// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;

public class Auton1 extends SequentialCommandGroup {

  public Auton1(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem) {

    // SETUP
    TrajectoryConfig config = new TrajectoryConfig(m_drivetrainSubsystem.get_max_velocity(), 3.0);
    config.setKinematics(m_drivetrainSubsystem.getKinematics());
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AUTON_TURN_KP, 0.0, 0.0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


    // Drive Forward
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new Pose2d(7.639, 1.5, new Rotation2d(Math.toRadians(-90.0))), 
            new Pose2d(7.639, 0.1, new Rotation2d(Math.toRadians(-90.0)))
        ),
        config
    );
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        m_drivetrainSubsystem.getKinematics(),
        new PIDController(AUTON_X_KP, 0.0, 0.0),
        new PIDController(AUTON_Y_KP, 0.0, 0.0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem
    );

    // Drive to Ball 2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Ball1ToBall2", m_drivetrainSubsystem.get_max_velocity(), 3.0);
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        examplePath,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        m_drivetrainSubsystem.getKinematics(),
        new PIDController(AUTON_X_KP, 0.0, 0.0),
        new PIDController(AUTON_Y_KP, 0.0, 0.0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem
    );

    addCommands(
        // reset odomotry
        new InstantCommand(()->m_drivetrainSubsystem.resetOdometry(new Pose2d(7.639, 1.5, new Rotation2d(Math.toRadians(-90.0)))), m_drivetrainSubsystem),

        // drive forward with intake
        new ParallelCommandGroup(
            swerveControllerCommand,
            new InstantCommand(m_intakeSubsystem::in, m_intakeSubsystem)
        ),

        // stop intake
        new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem),

        // reset odometry
        new InstantCommand(()->m_drivetrainSubsystem.resetOdometry(new Pose2d(7.64, 0.59, new Rotation2d(Math.toRadians(-90.0)))), m_drivetrainSubsystem),

        // drive from ball1 to ball2
        command
    );
  }
}
