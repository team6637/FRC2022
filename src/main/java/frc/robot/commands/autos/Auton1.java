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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;

public class Auton1 extends SequentialCommandGroup {

  public Auton1(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem, LimelightSubsystem m_limelightSubsystem) {

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

    // Drive Forward
    Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))), 
            new Pose2d(1.0, 0.0, new Rotation2d(Math.toRadians(0.0)))
        ),
        config
    );
    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        trajectory3,
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

        // drive from ball1 to ball2, the stop
        command,
        new InstantCommand(()->m_drivetrainSubsystem.drive(new ChassisSpeeds()), m_drivetrainSubsystem),

        // aim, shoot
        new ParallelCommandGroup(
            new AutoAim(false, m_limelightSubsystem, m_drivetrainSubsystem, m_shooterSubsystem).withTimeout(2.0),
            new ShootCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem).withTimeout(2.0)
        ),

        // turn intake on, reset post, drive forward, stop
        new InstantCommand(m_intakeSubsystem::in, m_intakeSubsystem),
        new InstantCommand(()->m_drivetrainSubsystem.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0)))), m_drivetrainSubsystem),
        swerveControllerCommand3,
        new InstantCommand(()->m_drivetrainSubsystem.drive(new ChassisSpeeds()), m_drivetrainSubsystem),

        new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem),

        // aim, shoot
        new ParallelCommandGroup(
            new AutoAim(false, m_limelightSubsystem, m_drivetrainSubsystem, m_shooterSubsystem).withTimeout(1.0),
            new ShootCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem).withTimeout(1.0)
        )

    );
  }
}
