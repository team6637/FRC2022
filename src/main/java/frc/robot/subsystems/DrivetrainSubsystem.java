// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    // limelight turn
    private double limelightTurn = 0;

    // reduce to slow robot
    public static final double MAX_VOLTAGE = 12.0;

    // 3.87119 m/s
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI
        / 1.074;

    public double get_max_velocity() {
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    // Theoretical maximum angular velocity
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    //private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
    private final Pigeon2 m_pigeon = new Pigeon2(30);

    SwerveModuleState[] currentStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());

    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), new Pose2d());

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(8, 12)
                    .withPosition(0, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(8, 12)
                    .withPosition(8, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(8, 12)
                    .withPosition(16, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(8, 12)
                    .withPosition(24, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );
    }

    public void zeroGyroscope() {
        //m_pigeon.setFusedHeading(0.0);
        m_pigeon.setYaw(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(states);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return m_chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        currentStates = states;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());        
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, this.getGyroscopeRotation());
    }

    public double getLimelightTurn() {
        return limelightTurn;
    }

    public void setLimelightTurn(double v) {
        limelightTurn = v;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("drive max velocity", MAX_VELOCITY_METERS_PER_SECOND);
        // put pose info in SmartDashboard
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation", getPose().getRotation().getDegrees());
        // SmartDashboard.putNumber("drive gyro rotation", getGyroscopeRotation().getDegrees());
        
        m_odometry.update(
            this.getGyroscopeRotation(),
            currentStates[0], currentStates[1], currentStates[2], currentStates[3]
        );
    }
}
