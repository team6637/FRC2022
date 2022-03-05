package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    DoubleSupplier pixyX;
    BooleanSupplier driveIsFieldRelative;
    private double turnMultiplier = 0.6;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier,
        DoubleSupplier x,
        BooleanSupplier driveIsFieldRelative) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.pixyX = x;
        this.driveIsFieldRelative = driveIsFieldRelative;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if(driveIsFieldRelative.getAsBoolean()) {
            drive_field_relative();
        } else {
            drive_robot_relative();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void drive_field_relative() {
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble() * turnMultiplier + pixyX.getAsDouble() + m_drivetrainSubsystem.getLimelightTurn(),
                m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    public void drive_robot_relative() {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble() + pixyX.getAsDouble(),
            m_rotationSupplier.getAsDouble() * turnMultiplier + m_drivetrainSubsystem.getLimelightTurn()
        ));
    }
}
