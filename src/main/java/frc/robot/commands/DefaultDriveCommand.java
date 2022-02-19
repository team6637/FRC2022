package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    DoubleSupplier x;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier,
        DoubleSupplier x) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.x = x;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        if(x.getAsDouble() > 0) {
            drive_robot_relative();
        } else {
            drive_robot_relative();
            //drive_field_relative();
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
                m_rotationSupplier.getAsDouble(),
                m_drivetrainSubsystem.getGyroscopeRotation()
            )
        );
    }

    public void drive_robot_relative() {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble() + x.getAsDouble(),
            m_rotationSupplier.getAsDouble()
        ));
    }
}
