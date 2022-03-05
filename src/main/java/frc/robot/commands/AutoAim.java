// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAim extends CommandBase {
  private LimelightSubsystem m_limelightSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private DrivetrainSubsystem m_drivetrain_subsystem;
  private double turnKp = 0.2;

  public AutoAim(LimelightSubsystem l, DrivetrainSubsystem d, ShooterSubsystem s) {
    this.m_limelightSubsystem = l;
    this.m_drivetrain_subsystem = d;
    this.m_shooterSubsystem = s;
    addRequirements(l,s);
    SmartDashboard.putNumber("auto aim turn kp", turnKp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightSubsystem.setupAutoAim();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnKp = SmartDashboard.getNumber("auto aim turn kp", turnKp);
    if(m_limelightSubsystem.isTarget()) {
      double turnError = m_limelightSubsystem.getTy();
      double turnPower = turnError * turnKp;
      m_drivetrain_subsystem.setLimelightTurn(turnPower);

    } else {
      m_drivetrain_subsystem.setLimelightTurn(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelightSubsystem.setupDriveMode();
    m_drivetrain_subsystem.setLimelightTurn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
