// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  public ShootCommand(ShooterSubsystem s, IndexerSubsystem index, IntakeSubsystem intake) {
    m_shooterSubsystem = s;
    m_indexerSubsystem = index;
    m_intakeSubsystem = intake;
    addRequirements(s, index, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.shoot();

    if(m_shooterSubsystem.atTargetVelocity()) {
        m_indexerSubsystem.in();
        m_intakeSubsystem.in();
    } else {
      m_indexerSubsystem.stop();
      m_intakeSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.stop();
    m_intakeSubsystem.stop();
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}