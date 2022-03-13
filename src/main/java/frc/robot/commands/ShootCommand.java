// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  private int justShootItTimer;
  private int intakeDelayTimer;

  private boolean indexerOn = false;

  public ShootCommand(ShooterSubsystem s, IndexerSubsystem index, IntakeSubsystem intake) {
    m_shooterSubsystem = s;
    m_indexerSubsystem = index;
    m_intakeSubsystem = intake;
    addRequirements(s, index, intake);
  }

  @Override
  public void initialize() {
    intakeDelayTimer = 0;
    justShootItTimer = 0;
    indexerOn = false;
  }

  @Override
  public void execute() {
    m_shooterSubsystem.shoot();

    if((m_shooterSubsystem.atTargetVelocity() || justShootItTimer > 50) && indexerOn == false) {
        m_indexerSubsystem.in();
        indexerOn = true;
    } else if(indexerOn && m_shooterSubsystem.atTargetVelocity() && intakeDelayTimer > 30) {
      m_intakeSubsystem.in();
    } else {
      justShootItTimer++;
    }

    if(indexerOn) intakeDelayTimer++;

    SmartDashboard.putNumber("shooter just shoot timer", justShootItTimer);
    SmartDashboard.putNumber("shooter timer", intakeDelayTimer);

  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.stop();
    m_intakeSubsystem.stop();
    m_shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
