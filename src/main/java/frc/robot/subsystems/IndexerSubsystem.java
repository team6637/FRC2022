// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IndexerSubsystem extends SubsystemBase {

  CANSparkMax motor;
  private final double power = 0.7;

  public IndexerSubsystem() {
    motor = new CANSparkMax(INDEXER, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
  }

  public void in() {
    motor.set(power);
  }

  public void out() {
    motor.set(-power);
  }

  public void stop() {
    motor.set(0.0);
  }

}
