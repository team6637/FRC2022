// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(53, MotorType.kBrushless);

  public void in() {
    motor.set(-0.7);
  }

  public void out() {
    motor.set(0.7);
  }

  public void stop() {
    motor.set(0.0);
  }

}
