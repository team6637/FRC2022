// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;


public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(SHOOTER_LEFT, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(SHOOTER_RIGHT, MotorType.kBrushless);

  Encoder velocityEncoder = new Encoder(SHOOTER_ENCODER_1, SHOOTER_ENCODER_2);

  public void shoot() {
    motor1.set(-0.8);
    motor2.set(0.8);
  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
  }
}