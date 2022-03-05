// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax motor;
  DigitalInput ball1IsSeen;
  DigitalInput ball2IsSeen;

  private final double power = 0.9;

  public IntakeSubsystem() {
    motor = new CANSparkMax(INTAKE, MotorType.kBrushless);
    motor.restoreFactoryDefaults();

    ball1IsSeen = new DigitalInput(INTAKE_BALL_SENSOR_1);
    ball2IsSeen = new DigitalInput(INTAKE_BALL_SENSOR_2);
  }

  public void in() {
    motor.set(-power);
  }

  public void out() {
    motor.set(power);
  }

  public void stop() {
    motor.set(0.0);
  }

  public boolean getBall1IsSeen() {
    return ball1IsSeen.get();
  }

  public boolean getBall2IsSeen() {
    return ball2IsSeen.get();
  }
  
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake ball sensor 1", getBall1IsSeen());
    SmartDashboard.putBoolean("intake ball sensor 2", getBall2IsSeen());
  }
}
