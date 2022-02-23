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

  CANSparkMax motor = new CANSparkMax(INTAKE, MotorType.kBrushless);
  DigitalInput ball1IsSeen;
  DigitalInput ball2IsSeen;

  public IntakeSubsystem() {
    ball1IsSeen = new DigitalInput(6);
    ball2IsSeen = new DigitalInput(7);
  }

  public void in() {
    motor.set(-0.6);
  }

  public void out() {
    motor.set(0.6);
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
    SmartDashboard.putBoolean("intake distance sensor 1", getBall1IsSeen());
    SmartDashboard.putBoolean("intake distance sensor 2", getBall2IsSeen());
  }
}
