// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class LiftSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(LIFT_LEFT, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(LIFT_RIGHT, MotorType.kBrushless);
  //CANSparkMax winch = new CANSparkMax(WINCH, MotorType.kBrushless);

  DutyCycleEncoder leftEncoder = new DutyCycleEncoder(LEFT_LIFT_ENCODER);
  DutyCycleEncoder rightEncoder = new DutyCycleEncoder(RIGHT_LIFT_ENCODER);
  ///DutyCycleEncoder winchEncoder = new DutyCycleEncoder(3);

  public LiftSubsystem() {
    leftEncoder.setConnectedFrequencyThreshold(900);
    rightEncoder.setConnectedFrequencyThreshold(900);
    //winchEncoder.setConnectedFrequencyThreshold(900);
  }

  double power = 0.4;
  double winchPower = 0.4;

  public void up() {
    motor1.set(power);
    motor2.set(power);
  }

  public void down() {
    motor1.set(-power);
    motor2.set(-power);
  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
  }

  public void in() {
    //winch.set(winchPower);
  }

  public void out() {
    //winch.set(-winchPower);
  }

  public void stopwinch() {
    //winch.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("lift left encoder position", leftEncoder.get());
    SmartDashboard.putNumber("lift right encoder position", rightEncoder.get());
    //SmartDashboard.putNumber("winch encoder position", winchEncoder.get());
  }

}
