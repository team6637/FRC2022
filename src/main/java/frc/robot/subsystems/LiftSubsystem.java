// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class LiftSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(LIFT_LEFT, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(LIFT_RIGHT, MotorType.kBrushless);
  CANSparkMax winch = new CANSparkMax(WINCH, MotorType.kBrushless);

  DutyCycleEncoder leftEncoder = new DutyCycleEncoder(LIFT_LEFT_ENCODER);
  DutyCycleEncoder rightEncoder = new DutyCycleEncoder(LIFT_RIGHT_ENCODER);
  // DutyCycleEncoder winchEncoder = new DutyCycleEncoder(3);

  double liftTarget = 0;
  double liftTargetMin = 0.0;
  double liftTargetMax = 6.0;

  double liftLeftOffset;
  double liftRightOffset;

  PIDController liftPID;
  double kP = 0.8;

  public LiftSubsystem() {
    leftEncoder.setConnectedFrequencyThreshold(900);
    rightEncoder.setConnectedFrequencyThreshold(900);

    // winchEncoder.setConnectedFrequencyThreshold(900);

    liftPID = new PIDController(kP, 0, 0);
    reset();
  }

  public void reset() {
    leftEncoder.reset();
    rightEncoder.reset();
    liftTarget = 0;
  }

  double winchPower = 0.4;

  public void up() {
    liftTarget += 0.1;
  }

  public void down() {
    liftTarget -= 0.1;
  }

  public void in() {
    winch.set(winchPower);
  }

  public void out() {
    winch.set(-winchPower);
  }

  public void stopwinch() {
    winch.set(0.0);
  }

  @Override
  public void periodic() {
    if(liftTarget > liftTargetMax) liftTarget = liftTargetMax;
    if(liftTarget < liftTargetMin) liftTarget = liftTargetMin;

    SmartDashboard.putNumber("lift left encoder position", leftEncoder.get());
    
    SmartDashboard.putNumber("lift right encoder position", rightEncoder.get());

    
    SmartDashboard.putNumber("lift target", liftTarget);

    double leftPower = liftPID.calculate(leftEncoder.get(), liftTarget);
    double rightPower = liftPID.calculate(rightEncoder.get(), liftTarget);

    SmartDashboard.putNumber("lift left power", leftPower);
    SmartDashboard.putNumber("lift right power", rightPower);

    motor1.set(leftPower);
    motor2.set(rightPower);
  }

}
