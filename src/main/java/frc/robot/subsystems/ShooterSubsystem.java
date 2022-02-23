// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(SHOOTER_LEFT, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(SHOOTER_RIGHT, MotorType.kBrushless);
  CANSparkMax hoodmotor = new CANSparkMax(HOOD, MotorType.kBrushless);

  double hoodMin = 2.0;
  double hoodMax = 8.0;

  Encoder velocityEncoder = new Encoder(SHOOTER_ENCODER_1, SHOOTER_ENCODER_2);
  BangBangController controller = new BangBangController();
  double shooterKp = 0.0;
  PIDController shooterController;
  double setpoint = 2000.0;

  double kp = 0.4;
  PIDController hoodController;
  double hoodTarget = 2.2;

  Rev2mDistanceSensor hoodDistanceSensor;

  public ShooterSubsystem() {
    motor1.setInverted(false);
    motor2.setInverted(true);

    hoodController = new PIDController(kp, 0.0, 0.0);
    hoodDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    hoodDistanceSensor.setAutomaticMode(true);  

    SmartDashboard.putNumber("shooter target speed", setpoint);
    SmartDashboard.putNumber("hood kp", kp);
    
    SmartDashboard.putNumber("hood target", hoodTarget);

    shooterController = new PIDController(shooterKp, 0.0, 0.0);

    SmartDashboard.putNumber("shooter kp", shooterKp);
  }

  public void shoot() {
    setpoint = SmartDashboard.getNumber("shooter target speed", setpoint);

    double power = controller.calculate(getSpeed(), setpoint);
    // double power = shooterController.calculate(getSpeed(), setpoint);
    motor1.set(power);
    motor2.set(power);
  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
  }

  public double getSpeed() {
    return velocityEncoder.getRate() / 2048.0 * 60.0;
  }

  public void setSetpoint(double s) {
    SmartDashboard.putNumber("shooter target speed", s);
    setpoint = s;
  }

  public void setLowFar() {
    setSetpoint(1800.0);
  }

  public void setLowClose() {
    setSetpoint(1800.0);
  }

  public void setHighFar() {
    setSetpoint(2500.0);
  }

  public void setHighClose() {
    setSetpoint(2000.0);
  }

  public void setHighAuto() {
    setSetpoint(2300.0);
  }

  public double getHoodSensorValue() {
    return hoodDistanceSensor.getRange();
  }

  public void hoodOut() {
    hoodmotor.set(0.5);
  }

  public void hoodIn() {
    hoodmotor.set(-0.5);
  }

  public void hoodStop() {
    hoodmotor.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter speed", getSpeed());
    SmartDashboard.putNumber("hood distance sensor", getHoodSensorValue());

    kp = SmartDashboard.getNumber("hood kp", kp);
    hoodController.setP(kp);

    hoodTarget = SmartDashboard.getNumber("hood target", hoodTarget);

    if(hoodTarget < hoodMin) hoodTarget = hoodMin;
    if(hoodTarget > hoodMax) hoodTarget = hoodMax;
    hoodmotor.set(hoodController.calculate(getHoodSensorValue(), hoodTarget));

    shooterKp = SmartDashboard.getNumber("shooter kp", shooterKp);
    shooterController.setP(shooterKp);

    


  }
}