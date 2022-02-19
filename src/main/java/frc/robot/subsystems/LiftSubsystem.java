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
    DutyCycleEncoder winchEncoder = new DutyCycleEncoder(LIFT_WINCH_ENCODER);

    double liftTarget = 0.0;
    double liftTargetMin = 0.0;
    double liftTargetMax = 6.0;

    PIDController liftPID;
    double liftKp = 0.8;

    double winchTarget = 0.0;
    double winchTargetMin = 0.0;
    double winchTargetMax = 10.0;

    PIDController winchPID;
    double winchKp = 0.0;

    public LiftSubsystem() {
        leftEncoder.setConnectedFrequencyThreshold(900);
        rightEncoder.setConnectedFrequencyThreshold(900);
        winchEncoder.setConnectedFrequencyThreshold(900);

        liftPID = new PIDController(liftKp, 0.0, 0.0);
        winchPID = new PIDController(winchKp, 0.0, 0.0);

        SmartDashboard.putNumber("lift winch kp", winchKp);
        
        resetLift();
        resetWinch();
    }

    public void resetLift() {
        leftEncoder.reset();
        rightEncoder.reset();
        liftTarget = 0;
    }

    public void resetWinch() {
        winchEncoder.reset();
        winchTarget = 0;
    }

    public void up() {
        liftTarget += 0.1;
    }

    public void down() {
        liftTarget -= 0.1;
    }

    public void in() {
        winchTarget -= 0.1;
    }

    public void out() {
        winchTarget += 0.1;
    }

    @Override
    public void periodic() {
        if(liftTarget > liftTargetMax) liftTarget = liftTargetMax;
        if(liftTarget < liftTargetMin) liftTarget = liftTargetMin;
        if(winchTarget > winchTargetMax) winchTarget = winchTargetMax;
        if(winchTarget < winchTargetMin) winchTarget = winchTargetMin;

        SmartDashboard.putNumber("lift left encoder position", leftEncoder.get());
        SmartDashboard.putNumber("lift right encoder position", rightEncoder.get());
        SmartDashboard.putNumber("lift target", liftTarget);
        SmartDashboard.putNumber("lift winch position", winchEncoder.get());
        SmartDashboard.putNumber("lift winch target", winchTarget);

        double kp = SmartDashboard.getNumber("lift winch kp", winchKp);
        winchPID.setP(kp);

        double leftPower = liftPID.calculate(leftEncoder.get(), liftTarget);
        double rightPower = liftPID.calculate(rightEncoder.get(), liftTarget);
        double winchPower = winchPID.calculate(winchEncoder.get(), winchTarget);

        motor1.set(leftPower);
        motor2.set(rightPower);
        winch.set(winchPower);

        SmartDashboard.putNumber("lift left power", leftPower);
        SmartDashboard.putNumber("lift right power", rightPower);
        SmartDashboard.putNumber("winch power", winchPower);
    }
}
