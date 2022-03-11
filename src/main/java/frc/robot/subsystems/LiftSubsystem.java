// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class LiftSubsystem extends SubsystemBase {

    private final PWMSparkMax motor1;
    private final PWMSparkMax motor2;
    private final DutyCycleEncoder leftEncoder;
    private final DutyCycleEncoder rightEncoder;

    private double liftTarget = 0.0;
    private double liftTargetMin = 0.0;
    private double liftTargetMax = 6.0;
    private boolean liftLeftWasReset = false;
    private boolean liftRightWasReset = false;
    private PIDController liftPID;

    private double liftKp = 0.8;

    private final CANSparkMax winch;

    private double winchSetpoint = 0.0;
    private double winchSetpointMin = 0.0;
    private double winchSetpointMax = 100;

    private boolean usePID = false;

    private final RelativeEncoder winchEncoder;

    private final PIDController winchPID;
    private final double winchKp = 0.2;

    private double leftPower = 0.0;
    private double rightPower = 0.0;
    private double winchPower = 0.0;

    public LiftSubsystem() {
        motor1 = new PWMSparkMax(LIFT_LEFT);
        motor2 = new PWMSparkMax(LIFT_RIGHT);

        winch = new CANSparkMax(WINCH, MotorType.kBrushless);
        winch.restoreFactoryDefaults();

        leftEncoder = new DutyCycleEncoder(LIFT_LEFT_ENCODER);
        rightEncoder = new DutyCycleEncoder(LIFT_RIGHT_ENCODER);
        leftEncoder.setConnectedFrequencyThreshold(900);
        rightEncoder.setConnectedFrequencyThreshold(900);

        liftPID = new PIDController(liftKp, 0.0, 0.0);
        liftPID.setTolerance(.2);

        winchPID = new PIDController(winchKp, 0.0, 0.0);
        winchPID.setTolerance(.2);
        winchEncoder = winch.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void resetLift() {
        leftEncoder.reset();
        rightEncoder.reset();
        liftTarget = 0;        
    }

    public void up() {
        liftTarget += 0.1;
    }

    public void down() {
        liftTarget -= 0.1;
    }

    public void in() {
        winchSetpoint -= 1.5;
    }

    public void out() {
        winchSetpoint += 1.5;
    }

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }

    public void setWinchSetPoint(double s) {
        winchSetpoint = s;
    }

    public boolean isWinchAtSetpoint() {
        return winchPID.atSetpoint();
    }

    public void setLiftArmsSetpoint(double s) {
        liftTarget = s;
    }

    public boolean isLiftAtSetpoint() {
        return liftPID.atSetpoint();
    }

    public void stopLiftAndWinch() {
        liftTarget = leftEncoder.get();
        winchSetpoint = getWinchPosition();
    }

    public void doPID() {
        if(liftTarget > liftTargetMax) liftTarget = liftTargetMax;
        if(liftTarget < liftTargetMin) liftTarget = liftTargetMin;

        if(winchSetpoint > winchSetpointMax) winchSetpoint = winchSetpointMax;
        if(winchSetpoint < winchSetpointMin) winchSetpoint = winchSetpointMin;

        leftPower = liftPID.calculate(leftEncoder.get(),  liftTarget);
        rightPower = liftPID.calculate(rightEncoder.get(),  liftTarget);
        winchPower = winchPID.calculate(getWinchPosition(), winchSetpoint);

        motor1.set(leftPower);
        motor2.set(rightPower);
        winch.set(winchPower);
        
        //SmartDashboard.putNumber("lift winch position", getWinchPosition());
        //SmartDashboard.putNumber("lift winch setpoint", winchSetpoint);
        //SmartDashboard.putNumber("winch power", winchPower);
        //SmartDashboard.putBoolean("winch at setpoint", isWinchAtSetpoint());
        //SmartDashboard.putBoolean("lift at setpoint", isLiftAtSetpoint());
    }

    public void setUsePID(boolean v) {
        usePID = v;
    }

    public void stopLift() {
        motor1.set(0.0);
        motor2.set(0.0);
        winch.set(0.0);
    }

    @Override
    public void periodic() {

        if(liftLeftWasReset == false && leftEncoder.get() != 0.0) {
            resetLift();
            liftLeftWasReset = true;
        }

        if(liftRightWasReset == false && rightEncoder.get() != 0.0) {
            resetLift();
            liftRightWasReset = true;
        }

        // turn on PID after an initial target was set
        if(liftTarget != 0 && usePID == false) {
            usePID = true;
        }

        doPID();
        // if(usePID) {
        //     doPID();
        // } else {
        //     stopLift();
        // }

        SmartDashboard.putNumber("lift left encoder position", leftEncoder.get());
        SmartDashboard.putNumber("lift right encoder position", rightEncoder.get());
        // SmartDashboard.putNumber("lift target", liftTarget);

        // SmartDashboard.putNumber("lift left power", leftPower);
        // SmartDashboard.putNumber("lift right power", rightPower);

        // SmartDashboard.putNumber("lift left temperature", motor1.getMotorTemperature());
        // SmartDashboard.putNumber("lift right temperature", motor2.getMotorTemperature());
        //SmartDashboard.putNumber("lift left output", motor1.getAppliedOutput());
        //SmartDashboard.putNumber("lift right output", motor2.getAppliedOutput());

        SmartDashboard.putNumber("winch position", getWinchPosition());
    }
}
