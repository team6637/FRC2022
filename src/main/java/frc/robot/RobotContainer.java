// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autos.Auton1;
import frc.robot.commands.autos.ShootThenTaxi;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    PixySubsystem m_pixySubsystem = new PixySubsystem();
    LiftSubsystem m_liftSubsystem = new LiftSubsystem();
    LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    Joystick stick = new Joystick(0);
    Joystick stick2 = new Joystick(1);

    // setup auton dropdown
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    public RobotContainer() {
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(-stick.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-stick.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-stick.getTwist()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> m_pixySubsystem.getX()
        ));
        m_limelightSubsystem.setupDriveMode();
        configureButtonBindings();

         // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("5 Ball!!", new Auton1(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
        m_chooser.addOption("Shoot Then Taxi", new ShootThenTaxi());

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    private void configureButtonBindings() {

        // shoot
        new JoystickButton(stick, 1).whenHeld(
            new ShootCommand(m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem)
        );
                    
        // Button 2 - intake in
        new JoystickButton(stick, 2).whileHeld(new RunCommand(m_intakeSubsystem::in, m_intakeSubsystem)).whenReleased(new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem));
        
        // Button 3  - intake and indexer out
        new JoystickButton(stick, 3).whileHeld(
             new ParallelCommandGroup(
                new RunCommand(m_intakeSubsystem::out, m_intakeSubsystem),
                new RunCommand(m_indexerSubsystem::out, m_indexerSubsystem)
             )
         ).whenReleased(
             new ParallelCommandGroup(
                new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem),
                new InstantCommand(m_indexerSubsystem::stop, m_indexerSubsystem)
             )
         );

        // Button 4 = intake to indexer to shooter
        new JoystickButton(stick, 4).whileHeld(
            new ParallelCommandGroup(
                new RunCommand(m_indexerSubsystem::in, m_indexerSubsystem),
                new RunCommand(m_intakeSubsystem::in, m_intakeSubsystem)
            )
        ).whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(m_indexerSubsystem::stop, m_indexerSubsystem),
                new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem)
            )
        );

        // reset gyro
        new JoystickButton(stick, 6).whenPressed(
            new ParallelCommandGroup(    
                new InstantCommand(m_drivetrainSubsystem::zeroGyroscope, m_drivetrainSubsystem),
                new InstantCommand(()->m_drivetrainSubsystem.resetOdometry(new Pose2d()))
            )
        );

        // set shooter pos low close
        new JoystickButton(stick, 7).whenPressed(new InstantCommand(m_shooterSubsystem::setLowClose, m_shooterSubsystem));

        // set shooter pos high close
        new JoystickButton(stick, 8).whenPressed(new InstantCommand(m_shooterSubsystem::setHighClose, m_shooterSubsystem));

        // set shooter pos low far
        new JoystickButton(stick, 9).whenPressed(new InstantCommand(m_shooterSubsystem::setLowFar, m_shooterSubsystem));

        // set shooter pos high far
        new JoystickButton(stick, 10).whenPressed(new InstantCommand(m_shooterSubsystem::setHighFar, m_shooterSubsystem));

        // hold pixy sensor
        new JoystickButton(stick, 11).whileHeld(new RunCommand(() -> m_pixySubsystem.activate(getAllianceColor()), m_pixySubsystem))
        .whenReleased(new InstantCommand(m_pixySubsystem::deactivate, m_pixySubsystem));

        // auto aim
        new JoystickButton(stick, 12).whenHeld(
            new AutoAim(m_limelightSubsystem, m_drivetrainSubsystem, m_shooterSubsystem)
        );

        // initial acquire
        new JoystickButton(stick2, 3).whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(()-> m_liftSubsystem.setLiftArmsSetpoint(4.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setWinchSetPoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setLiftArmsSetpoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(() -> m_liftSubsystem.setWinchSetPoint(19.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(() -> m_liftSubsystem.setLiftArmsSetpoint(1.8), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint)
            )
        ).whenReleased(
            new InstantCommand(m_liftSubsystem::stopLiftAndWinch, m_liftSubsystem)
        );

        // detach
        new JoystickButton(stick2, 5).whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_liftSubsystem.setLiftArmsSetpoint(1.8), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setWinchSetPoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setLiftArmsSetpoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint)

            )

        ).whenReleased(
            new InstantCommand(m_liftSubsystem::stopLiftAndWinch, m_liftSubsystem)
        );

        // extendo
        new JoystickButton(stick2, 6).whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_liftSubsystem.setWinchSetPoint(62.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setLiftArmsSetpoint(6.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setWinchSetPoint(100.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint)

            )

        ).whenReleased(
            new InstantCommand(m_liftSubsystem::stopLiftAndWinch, m_liftSubsystem));

        // acquire
        new JoystickButton(stick2, 4).whenPressed(
            new SequentialCommandGroup(
                new InstantCommand(() -> m_liftSubsystem.setLiftArmsSetpoint(1.8), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setWinchSetPoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(()-> m_liftSubsystem.setLiftArmsSetpoint(0.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint),

                new InstantCommand(() -> m_liftSubsystem.setWinchSetPoint(19.0), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isWinchAtSetpoint),

                new InstantCommand(() -> m_liftSubsystem.setLiftArmsSetpoint(1.8), m_liftSubsystem),
                new WaitCommand(.1),
                new WaitCommand(2).withInterrupt(m_liftSubsystem::isLiftAtSetpoint)

            )
        ).whenReleased(
            new InstantCommand(m_liftSubsystem::stopLiftAndWinch, m_liftSubsystem));

        // lift up
        new JoystickButton(stick2, 9).whileHeld(new RunCommand(m_liftSubsystem::up, m_liftSubsystem));

        // lift down
        new JoystickButton(stick2, 11).whileHeld(new RunCommand(m_liftSubsystem::down, m_liftSubsystem));

        // winch in
        new JoystickButton(stick2, 12).whileHeld(new RunCommand(m_liftSubsystem::in, m_liftSubsystem));

        // winch out
        new JoystickButton(stick2, 10).whileHeld(new RunCommand(m_liftSubsystem::out, m_liftSubsystem));

        // lift - reset lift encoders to position 0
        //new JoystickButton(stick2, 2).whenPressed(new InstantCommand(m_liftSubsystem::resetLift, m_liftSubsystem));

    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband - ignore really low numbers
        value = deadband(value, 0.3);

        // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
        value = Math.copySign(value * value, value);

        return value;
    }

    // Get Alliance Color from throttle position of Jostick 2
    public String getAllianceColor() {
        if(stick.getThrottle() > 0) {
            return "blue";
        } else {
            return "red";
        } 
    }

    // Check if the robot should drive with field relative orientation
    public Boolean driveIsFieldRelative() {
        if(stick2.getThrottle() > 0) {
            return true;
        } else {
            return false;
        } 
    }
    public void resetLift() {
        m_liftSubsystem.resetLift();
    }
    public void setLiftUsePID(boolean v) {
        m_liftSubsystem.setUsePID(v);
    }
}
