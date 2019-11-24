/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ExecuteSubsystems extends Command{

    public ExecuteSubsystems(){
        requires(Robot.intake);
        requires(Robot.drivetrain);
        requires(Robot.gyro);
    }

    protected void initialized(){

    }

    protected void execute() {
        //This sets the values for the arcade Drivetrain

        //This sets the values for the Intake motors using the joysticks

        //.arcadeDrive(Robot.oi.getLeftJoyY(), Robot.oi.getRightJoyX());



        Robot.intake.setIntakeVals(
            ControlMode.PercentOutput,
            Robot.oi.getLeftJoyButtons(1),
            Robot.oi.getRightJoyButtons(1)
        );
        
        Robot.drivetrain.drivetrainVals(
            Math.abs(Robot.oi.getLeftJoyY()) > 0.2 ? Robot.oi.getLeftJoyY() * -0.5 : 0,
            Math.abs(Robot.oi.getRightJoyX()) > 0.2 ? Robot.oi.getRightJoyX() * -0.3 : 0,
            Robot.gyro.getYaw()
        );

        /*
        Robot.drivetrain.arcadeTest(
            Robot.oi.getRightJoyX() * throttleLeft
        );
        */
        SmartDashboard.putNumber("Heading", Robot.gyro.getYaw());
    }
    protected boolean isFinished(){
        return false;
    }

    protected void interrupted() {
        end();
    }
}