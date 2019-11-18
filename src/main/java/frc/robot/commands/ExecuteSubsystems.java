/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ExecuteSubsystems extends Command{

    public ExecuteSubsystems(){
        requires(Robot.intake);
        requires(Robot.drivetrain);
    }

    protected void initialized(){

    }

    protected void execute() {
        //This sets the values for the arcade Drivetrain

        //This sets the values for the Intake motors using the joysticks
        double throttleLeft = (1.0 - Robot.oi.LEFT_JOY.getThrottle())/-2.0;
        double throttleRight = (1.0 - Robot.oi.RIGHT_JOY.getThrottle())/-2.0;
        Robot.intake.setIntakeVals(ControlMode.PercentOutput, Robot.oi.getLeftJoyY()*throttleLeft, Robot.oi.getRightJoyY()*throttleRight, Robot.oi.getRightJoyTrigger());
    }
    protected boolean isFinished(){
        return false;
    }

    protected void interrupted() {
        end();
    }
}
