/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ExecuteSubsystems;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem{
    //Create the SPARK vars

    public Drivetrain() {
        //Initialize the SPARK vars
    }

    public void DrivetrainVals( ControlMode mode, double leftValue, double rightValue, boolean intakeSolenoidValue){
        
    }

    protected void initDefaultCommand() {
        //setDefaultCommand(new TankDrive());
    }
}
