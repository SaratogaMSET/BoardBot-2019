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
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class Drivebase extends Subsystem{
    private TalonSRX LeftMotor;
    //private TalonSRX LeftMotorFollower;
    private TalonSRX RightMotor;
    //private TalonSRX RightMotorFollower;
    private TalonSRX CargoDeploy;
    private TalonSRX IntakeMotor;

    public Drivebase() {
        LeftMotor = new TalonSRX(RobotMap.LEFT_INTAKE.value);
        RightMotor = new TalonSRX(RobotMap.RIGHT_INTAKE.value);
        CargoDeploy = new TalonSRX(RobotMap.CARGO_DEPLOY.value);
        IntakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);
        //RightMotorFollower = new TalonSRX(RobotMap.RIGHT_FOLLOW_MOTOR.value);
        //LeftMotorFollower = new Talon
        Robot.initTalon(LeftMotor);
        Robot.initTalon(RightMotor);
        Robot.initTalon(CargoDeploy);
        Robot.initTalon(IntakeMotor);
        //Robot.initTalon(LeftMotorFollower);
        //Robot.initTalon(RightMotorFollower);

        //LeftMotorFollower.follow(LeftMotor);
        //RightMotorFollower.follow(RightMotor);

        RightMotor.setInverted(true);
        IntakeMotor.setInverted(true);

    }

    public void setIntake( ControlMode mode, double leftValue, double rightValue){
        LeftMotor.set(mode, leftValue);
        RightMotor.set(mode, leftValue);
        CargoDeploy.set(mode, leftValue);
        // IntakeMotor.set(mode, leftValue);
        IntakeMotor.set(mode, rightValue);
    }

    protected void initDefaultCommand() {
        //setDefaultCommand(new TankDrive());
    }
}
