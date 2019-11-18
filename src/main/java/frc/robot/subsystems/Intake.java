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
public class Intake extends Subsystem{
    private TalonSRX leftMotor;
    //private TalonSRX LeftMotorFollower;
    private TalonSRX rightMotor;
    //private TalonSRX RightMotorFollower;
    private TalonSRX cargoDeploy;
    private TalonSRX intakeMotor;
    private Solenoid intakeSolenoid;

    public Intake() {
        leftMotor = new TalonSRX(RobotMap.LEFT_INTAKE.value);
        rightMotor = new TalonSRX(RobotMap.RIGHT_INTAKE.value);
        cargoDeploy = new TalonSRX(RobotMap.CARGO_DEPLOY.value);
        intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR.value);
        intakeSolenoid = new Solenoid(RobotMap.PCM.value, RobotMap.SOLENOID.value);
        //RightMotorFollower = new TalonSRX(RobotMap.RIGHT_FOLLOW_MOTOR.value);
        //LeftMotorFollower = new Talon
        Robot.initTalon(leftMotor);
        Robot.initTalon(rightMotor);
        Robot.initTalon(cargoDeploy);
        Robot.initTalon(intakeMotor);
        //Robot.initTalon(LeftMotorFollower);
        //Robot.initTalon(RightMotorFollower);

        //LeftMotorFollower.follow(LeftMotor);
        //RightMotorFollower.follow(RightMotor);

        // RightMotor.setInverted(true);
        cargoDeploy.setInverted(true);
        leftMotor.setInverted(true);
        intakeMotor.setInverted(false);

    }

    public void setIntakeVals( ControlMode mode, double leftValue, double rightValue, boolean isIntakePressed, boolean isOuttakePressed){
        if(isIntakePressed){
            leftMotor.set(mode, 0.5);
            rightMotor.set(mode, 0.5);
            cargoDeploy.set(mode, 0.5);
            // IntakeMotor.set(mode, leftValue);
            intakeMotor.set(mode, 1);
            intakeSolenoid.set(true);
        } else if(isOuttakePressed){
            leftMotor.set(mode, -0.5);
            rightMotor.set(mode, -0.5);
            cargoDeploy.set(mode, -0.5);
            // IntakeMotor.set(mode, leftValue);
            intakeMotor.set(mode, 0);
            intakeSolenoid.set(true);
        } else {
            leftMotor.set(mode, 0);
            rightMotor.set(mode, 0);
            cargoDeploy.set(mode, 0);
            // IntakeMotor.set(mode, leftValue);
            intakeMotor.set(mode, 0);
            intakeSolenoid.set(false);
        }
        
    }

    protected void initDefaultCommand() {
        //setDefaultCommand(new TankDrive());
    }
}
