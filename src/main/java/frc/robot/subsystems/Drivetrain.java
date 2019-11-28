/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ExecuteSubsystems;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem{
    //Create the SPARK vars
    private CANSparkMax MotorL1;
    private CANSparkMax MotorL2;
    private CANSparkMax MotorR1;
    private CANSparkMax MotorR2;
    private double stayAt = 0;

    public Drivetrain() {
        //Initialize the SPARK vars
        MotorL1 = new CANSparkMax(RobotMap.DRIVETRAIN_L1.value, MotorType.kBrushless);
        MotorL2 = new CANSparkMax(RobotMap.DRIVETRAIN_L2.value, MotorType.kBrushless);
        //SpeedControllerGroup L = new SpeedControllerGroup(MotorL1, MotorL2);

        MotorR1 = new CANSparkMax(RobotMap.DRIVETRAIN_R1.value, MotorType.kBrushless);
        MotorR2 = new CANSparkMax(RobotMap.DRIVETRAIN_R2.value, MotorType.kBrushless);
        //SpeedControllerGroup R = new SpeedControllerGroup(MotorR1, MotorR2);

        //DifferentialDrive drive = new DifferentialDrive(L, R);

        MotorL1.clearFaults();
        MotorL2.clearFaults();
        MotorR1.clearFaults();
        MotorR2.clearFaults();

        MotorR1.setInverted(true);
        MotorR2.setInverted(true);



    }

    public void drivetrainVals(double leftValue, double rightValue, double veerVal){
        
        // SmartDashboard.putNumber("VEERVAL", veerVal);  
        // SmartDashboard.putNumber("LeftValue", leftValue);
        // SmartDashboard.putNumber("RightValue", rightValue);    
        SmartDashboard.putNumber("Total Veer", veerVal-stayAt);  
        double gainL = 0;
        double gainR = 0;
        if(rightValue == 0 && leftValue != 0){
            if(veerVal < stayAt-0.5){
                gainL = -veerVal;
            }
            if(veerVal > stayAt+0.5){
                gainR = -veerVal;
            }
        }
        stayAt = veerVal;
        if(leftValue == 0){
            Robot.gyro.reset();
        }
        
        MotorL1.set(leftValue - rightValue + gainL);
        MotorL2.set(leftValue - rightValue + gainL);
        MotorR1.set(leftValue + rightValue + gainR);
        MotorR2.set(leftValue + rightValue + gainR);  
    }

    public void arcadeTest(double rightValue){
        MotorL1.set(rightValue);
        MotorL2.set(rightValue);
        MotorR1.set(-rightValue);
        MotorR2.set(-rightValue);
    }

    protected void initDefaultCommand() {
        //setDefaultCommand(new TankDrive());
    }
}
