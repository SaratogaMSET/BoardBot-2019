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

    public Drivetrain() {
        //Initialize the SPARK vars
        MotorL1 = new CANSparkMax(RobotMap.DRIVETRAIN_L1.value, MotorType.kBrushless);
        MotorL2 = new CANSparkMax(RobotMap.DRIVETRAIN_L2.value, MotorType.kBrushless);
        MotorR1 = new CANSparkMax(RobotMap.DRIVETRAIN_R1.value, MotorType.kBrushless);
        MotorR2 = new CANSparkMax(RobotMap.DRIVETRAIN_R2.value, MotorType.kBrushless);

        MotorL1.clearFaults();
        MotorL2.clearFaults();
        MotorR1.clearFaults();
        MotorR2.clearFaults();

        MotorR1.setInverted(true);
        MotorR2.setInverted(true);



    }

    public void drivetrainVals(double leftValue){
        MotorL1.set(leftValue);
        MotorL2.set(leftValue);
        MotorR1.set(leftValue);
        MotorR2.set(leftValue);
    }

    protected void initDefaultCommand() {
        //setDefaultCommand(new TankDrive());
    }
}
