/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 * Add your docs here.
 */
public class Encoders extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double COUNTS_PER_REV = 4096;

  public static TalonSRX leftEncoder;
  public static TalonSRX rightEncoder;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Encoders(){
    leftEncoder = new TalonSRX(23);
    rightEncoder = new TalonSRX(14);

    configEncoder(leftEncoder);
    configEncoder(rightEncoder);
  }

  public int getLeftAbsolute(){
    return leftEncoder.getSensorCollection().getPulseWidthPosition();
  }

  public double getLeftAngle(){
    return leftEncoder.getSensorCollection().getPulseWidthPosition()/COUNTS_PER_REV*360;
  }

  public int getLeftRelative(){
    return leftEncoder.getSensorCollection().getQuadraturePosition();
  }

  public double getLeftRotations(){
    return leftEncoder.getSensorCollection().getQuadraturePosition()/COUNTS_PER_REV;
  }
  
  public int getRightAbsolute(){
    return rightEncoder.getSensorCollection().getPulseWidthPosition();
  }

  public double getRightAngle(){
    return rightEncoder.getSensorCollection().getPulseWidthPosition()/COUNTS_PER_REV*360;
  }

  public int getRightRelative(){
    return rightEncoder.getSensorCollection().getQuadraturePosition();
  }

  public double getRightRotations(){
    return rightEncoder.getSensorCollection().getQuadraturePosition()/COUNTS_PER_REV;
  }
  
  private static void configEncoder(TalonSRX encoder) {
    encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    encoder.getSensorCollection().setQuadraturePosition(0, 10);
    encoder.setSelectedSensorPosition(0);
  }
}
