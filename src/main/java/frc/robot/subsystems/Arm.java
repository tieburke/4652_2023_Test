package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;


public class Arm extends SubsystemBase{
    
    private CANSparkMax ElbowMotor = new CANSparkMax(Constants.ELBOW_DRIVE_PORT, MotorType.kBrushless);
    private CANSparkMax WristMotor = new CANSparkMax(Constants.WRIST_DRIVE_PORT, MotorType.kBrushless);

    private SparkMaxPIDController elbowController;
    private SparkMaxPIDController wristController;

    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;

    public Arm(){

        ElbowMotor.restoreFactoryDefaults();
        WristMotor.restoreFactoryDefaults();


        elbowController = ElbowMotor.getPIDController();
        wristController = WristMotor.getPIDController();
    
        elbowEncoder = ElbowMotor.getEncoder();
        wristEncoder = WristMotor.getEncoder();    

        elbowController.setP(0.05);
        elbowController.setI(0);
        elbowController.setD(0);
        elbowController.setFF(0.1);
        elbowController.setOutputRange(-0.2, 0.2);

        wristController.setP(0.05);
        wristController.setI(0);
        wristController.setD(0);
        wristController.setFF(0.1);
        wristController.setOutputRange(-0.2, 0.2);

    }

    public void setElbowSpeed(double speed){
        ElbowMotor.set(speed);
    }

    public void setWristSpeed(double speed){
        WristMotor.set(speed);
    }

    public void setElbowToAngle(double angle){
        elbowController.setReference(angle, CANSparkMax.ControlType.kPosition);
        
        SmartDashboard.putNumber("ElbowGoalAngle", angle);
        SmartDashboard.putNumber("ElbowEncoder", elbowEncoder.getPosition());
        SmartDashboard.putNumber("ElbowMotorRate", elbowEncoder.getVelocity());
    }

    public void setWristToAngle(double angle){
        wristController.setReference(angle, CANSparkMax.ControlType.kPosition);
        
        SmartDashboard.putNumber("WristGoalAngle", angle);
        SmartDashboard.putNumber("WristEncoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("WristEncoder", wristEncoder.getVelocity());
    }

}
