package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

    private final DoubleSolenoid clawToggle;
    private final DoubleSolenoid pressureToggle;

    private boolean pressure;

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
        elbowController.setOutputRange(-0.3, 0.3);

        wristController.setP(0.05);
        wristController.setI(0);
        wristController.setD(0);
        wristController.setFF(0.1);
        wristController.setOutputRange(-0.2, 0.2);

        clawToggle = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3); 
        pressureToggle = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);

        clawToggle.set(DoubleSolenoid.Value.kReverse);
        pressureToggle.set(DoubleSolenoid.Value.kReverse);

        pressure = false;

        setEncoderCoversions();

        burnFlash();

    }

    public void setElbowSpeed(double speed){
        ElbowMotor.set(speed);
    }

    public void setWristSpeed(double speed){
        WristMotor.set(speed);
    }

    public void setElbowToAngle(double angle){
        elbowController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void setWristToAngle(double angle){
        wristController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void resetElbowAngle(){
        ElbowMotor.getEncoder().setPosition(0);
    }

    public void resetWristAngle(){
        WristMotor.getEncoder().setPosition(0);
    }

    public void clawToggle(){
        clawToggle.toggle();
    }

    public void pressureToggle(){
        pressureToggle.toggle();
        if (pressure){
            pressure = false;
        }
        else if (!pressure){
            pressure = true;
        }
    }

    public double getElbowPosition(){
        return elbowEncoder.getPosition();
    }

    public double getWristPosition(){
        return wristEncoder.getPosition();
    }

    public void setGearShift(boolean shift){
		if (shift){
		clawToggle.set(DoubleSolenoid.Value.kForward);
		}
		else{
			clawToggle.set(DoubleSolenoid.Value.kReverse);
		}
	}

    public void setEncoderCoversions(){
        elbowEncoder.setPositionConversionFactor((1.0 / 144) * 360.0); // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
        elbowEncoder.setVelocityConversionFactor(((1.0 / 144) * 360.0) / 60.0);
        //wristEncoder.setPositionConversionFactor((1.0 / 64) * 360.0); // We do 1 over the gear ratio because 1 rotation of the motor is < 1 rotation of the module
        //wristEncoder.setVelocityConversionFactor(((1.0 / 64) * 360.0) / 60.0);
      }

    public void resetEncoders(){
        elbowEncoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    public void burnFlash(){
        WristMotor.burnFlash();
        ElbowMotor.burnFlash();
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElbowEncoder", elbowEncoder.getPosition());
        SmartDashboard.putNumber("ElbowMotorRate", elbowEncoder.getVelocity());
        SmartDashboard.putNumber("WristEncoder", wristEncoder.getPosition());
        SmartDashboard.putNumber("WristMotorRate", wristEncoder.getVelocity());
        SmartDashboard.putBoolean("pressure", pressure);
    }

}
