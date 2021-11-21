package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.XboxRocketLeagueDrive;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;


public class DriveBase 
{
    private MustangController mController;
    private DifferentialDrivePoseEstimator poseEstimator, visionPoseEstimator;  
    private DifferentialDrivePoseEstimator encoderOnlyPoseEstimator;

    private VisionCTRL VC = new VisionCTRL();
    private GyroCTRL GC = new GyroCTRL();
    private EncoderCTRL EC = new EncoderCTRL();
    private Constants C = new Constants();
        

    public DriveBase(MustangController mustangController) 
    {
        mController = mustangController;
        poseEstimator = new DifferentialDrivePoseEstimator(
            Rotation2d.fromDegrees(GC.getHeading()),
            new Pose2d(C.START_X, C.START_Y, new Rotation2d()), // TODO: change this to be a constant with the starting position
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01), // if u need to, find the correct values for the three vectors
            VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), 
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    
        encoderOnlyPoseEstimator = new DifferentialDrivePoseEstimator(
            Rotation2d.fromDegrees(GC.getHeading()),
            new Pose2d(C.START_X, C.START_Y, new Rotation2d()), // TODO: change this to be a constant with the starting position
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01), // if u need to, find the correct values for the three vectors
            VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), 
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        ); 
    
        visionPoseEstimator = new DifferentialDrivePoseEstimator(
            Rotation2d.fromDegrees(GC.getHeading()),
            new Pose2d(C.START_X, C.START_Y, new Rotation2d()), // TODO: change this to be a constant with the starting position
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01), // if u need to, find the correct values for the three vectors
            VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), 
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        ); 

    }

    public void initDefaultCommand() {
        MustangScheduler.getInstance().setDefaultCommand(this, new XboxRocketLeagueDrive(this, mController));
    }

    //importos
    @Override
    public boolean isQuickTurnPressed() {
      return mController.getRightBumper();
    }


    //setters
    public void resetOdometry(Pose2d pose) {
        GC.zeroHeading();
        poseEstimator.resetPosition(pose, Rotation2d.fromDegrees(GC.getHeading()));

        CANError lE = EC.left1Encoder.setPosition(0);
        CANError rE = EC.right1Encoder.setPosition(0);

        SmartDashboard.putString("Encoder return value left", lE.toString());
        SmartDashboard.putString("Encoder return value right", rE.toString());
        SmartDashboard.putNumber("Encoder positions left", EC.left1Encoder.getPosition()); 
        SmartDashboard.putNumber("Encoder positions left", EC.right1Encoder.getPosition()); 

        int counter = 0;
        while ((EC.left1Encoder.getPosition() != 0 || EC.right1Encoder.getPosition() != 0) && counter < 30) {
          lE = EC.left1Encoder.setPosition(0);
          rE = EC.right1Encoder.setPosition(0);
          counter++;
        }
    }

    public void resetOdometry() {
        GC.zeroHeading();
        EC.left1Encoder.setPosition(0);
        EC.right1Encoder.setPosition(0);

        poseEstimator = new DifferentialDrivePoseEstimator(Rotation2d.fromDegrees(GC.getHeading()),
            new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01), // TODO:
            VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), // TODO: find correct values
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // TODO: find correct values
      }


    //getter/infos
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public MustangController getMustangController() {
      return mController;
    }


    @Override
    public HealthState checkHealth() {
        HealthState state = HealthState.GREEN;
    
        CANError left1Error = EC.left1.getLastError();
        CANError left2Error = EC.left2.getLastError();
        CANError right1Error = EC.right1.getLastError();
        CANError right2Error = EC.right2.getLastError();
    
        boolean isLeft1Error = isSparkMaxErrored(EC.left1);
        boolean isLeft2Error = isSparkMaxErrored(EC.left2);
        boolean isRight1Error = isSparkMaxErrored(EC.right1);
        boolean isRight2Error = isSparkMaxErrored(EC.right2);
        boolean isNavXError = (GC.navXMicro == null);
    
        // used to check if it is green first which would be the case most of the times.
        // Then red as it is just 4 conditions and
        // finally yellow using else as it has many conditions to check for yellow
        if (!isLeft1Error && !isLeft2Error && !isRight1Error && !isRight2Error && !isNavXError) {
          state = HealthState.GREEN;
        } else if (isLeft1Error && isLeft2Error || isRight1Error && isRight2Error) {
          state = HealthState.RED;
          MustangNotifications.reportError("RED Errors: l1: %s, l2: %s, r1: %s, r2: %s", left1Error, left2Error,
              right1Error, right2Error);
        } else {
          state = HealthState.YELLOW;
          MustangNotifications.reportError("YELLOW Errors: l1: %s, l2: %s, r1: %s, r2: %s, navX: %s", left1Error,
              left2Error, right1Error, right2Error, isNavXError);
        }
        return state;
    }

}



class GyroCTRL
{
    public NavX navXMicro;

    public GyroCTRL()
    {
        navXMicro = new NavX(RobotMap.NAVX_PORT);
    }

    public double getHeading() {
        return Math.IEEEremainder(navXMicro.getAngle(), 360) * (RobotConstants.kNavXReversed ? -1. : 1.);
    }

    @Override
    public void zeroHeading() {
      navXMicro.reset();
    }



}

class VisionCTRL extends Constants
{
    private PhotonCamera camera;
    
    public VisionCTRL()
    {
        camera = new PhotonCamera(RobotConstants.TURRET_CAMERA_NAME);
    }

    public Pose2d getVisionPose(PhotonPipelineResult res) {
        Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget().inverse();
        Pose2d targetOffset = CAMERA_OFFSET.transformBy(camToTargetTrans);
        return targetOffset;
    } 

    public double getVisionCaptureTime(PhotonPipelineResult res) {
        return Timer.getFPGATimestamp() - res.getLatencyMillis();
    }




}

abstract class Constants
{
    public static final double CURRENT_WHEN_AGAINST_BAR = 5; // TODO Find this
    public int againstBarCount = 0;
  
    public static final double START_X = 2.4;
    public static final double START_Y = 15.983 - 3.8;

    public static final Pose2d TARGET_POSE = new Pose2d(2.4, 15.983, Rotation2d.fromDegrees(0));
    public static final Pose2d CAMERA_OFFSET = TARGET_POSE.transformBy(
        new Transform2d(
            new Translation2d(0, 2.3), 
            Rotation2d.fromDegrees(0)
        )
    );

    public Constants(){

    }

    public DifferentialDriveKinematics getKDriveKinematics() {
      return RobotConstants.kDriveKinematics;
    }
  
    public PIDController getLeftPIDController() {
      return new PIDController(RobotConstants.leftKPDriveVel, RobotConstants.leftKIDriveVel,
          RobotConstants.leftKDDriveVel);
    }
  
    public SimpleMotorFeedforward getLeftSimpleMotorFeedforward() {
      return new SimpleMotorFeedforward(RobotConstants.leftKsVolts, RobotConstants.leftKvVoltSecondsPerMeter,
          RobotConstants.leftKaVoltSecondsSquaredPerMeter);
    }
  
    public PIDController getRightPIDController() {
      return new PIDController(RobotConstants.rightKPDriveVel, RobotConstants.rightKIDriveVel,
          RobotConstants.rightKDDriveVel);
    }
  
    public SimpleMotorFeedforward getRightSimpleMotorFeedforward() {
      return new SimpleMotorFeedforward(RobotConstants.rightKsVolts, RobotConstants.rightKvVoltSecondsPerMeter,
          RobotConstants.rightKaVoltSecondsSquaredPerMeter);
    }
}
  
 


class EncoderCTRL extends TankDriveBase
{
    public SparkMAXLite left1, left2, right1, right2;
    public CANEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;

    public List<SparkMAXLite> leftControllers, rightControllers;
    public List<SparkMAXLite> allMotors = new ArrayList<SparkMAXLite>();

    public EncoderCTRL()
    {
        leftControllers = SparkMAXFactory.buildFactorySparkMAXPair(
            RobotMap.SPARK_LEFT_MOTOR_1, 
            RobotMap.SPARK_LEFT_MOTOR_2,
            false, 
            MotorConfig.Motor_Type.NEO
        );
        
        rightControllers = SparkMAXFactory.buildFactorySparkMAXPair(
            RobotMap.SPARK_RIGHT_MOTOR_1,
            RobotMap.SPARK_RIGHT_MOTOR_2,
            false,
            MotorConfig.Motor_Type.NEO
        );
    
        left1 = leftControllers.get(0); left1Encoder = left1.getEncoder();
        left2 = leftControllers.get(1); left2Encoder = left2.getEncoder();

        right1 = rightControllers.get(0); right1Encoder = right1.getEncoder();
        right2 = rightControllers.get(1); right2Encoder = right2.getEncoder();
 
    
   
        left1Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
        left2Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
        right1Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
        right2Encoder.setVelocityConversionFactor(RobotConstants.sparkMaxVelocityConversionFactor);
    
        left1Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
        left2Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
        right1Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
        right2Encoder.setPositionConversionFactor(RobotConstants.DRIVEBASE_METERS_PER_ROTATION);
    
        allMotors.addAll(leftControllers);
        allMotors.addAll(rightControllers);
   
        setMotorsInvert(leftControllers, false);
        setMotorsInvert(rightControllers, true); 

        super.setMotorControllers(new SpeedController[] { left1, left2 }, new SpeedController[] { right1, right2 }, false,false, .1, true);

    }


    //inits
    public void initBrakeMode() {
        setMotorsBrakeMode(allMotors, IdleMode.kBrake);
    }

    public void initCoastMode() {
        setMotorsNeutralMode(IdleMode.kCoast);
    }
 

    //getters
    public double getRobotInputVoltage() {
        double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
        return output;
    }

    public double getRobotOutputVoltage() {
        double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
            + right2.getAppliedOutput();
        return output;
    }

    public double getLeftInputVoltage() {
        double output = left1.getBusVoltage() + left2.getBusVoltage();
        return output;
    }
    
    public double getRightInputVoltage() {
        double output = right1.getBusVoltage() + right2.getBusVoltage();
        return output;
    }

    public double getLeftOutputVoltage() {
        double output = left1.getAppliedOutput() + left2.getAppliedOutput();
        return output;
    }

    public double getRightOutputVoltage() {
        double output = right1.getAppliedOutput() + right2.getAppliedOutput();
        return output;
    }

    public double getLeftOutputCurrent() {
        double output = left1.getOutputCurrent() + left2.getOutputCurrent();
        return output;
    }

    public double getRightOutputCurrent() {
        double output = right1.getOutputCurrent() + right2.getOutputCurrent();
        return output;
    }

    public double getRobotOutputCurrent() {
        double output = left1.getOutputCurrent() + left2.getOutputCurrent() + right1.getOutputCurrent()
            + right2.getOutputCurrent();
        return output;
    }

    public CANEncoder getLeftMainEncoder() {
        return left1Encoder;
    }

    public CANEncoder getLeftFollowerEncoder() {
        return left2Encoder;
    }

    public CANEncoder getRightMainEncoder() {
        return right1Encoder;
    }    

    public CANEncoder getRightFollowerEncoder() {
        return right2Encoder;
    }

    public List<SparkMAXLite> getLeftControllers() {
        return leftControllers;
    }

    public List<SparkMAXLite> getRightControllers() {
        return rightControllers;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(left1Encoder.getVelocity(), right1Encoder.getVelocity());
    }
    
    @Override
    public double getLeftPositionTicks() {
      return (int) (left1Encoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
    }
  
    @Override
    public double getLeftVelocityTicks() {
      return (left1Encoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
    }

    @Override
    public double getRightPositionTicks() {
      return (int) (left1Encoder.getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
    }
  
    @Override
    public double getRightVelocityTicks() {
      return (right1Encoder.getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
    }
  



    //setters
    private void setMotorsInvert(List<SparkMAXLite> motorGroup, boolean invert) {
        for (CANSparkMax m : motorGroup) {
          m.setInverted(invert);
        }
    }

    public void setMotorsNeutralMode(IdleMode mode) {
        for (CANSparkMax m : allMotors) {
          m.setIdleMode(mode);
        }
    }
    
    public void setMotorsCoastMode(List<CANSparkMax> motorGroup, IdleMode mode) {
        for (CANSparkMax m : motorGroup) {
          m.setIdleMode(IdleMode.kCoast);
        }
    }
    
    public void setMotorsBrakeMode(List<SparkMAXLite> motorGroup, IdleMode mode) {
        for (CANSparkMax m : motorGroup) {
          m.setIdleMode(IdleMode.kBrake);
        }
    }

    public void setRampRate(List<SparkMAXLite> motors, double rampRate) {
        for (CANSparkMax m : motors) {
          m.setClosedLoopRampRate(rampRate);
          m.setOpenLoopRampRate(rampRate);
        }
    }
    
    public void setTeleopRampRate() {
        setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs
    }

    public void zeroSensors() {
        left1Encoder.setPosition(0);
        right1Encoder.setPosition(0);
    }


    public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
        left1.setVoltage(leftVoltage);
        right1.setVoltage(rightVoltage);
        getDriveTrain().feed(); // super
    }

    @Override
    public void setEncodersPositionControl(double leftPos, double rightPos) {
      left1Encoder.setPosition(leftPos);
      right1Encoder.setPosition(rightPos);
    }

    @Override
    public void setRampRate(double rampRate) {
      left1.setOpenLoopRampRate(rampRate);
      right1.setOpenLoopRampRate(rampRate);
    }

    @Override
    public void setVelocityControl(double leftSpeed, double rightSpeed) {
      left1.set(leftSpeed);
      right1.set(rightSpeed);
    }


    //unit changes:

    @Override
    public double ticksToInches(double ticks) {
      double rotations = ticks / RobotConstants.SPARK_TICKS_PER_ROTATION;
      return rotations * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER;
    }
  
    @Override
    public double inchesToTicks(double inches) {
      double rotations = inches / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
      return rotations * RobotConstants.SPARK_TICKS_PER_ROTATION;
    }


    //importos

    public boolean isAlignedOnFloorBars() {
        double backLeftCurrent = left2.getOutputCurrent();
        double backRightCurrent = right2.getOutputCurrent();
        if (backLeftCurrent > 0.2 && backRightCurrent > 0.2) {
          if (backLeftCurrent >= CURRENT_WHEN_AGAINST_BAR && backRightCurrent >= CURRENT_WHEN_AGAINST_BAR) {
            againstBarCount++;
          } else {
            againstBarCount = 0;
          }
          if (againstBarCount >= 4) { // 4 consecutive readings higher than peak
            return true;
          }
        }
        return false;
      }



    //other
    public void sendEncoderDataToDashboard() {
        SmartDashboard.putNumber("Left M Position Ticks", left1Encoder.getPosition());
        SmartDashboard.putNumber("Left M Velocity Ticks", left1Encoder.getVelocity());
        SmartDashboard.putNumber("Left S Position Ticks", left2Encoder.getPosition());
        SmartDashboard.putNumber("Left S Velocity Ticks", left2Encoder.getVelocity());
        SmartDashboard.putNumber("Right M Position Ticks", right1Encoder.getPosition());
        SmartDashboard.putNumber("Right M Velocity Ticks", right1Encoder.getVelocity());
        SmartDashboard.putNumber("Right S Position Ticks", right2Encoder.getPosition());
        SmartDashboard.putNumber("Right S Velocity Ticks", right2Encoder.getVelocity());
    }

}
