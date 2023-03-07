
package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Encoder encoderJoint1;
  private Encoder encoderJoint2;
  private VictorSP motorJoint1a;
  private VictorSP motorJoint1b;
  private Spark motorJoint2;
  private PIDController pidJoint1;
  private PIDController pidJoint2;
  private double targetPositionJoint1 = 0.0;
  private double targetPositionJoint2 = 0.0;
  private XboxController xboxController;

  private final double Kp = 0.1;
  private final double Ki = 0.0;
  private final double Kd = 0.0;

  
  



  
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    xboxController = new XboxController(0);
    encoderJoint1 = new Encoder(0, 1); 
    encoderJoint1.setDistancePerPulse(0.01); 
    encoderJoint2 = new Encoder(2, 3); 
    encoderJoint2.setDistancePerPulse(0.01);

    motorJoint1a = new VictorSP(1); 
    motorJoint1b = new VictorSP(2); 
    motorJoint2 = new Spark(3); 

    pidJoint1 = new PIDController(Kp, Ki, Kd);
    pidJoint1.setTolerance(0.01);
    pidJoint2 = new PIDController(Kp, Ki, Kd);
    pidJoint2.setTolerance(0.01);
  }


  


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    targetPositionJoint1 = encoderJoint1.getDistance() + 50.0 / 2.0 / Math.PI / 0.2; //  2*pi*r
    targetPositionJoint2 = encoderJoint2.getDistance() + 50.0 / 2.0 / Math.PI / 0.2; 
    pidJoint1.setSetpoint(targetPositionJoint1);
    pidJoint2.setSetpoint(targetPositionJoint2);
    pidJoint1.reset();
    pidJoint2.reset();
   
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    
    if (xboxController.getXButton()) {
    
    
    pidJoint1.setSetpoint(targetPositionJoint1);
    pidJoint2.setSetpoint(targetPositionJoint2);
    double outputJoint1 = pidJoint1.calculate(encoderJoint1.getDistance());
    double outputJoint2 = pidJoint2.calculate(encoderJoint2.getDistance());
    
 
    motorJoint1a.set(-outputJoint1);
    motorJoint1b.set(outputJoint1);
    motorJoint2.set(outputJoint2);

    
      if(targetPositionJoint1 < pidJoint1.calculate(encoderJoint1.getDistance())) {

      motorJoint1a.set(outputJoint1 * 0.9);
      motorJoint1b.set(- outputJoint1 * 0.9);
  
      }

      if(targetPositionJoint1 > pidJoint1.calculate(encoderJoint1.getDistance())) {

        motorJoint1a.set(-outputJoint1 * 0.9);
        motorJoint1b.set( outputJoint1 * 0.9);
    
        }

      if(targetPositionJoint2 > pidJoint2.calculate(encoderJoint2.getDistance())) {

          motorJoint2.set( outputJoint2 * 0.9);
      
          }
      if(targetPositionJoint2 > pidJoint2.calculate(encoderJoint2.getDistance())) {

            motorJoint2.set(outputJoint2 * 0.9);
        
            }
        
      if(targetPositionJoint2 < pidJoint2.calculate(encoderJoint2.getDistance())) {

              motorJoint2.set(-outputJoint2 * 0.9);
           
          
              }
          
    
    
    
      if(targetPositionJoint1 == pidJoint1.calculate(encoderJoint1.getDistance())) {

          motorJoint1a.set(0);
          motorJoint1b.set(0);

    }

      if(targetPositionJoint2 == pidJoint2.calculate(encoderJoint2.getDistance())) {

        motorJoint2.set(0);
  }
  



     
  
  }


    else {
    motorJoint1a.set(0);
    motorJoint1b.set(0);
    motorJoint2.set(0);

    }
   
   
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
