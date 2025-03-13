package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot{


private static Robot instance;
private Command m_autonomousCommand;
private RobotContainer m_robotContainer;
private Timer disabledTimer;

private XboxController Driver;
private XboxController Operater;

private TalonFX BallMotorI;



//Motors that outtake the Coral//
private SparkMax CoralMotorL;
private SparkMax CoralMotorR;

private SparkMax BallMotorP;

//Human source Intake//
private SparkMax CSource;

//Lift the Motor//
private SparkMax LiftMotor;

private final DoubleSolenoid Endsolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
//private final DoubleSolenoid clawsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8,9);
//private final DoubleSolenoid Ballsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 10);
//private final DoubleSolenoid Armsolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4);

private boolean BallOut = false;
private boolean Processor = false;
private boolean ground = false;
private boolean End = false;

Servo leftServo = new Servo(7);
Servo rightServo = new Servo(8);


public Robot()
  {
    instance = this;
   

    Driver = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    Operater = new XboxController(Constants.OperatorConstants.kOperatorControllerPort);


    CoralMotorL = new SparkMax(Constants.OperatorConstants.CoralMotorL, MotorType.kBrushless);
    CoralMotorR = new SparkMax(Constants.OperatorConstants.CoralMotorR, MotorType.kBrushless);
    //BallMotorI= new SparkMax(Constants.OperatorConstants.BallMotorI, MotorType.kBrushless);
    BallMotorP = new SparkMax(Constants.OperatorConstants.BallMotorP, MotorType.kBrushless);
    CSource= new SparkMax(Constants.OperatorConstants.CSource, MotorType.kBrushless);
    LiftMotor= new SparkMax(Constants.OperatorConstants.LiftMotor, MotorType.kBrushed);
    BallMotorI = new TalonFX(Constants.OperatorConstants.BallMotorI);


  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    CameraServer.startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    Endsolenoid.set(Value.kReverse);
   // Armsolenoid.set(Value.kReverse);
    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(true);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){
  
      

      //elevator subsystem
      // // Set the motor speed based on trigger values
       if (Operater.getPOV() == 0) {
           // Move motor forward
           LiftMotor.set(-.95); // Scale speed down to 50%
       } else if (Operater.getPOV() == 180) {
           // Move motor backward
           LiftMotor.set(.98); // Scale speed down to 50%
       } else  {
           // Stop motor
           LiftMotor.set(0);
      }


      // Motors that push the coral to the reef

       // Control the motors based on bumper inputs
       if (Operater.getAButtonPressed()) {
      //      // Spin motors forward
           CoralMotorL.set(.5); // 80% speed forward
           CoralMotorR.set(-.5); // 80% speed forward

        } else if (Operater.getBButtonPressed()) {
           // Spin motors backward
           CoralMotorL.set(-.5); // 80% speed forward
           CoralMotorR.set(.5); // 80% speed forward
        } else if (Operater.getBButtonReleased() || Operater.getAButtonReleased()) {
           // Stop motors
         CoralMotorL.set(0);
         CoralMotorR.set(0);

            if (Operater.getRightBumperButtonPressed()) {
            //      // Spin motors forward
              BallMotorP.set(.5); // 80% speed forward
    
            } else if (Operater.getLeftBumperButtonPressed()) {
               // Spin motors backward
               BallMotorP.set(-.5); // 80% speed forward
            } else if (Operater.getRightBumperButtonReleased() || Operater.getLeftBumperButtonReleased()) {
               // Stop motors
               BallMotorP.set(0);
            }
       
            if (Operater.getRightTriggerAxis() > 0.1 ) {
              //      // Spin motors forward
                  BallMotorI.set(.5); // 80% speed forward
            } else if (Operater.getLeftTriggerAxis() > .1) {
                   // Spin motors backward
                   BallMotorI.set(-.5); // 80% speed forward
            } else if (Operater.getRightTriggerAxis() < .1 || Operater.getLeftTriggerAxis() < 0.1) {
                   // Stop motors
                   BallMotorI.set(0);
            }


       if (Operater.getXButtonPressed()) {
        //      // Spin motors forward
             CSource.set(-.8); // 80% speed forward
          }  else if ( Operater.getYButtonReleased() || Operater.getXButtonReleased()) { 
             // Stop motors
             CSource.set(0);
          }
        }
        }

        

    //on and off for hang cylinder



    //if(Operater.getRightBumperButtonPressed()){

      //End = !End;
  //}  
  //if (End){
    
    //Endsolenoid.set(Value.kForward);
   
//} else {
    //Endsolenoid.set(Value.kReverse);
//}
    
  
    
    //if(Operater.getYButtonPressed()){

      //BallOut = !BallOut;
  //}  
  //if (BallOut){
    
    //clawsolenoid.set(Value.kForward);
   
//} else {
    //clawsolenoid.set(Value.kReverse);
//}

    

  //if(Operater.getXButtonPressed()){

      //Processor = !Processor;
  //}  
  //if (Processor){
    
    //Ballsolenoid.set(Value.kForward);
   
//} else {
    //Ballsolenoid.set(Value.kReverse);
//}

//if(Operater.getBButtonPressed()){

  //ground = !ground;
//}  
//if (ground){

//Armsolenoid.set(Value.kForward);

//} else {
//Armsolenoid.set(Value.kReverse);
//}




   

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

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