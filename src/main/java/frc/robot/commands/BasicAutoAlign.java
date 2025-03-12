public class BasicAutoAlign extends Command{
    private final CommandSwerveDrivetrain m_DriveTrain;
    private Pose2d targetPose;
    private Pose2d curPose;
    private ProfiledPIDController translationalPID;
    private ProfiledPIDController rotationalPID;
    private Optional<LeftOrRight> leftOrRight;
    public BasicAutoAlign(CommandSwerveDrivetrain drivetrain, LeftOrRight leftOrRight){
        m_DriveTrain = drivetrain;
        this.leftOrRight = leftOrRight;
        addRequirements(m_DriveTrain);
    }
    public void initialize(){        
        translationalPID = new ProfiledPIDController();

    }
    public void execute(){

    }
    public void end(){

    }
    public boolean isFinished(){

    }
    
}