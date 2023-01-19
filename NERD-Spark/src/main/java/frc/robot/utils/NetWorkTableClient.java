package frc.robot.utils;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetWorkTableClient {
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = inst.getTable("photonvision");
        NetworkTable subTable = visionTable.getSubTable("photonvision");
        BooleanSubscriber targetFoundSub = subTable.getBooleanTopic("hasTarget").subscribe(false);
        DoubleArraySubscriber targetPoseSub = subTable.getDoubleArrayTopic("targetPose").subscribe(new double[] {});
    

    public boolean isTargetFound(){

        
            boolean hasTarget = targetFoundSub.get();
            double[] targetPose = targetPoseSub.get();

            SmartDashboard.putBoolean("NT Target Foynd", hasTarget);
            SmartDashboard.putNumberArray("NT Target Pose", targetPose);
            return hasTarget;
        
    
    }

}
