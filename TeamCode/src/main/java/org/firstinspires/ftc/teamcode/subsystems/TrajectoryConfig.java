package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;

public class TrajectoryConfig {
        private MecanumDrive rrdrive;
        PoseConfig poseConfig = new PoseConfig(rrdrive);
        public TrajectoryConfig(MecanumDrive autonDrive){
            this.rrdrive = autonDrive;
        }

        public Action ScoreChamberTrajectory(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
            double endTangent;

            endTangent = (allianceColor == AllianceColor.BLUE) ? 0 : 90;

            TrajectoryActionBuilder tab1 = rrdrive.actionBuilder(poseConfig.getInitialPose(allianceColor, fieldStartPosition))
                    .splineToLinearHeading(poseConfig.getChamberPose(allianceColor, fieldStartPosition), Math.toRadians(endTangent));

            return tab1.build();
        }
}
