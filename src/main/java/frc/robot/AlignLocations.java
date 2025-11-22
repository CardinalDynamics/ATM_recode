package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlignLocations {
    
    public enum BranchSide{
        LEFT(new Translation2d(0.16, .47)),
        RIGHT(new Translation2d(0.16, .47)),
        CENTER(new Translation2d(0.0, .47));

        public Translation2d tagOffset;
        private BranchSide(Translation2d offsets) {
            tagOffset = offsets;
        }

        public BranchSide mirror(){
            switch (this) {
                case LEFT: return RIGHT;
                default: return LEFT;
            }
        }
    }

    public enum ReefSide{
        BLUEONE(18),
        BLUESIX(19),
        BLUEFIVE(20),
        BLUEFOUR(21),
        BLUETHREE(22),
        BLUETWO(17),
        REDONE(7),
        REDSIX(6),
        REDFIVE(11),
        REDFOUR(10),
        REDTHREE(9),
        REDTWO(8);

        public final Pose2d tagPose;

        private ReefSide(int side) {
            var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            tagPose = layout.getTagPose(side).get().toPose2d();
        }
    }
}
