package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class LaserPoint {
    static Point pos_navcam_before_rotation_astrobee_frame = new Point(0.1177, -0.0422, -0.0826);
    static Point pos_laserpointer_before_rotation_astrobee_frame = new Point(0.1302, 0.0572, -0.1111);


    static double[] calcTargetQuaternion_reverse(Point cur_pos) {

        Vector3Cal target_after_rotation_astrobee_frame_vector = new Vector3Cal(
                11.2625 - cur_pos.getX(),
                -10.58 - cur_pos.getY(),
                5.3625 - cur_pos.getZ());
        Point pos_target_before_rotation_astrobee_frame = new Point(
                -target_after_rotation_astrobee_frame_vector.getY(),
                target_after_rotation_astrobee_frame_vector.getX(),
                target_after_rotation_astrobee_frame_vector.getZ());
        Point pos_target_navcam_frame = new Point(
                pos_target_before_rotation_astrobee_frame.getX() - pos_navcam_before_rotation_astrobee_frame.getX(),
                pos_target_before_rotation_astrobee_frame.getY() - pos_navcam_before_rotation_astrobee_frame.getY(),
                pos_target_before_rotation_astrobee_frame.getZ() - pos_navcam_before_rotation_astrobee_frame.getZ()
        );
        return new double[]{pos_target_navcam_frame.getY(), pos_target_navcam_frame.getZ(), pos_target_navcam_frame.getX()};
    }


    static Point calcTargetPositionRelative(Point target_tvec_cam_frame) {


        Point pos_target_navcam_frame = new Point(
                target_tvec_cam_frame.getZ(),
                target_tvec_cam_frame.getX(),
                target_tvec_cam_frame.getY());

        Point pos_target_before_rotation_astrobee_frame = new Point(
                pos_target_navcam_frame.getX() + pos_navcam_before_rotation_astrobee_frame.getX(),
                pos_target_navcam_frame.getY() + pos_navcam_before_rotation_astrobee_frame.getY(),
                pos_target_navcam_frame.getZ() + pos_navcam_before_rotation_astrobee_frame.getZ());


        Point target = new Point(pos_target_before_rotation_astrobee_frame.getY(),
                -pos_target_before_rotation_astrobee_frame.getX(),
                pos_target_before_rotation_astrobee_frame.getZ());

        return target;
    }
}
