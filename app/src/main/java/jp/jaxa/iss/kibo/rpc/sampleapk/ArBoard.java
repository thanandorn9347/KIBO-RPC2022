package jp.jaxa.iss.kibo.rpc.sampleapk;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

class ArBoard {
    private List<Mat> objPoints1;
    private List<Mat> objPoints2;
    private MatOfInt board1ID;
    private MatOfInt board2ID;

    private List<MatOfPoint3f> offset_c;

    List<MatOfPoint3f> get_offset() {
        return offset_c;

    }

    List<Mat> getObjP1() {
        return objPoints1;
    }

    MatOfInt getBoard1ID() {
        return board1ID;
    }

    List<Mat> getObjP2() {
        return objPoints2;
    }

    MatOfInt getBoard2ID() {
        return board2ID;
    }

    void find_ROI3D(Mat rvec, Mat tvec) {

        MatrixUtils matrixUtils = new MatrixUtils();
        // define paper_corner offset from its center

        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot);
        double[][] offset_corner = {new double[]{0.075f, -0.075f, -0.075f, 0.075f},
                new double[]{0.0625f, 0.0625f, -0.0625f, -0.0625f}, new double[]{0f, 0f, 0f, 0f},};

        double[][] rotationMatrix = {new double[]{rot.get(0, 0)[0], rot.get(0, 1)[0], rot.get(0, 2)[0]},
                new double[]{rot.get(1, 0)[0], rot.get(1, 1)[0], rot.get(1, 2)[0]},
                new double[]{rot.get(2, 0)[0], rot.get(2, 1)[0], rot.get(2, 2)[0]}};

        double[][] global_offset = matrixUtils.multiplyMatrices(rotationMatrix, offset_corner);
        // c1------c0
        // | |
        // | |
        // c2------c3

        Point3 tar_c0 = new Point3(tvec.get(0, 0)[0] + global_offset[0][0],
                tvec.get(1, 0)[0] + global_offset[1][0], tvec.get(2, 0)[0] + global_offset[2][0]);

        Point3 tar_c1 = new Point3(tvec.get(0, 0)[0] + global_offset[0][1],
                tvec.get(1, 0)[0] + global_offset[1][1], tvec.get(2, 0)[0] + global_offset[2][1]);

        Point3 tar_c2 = new Point3(tvec.get(0, 0)[0] + global_offset[0][2],
                tvec.get(1, 0)[0] + global_offset[1][2], tvec.get(2, 0)[0] + global_offset[2][2]);

        Point3 tar_c3 = new Point3(tvec.get(0, 0)[0] + global_offset[0][3],
                tvec.get(1, 0)[0] + global_offset[1][3], tvec.get(2, 0)[0] + global_offset[2][3]);

        MatOfPoint3f offset_c0 = new MatOfPoint3f();
        offset_c0.fromArray(tar_c0);

        MatOfPoint3f offset_c1 = new MatOfPoint3f();
        offset_c1.fromArray(tar_c1);

        MatOfPoint3f offset_c2 = new MatOfPoint3f();
        offset_c2.fromArray(tar_c2);

        MatOfPoint3f offset_c3 = new MatOfPoint3f();
        offset_c3.fromArray(tar_c3);

        offset_c = new ArrayList<>();
        offset_c.add(offset_c0);
        offset_c.add(offset_c1);
        offset_c.add(offset_c2);
        offset_c.add(offset_c3);

    }

    void set_target_board1() {

        int[] id = new int[]{1, 2, 3, 4};
        board1ID = new MatOfInt();
        board1ID.fromArray(id);

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.075f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0625f, 0.0f));
        c0.add(new Point3(0.125f, 0.0125f, 0.0f));
        c0.add(new Point3(0.075f, 0.0125f, 0.0f));

        c1.add(new Point3(-0.125f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0625f, 0.0f));
        c1.add(new Point3(-0.075f, 0.0125f, 0.0f));
        c1.add(new Point3(-0.125f, 0.0125f, 0.0f));

        c2.add(new Point3(-0.125f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0125f, 0.0f));
        c2.add(new Point3(-0.075f, -0.0625f, 0.0f));
        c2.add(new Point3(-0.125f, -0.0625f, 0.0f));

        c3.add(new Point3(0.075f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0125f, 0.0f));
        c3.add(new Point3(0.125f, -0.0625f, 0.0f));
        c3.add(new Point3(0.075f, -0.0625f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        objPoints1 = new ArrayList<>();
        objPoints1.add(c_id0);
        objPoints1.add(c_id1);
        objPoints1.add(c_id2);
        objPoints1.add(c_id3);
    }

    void set_target_board2() {

        int[] id = new int[]{11, 12, 13, 14};
        board2ID = new MatOfInt();
        board2ID.fromArray(id);

        List<Point3> c0 = new ArrayList<>();
        List<Point3> c1 = new ArrayList<>();
        List<Point3> c2 = new ArrayList<>();
        List<Point3> c3 = new ArrayList<>();

        c0.add(new Point3(0.0875f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0665f, 0.0f));
        c0.add(new Point3(0.1375f, 0.0165f, 0.0f));
        c0.add(new Point3(0.0875f, 0.0165f, 0.0f));

        c1.add(new Point3(-0.1375f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0665f, 0.0f));
        c1.add(new Point3(-0.0875f, 0.0165f, 0.0f));
        c1.add(new Point3(-0.1375f, 0.0165f, 0.0f));

        c2.add(new Point3(-0.1375f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0165f, 0.0f));
        c2.add(new Point3(-0.0875f, -0.0665f, 0.0f));
        c2.add(new Point3(-0.1375f, -0.0665f, 0.0f));

        c3.add(new Point3(0.0875f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0165f, 0.0f));
        c3.add(new Point3(0.1375f, -0.0665f, 0.0f));
        c3.add(new Point3(0.0875f, -0.0665f, 0.0f));

        MatOfPoint3f c_id0 = new MatOfPoint3f();
        MatOfPoint3f c_id1 = new MatOfPoint3f();
        MatOfPoint3f c_id2 = new MatOfPoint3f();
        MatOfPoint3f c_id3 = new MatOfPoint3f();

        c_id0.fromList(c0);
        c_id1.fromList(c1);
        c_id2.fromList(c2);
        c_id3.fromList(c3);
        objPoints2 = new ArrayList<>();
        objPoints2.add(c_id0);
        objPoints2.add(c_id1);
        objPoints2.add(c_id2);
        objPoints2.add(c_id3);
    }

}
