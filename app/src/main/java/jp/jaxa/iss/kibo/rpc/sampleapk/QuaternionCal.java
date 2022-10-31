package jp.jaxa.iss.kibo.rpc.sampleapk;
import gov.nasa.arc.astrobee.types.Quaternion;
public class QuaternionCal {
    private float w;
    private float x;
    private float y;
    private float z;
    public  static QuaternionCal identity = new QuaternionCal(0,0,0,1);

    public String toString() {
        return w + " + " + x + "i + " + y + "j + " + z + "k";
    }

    public QuaternionCal(float ix, float iy, float iz, float iw) {
        this.w = iw;
        this.x = ix;
        this.y = iy;
        this.z = iz;
    }
    public double getW() {
        return w;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }


    // return the quaternion norm
    public double norm() {
        return Math.sqrt(w * w + x * x + y * y + z * z);
    }

    // return the quaternion conjugate
    public QuaternionCal conjugate() {
        return new QuaternionCal(-x, -y, -z, w);
    }

    // return a new Quaternion whose value is (this + b)
    public QuaternionCal plus(QuaternionCal b) {
        QuaternionCal a = this;
        return new QuaternionCal(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }

    // return a new Quaternion whose value is (this * b)
    public QuaternionCal times(QuaternionCal b) {
        QuaternionCal a = this;
        float y0 = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        float y1 = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        float y2 = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        float y3 = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
        return new QuaternionCal(y1, y2, y3, y0);
    }

    // return a new Quaternion whose value is the inverse of this
    public QuaternionCal inverse() {
        float d = w * w + x * x + y * y + z * z;
        return new QuaternionCal(-x / d, -y / d, -z / d, w / d);
    }

    private QuaternionCal divThis(double scale) {
        if (scale != 1) {
            double w0 = w/scale;
            double x0 = x/scale;
            double y0 = y/scale;
            double z0 = z/scale;
            return new QuaternionCal((float) x0,(float)y0,(float)z0,(float)w0);
        }
        return new QuaternionCal(x,y,z,w);
    }

    public QuaternionCal normalizeThis() {
        return divThis(this.norm());
    }

    // return a / b
    // we use the definition a * b^-1 (as opposed to b^-1 a)
    public QuaternionCal divides(QuaternionCal b) {
        QuaternionCal a = this;
        return a.times(b.inverse());
    }

    public Quaternion toQuaternion() {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }

    public static QuaternionCal cal_from_two_vectors(Vector3Cal A, Vector3Cal B) {
        Vector3Cal crossproduct_vec = Vector3Cal.crossProduct(A, B);

        float x = (float) crossproduct_vec.getX();
        float y = (float) crossproduct_vec.getY();
        float z = (float) crossproduct_vec.getZ();
        float w = (float) Math.sqrt((Math.pow(A.magnitude(), 2)) * (Math.pow(B.magnitude(), 2)) + Vector3Cal.dotProduct(A, B));
        QuaternionCal quaternion_cal = new QuaternionCal(x, y, z, w);
        quaternion_cal = quaternion_cal.normalizeThis();
        return quaternion_cal;

    }
    public static QuaternionCal LookAt(Vector3Cal _sourcePoint, Vector3Cal _destPoint)
    {   Vector3Cal sourcePoint=_sourcePoint;
        Vector3Cal destPoint=_destPoint;
        destPoint = destPoint.minus(sourcePoint);
        destPoint= destPoint.normalised();
        Vector3Cal forwardVector = destPoint;

        float dot = (float) Vector3Cal.dotProduct(Vector3Cal.forward, forwardVector);

        if (Math.abs(dot - (-1.0f)) < 0.000001f)
        {
            return new QuaternionCal((float) Vector3Cal.up.getX(), (float) Vector3Cal.up.getY(), (float) Vector3Cal.up.getZ(), 3.1415926535897932f);
        }
        if (Math.abs(dot - (1.0f)) < 0.000001f)
        {
            return QuaternionCal.identity;
        }

        float rotAngle = (float)Math.acos(dot);
        Vector3Cal rotAxis = Vector3Cal.crossProduct(Vector3Cal.forward, forwardVector);
        rotAxis = rotAxis.normalised();

        return CreateFromAxisAngle(rotAxis, rotAngle);
    }


    private static QuaternionCal CreateFromAxisAngle(Vector3Cal axis, float angle)
    {
        float halfAngle = angle * 0.5f;
        float s = (float)Math.sin(halfAngle);
        QuaternionCal q =new QuaternionCal((float)axis.getX() * s,(float)axis.getY() * s,(float)axis.getZ() * s,(float)Math.cos(halfAngle));
        q =q.normalizeThis();
        return q;
    }
}
