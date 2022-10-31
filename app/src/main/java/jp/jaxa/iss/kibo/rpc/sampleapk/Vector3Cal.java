package jp.jaxa.iss.kibo.rpc.sampleapk;

public final class Vector3Cal {
    private double x,y,z;
    public static Vector3Cal up = new Vector3Cal(0,0,-1);
    public static Vector3Cal forward = new Vector3Cal(1,0,0);
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public Vector3Cal(double ix, double iy, double iz) {
        x = ix;
        y = iy;
        z = iz;
    }

    public void set(double ix, double iy, double iz) {
        x = ix;
        y = iy;
        z = iz;
    }
    public Vector3Cal plus(Vector3Cal B){
        double x0= x +B.getX();
        double y0= y +B.getY();
        double z0= z +B.getZ();
        return new Vector3Cal(x0,y0,z0);
    }
    public Vector3Cal minus(Vector3Cal B){

        double x0= x -B.getX();
        double y0= y -B.getY();
        double z0= z -B.getZ();
        return new Vector3Cal(x0,y0,z0);
    }
    public double magnitude() {
        return Math.sqrt(x*x+y*y+z*z);
    }

    public Vector3Cal multiply(double f) {
        double x0= x *f;
        double y0= y *f;
        double z0= z * f;
        return new Vector3Cal(x0,y0,z0);
    }
    public String toString(){
        return "x:"+x+"y:"+y+"z:"+z;
    }

    public Vector3Cal normalised() {
        double mag = magnitude();
        if (mag==0)
            return new Vector3Cal(x,y,z);
        double x0 = x/ mag;
        double y0 = y/mag;
        double z0 = z/mag;
        return new Vector3Cal(x0,y0,z0);
    }
    public  static Vector3Cal crossProduct (Vector3Cal A, Vector3Cal B){
        double cp_x,cp_y,cp_z;
        cp_x = A.getY()*B.getZ() -A.getZ()*B.getY();
        cp_y = A.getZ()*B.getX()-A.getX()*B.getZ();
        cp_z =A.getX()*B.getY() - A.getY()*B.getX();
        Vector3Cal cross_product = new Vector3Cal(cp_x,cp_y,cp_z);
        return  cross_product;
    }
    public  static  double dotProduct(Vector3Cal A, Vector3Cal B){
        double product = 0;
        product += A.getX()*B.getX();
        product += A.getY()*B.getY();
        product += A.getZ()*B.getZ();
        return product;
    }


}