package coordinates;

import android.util.Log;

import java.util.ArrayList;
import java.util.List;

public class Coordinates {
    String TAG="Coordinates";
   public double lat,lon;
    public boolean include=false,alwaysinclude;
    private int totalIncluded;
    private  static  double r;
    public Coordinates(double lat,double lon){
        this.lat=lat;
        this.lon=lon;
    }
    public Coordinates(){

    }

    public  static double getDistance(){
        return r;
    }
    public  static void setDistance(double distance){
        r= distance;
    }

    /*Function which create a grid mission according to 4 corner which they created on the map
     *
     *@param lat Array which contains the four corner latitudes
     *@param lon     Array which contains the four corner longitudes
     *@param fov The field of view of the camera drone
     *@param height This the height of the drone
     * @return List This return the coordinates which drone is going to follow
     */
    public List<Coordinates> create_grid(double lat[], double lon[], float fov, float height) {


        List<Coordinates> drone_move = new ArrayList<>();

        r = 2.0 * height * Math.tan(Math.toRadians(fov / 2.0));

        //r =100;
        Log.e(TAG, "r " + r);
        long over = Math.round(r * 0.1);

        Log.e(TAG, "r over" + r);
        Coordinates c3 = new Coordinates(lon[1], lat[0]);
        Coordinates c4 = new Coordinates(lon[0], lat[1]);
        Coordinates drone_point[][] = null;
        int drone_num[][] = null;

        int m = 2, n = 2;
{

            int xd = (int) (1000.0 * distFrom(new Coordinates(lon[1], lat[1]), new Coordinates(lon[0], lat[1])));
            int yd = (int) (1000.0 * distFrom(new Coordinates(lon[1], lat[1]), new Coordinates(lon[1], lat[0])));


            int x = (int) ((xd) / (r * 1.0));
            int y = (int) (yd / (r * 1.0));

//            Log.e(TAG, "mod x " +(xd % x) );
            //  Log.e(TAG, "mod y " +(yd%y) );


            Log.e(TAG, "xy " + x + " " + y);


            if (x > 0 && xd % x < 0.5 && xd % x != 0)
                x--;

            if (y > 0 && yd % y < 0.5 && yd % y != 0)
                y--;


            Log.e("Coordinates", "  after" + x + " " + y);

            if (x == 0 && y == 0) {

             /*   corner[0] = new Coordinates(lon[0], lat[0]);
                corner[1] = new Coordinates(lon[1], lat[1]);
                corner[2] = new Coordinates(lon[0], lat[1]);
                corner[3] = new Coordinates(lon[1], lat[0]);*/
                return null;

            }

            drone_point = new Coordinates[x + 2][y + 2];
            drone_num = new int[x + 2][y + 2];
            boolean flag = create_array(lat, lon, drone_point, drone_num, x, y, c3, c4);

            if (!flag)
                return null;

            if (drone_point[drone_point.length - 1][drone_point[0].length - 1] != null) {
                lat[1] = drone_point[drone_point.length - 1][drone_point[0].length - 1].lon;
                lon[1] = drone_point[drone_point.length - 1][drone_point[0].length - 1].lat;
            }
            ArrayList<Coordinates> drone_move_div[] = null;





            algorithm_path(drone_point, drone_num, height, drone_move);

        }


 /*       if (divide){
            drone_move=new LinkedList<>();
           Divide d=new Divide();
            Coordinates divided[][][]=d.divide_grid(drone_point,parts);
            if (divided!=null) {
               // drone_move_div=new List<>()[k];
                Log.d("Coordinates","divide algorithm ");
                int drone_numt[][][]=d.getNumArray();


                for (int i = 0; i < parts; i++) {
                    algorithm_path(divided[i], drone_numt[i], height, drone_move);
                    Log.d("Coordinates","drone_move size: "+drone_move.size());


                }
            }
            else {
                Log.d("Coordinates","cant divide");

                algorithm_path(drone_point, drone_num, height, drone_move);
            }

        }
        else{

          algorithm_path(drone_point, drone_num,height,drone_move);
        }*/


      /*  corner[0] = new Coordinates(lon[0], lat[0]);
        corner[1] = new Coordinates(lon[1], lat[1]);
        corner[2] = new Coordinates(lon[0], lat[1]);
        corner[3] = new Coordinates(lon[1], lat[0]);

*/

//        Log.e("AMALVALUES", drone_move + "");

        return drone_move;
    }


    /**
     * Create a path for polygon or grid mission using a two-dimensional array which contains the coordinates
     *
     * @param drone_point This is an array which contains Coordinates to calculate the path
     * @param drone_num-  This is an array which contains Coordinates to calculate the path
     * @return List Returns coordinates which is gooing ti follow drone on the mission
     */
    private  List<Coordinates> algorithm_path(Coordinates drone_point[][], int drone_num[][], float altitude, List<Coordinates> drone_move) {



        boolean[][] drone_flag = new boolean[drone_point.length][drone_point[0].length];
        int dm = 1;

        for (int i = 0; i < drone_flag.length; i++) {

            for (int j = 0; j < drone_flag[0].length; j++) {
                drone_flag[i][j] = true;
            }
        }

        int s = 0;
        int i = 0, j = 0, k = 0;
        int flag[][] = new int[2][2];
        Log.e(TAG, "dimensions: " + drone_point.length + "x " + drone_point[0].length);
        // mission_points.add(drone_point[0][0]);


        if (drone_point[0][0] != null) {
            drone_move.add(drone_point[0][0]);
            drone_move.get(0).include = true;
            totalIncluded++;
        }

        String arrayx = "", arrayy = "";
        while (check_array(drone_flag) && dm < drone_point.length * drone_point[0].length) {


            //Log.e(TAG, "flag " + drone_flag[i][j] );
            drone_flag[i][j] = false;
            flag = neighboor(drone_flag, i, j);
            boolean f = false;

            if (flag[1][1] == -2)
                break;
            if (flag[1][1] == -1) {
                if (drone_point[flag[0][0]][flag[0][1]] != null) {
                    drone_point[flag[0][0]][flag[0][1]].include = false;

                        drone_move.add(drone_point[flag[0][0]][flag[0][1]]);


                    dm++;
                }
                i = flag[0][0];
                j = flag[0][1];
                Log.e(TAG, "Path " + " in1 " + drone_num[flag[0][0]][flag[0][1]]);
            } else {

                if ((drone_num[flag[0][0]][flag[0][1]] > drone_num[flag[1][0]][flag[1][1]]/* && position == 1*/) /*|| (drone_num[flag[0][0]][flag[0][1]] < drone_num[flag[1][0]][flag[1][1]] && position == 2)*/) {
                    if (drone_point[flag[0][0]][flag[0][1]] != null) {
                        drone_point[flag[0][0]][flag[0][1]].include = false;


                            if (drone_point[flag[0][0]][flag[0][1]].lat > 0) {

                                drone_move.add(drone_point[flag[0][0]][flag[0][1]]);

                            }

                        dm++;
                    }
                    i = flag[0][0];
                    j = flag[0][1];
                    Log.e(TAG, "Path " + " in2  " + drone_num[flag[0][0]][flag[0][1]]);

                } else {
                    if (drone_point[flag[1][0]][flag[1][1]] != null) {

                            if (drone_point[flag[1][0]][flag[1][1]].lat > 0) {
                                drone_move.add(drone_point[flag[1][0]][flag[1][1]]);


                            }

                        dm++;
                    }
                    i = flag[1][0];
                    j = flag[1][1];
                    Log.e(TAG, "Path3  " + drone_num[flag[1][0]][flag[1][1]]);

                }
            }

            if (drone_move.size() >= 3) {

                float b1 = (float) Coordinates.bearing(drone_move.get(drone_move.size() - 2), drone_move.get(drone_move.size() - 1));
                float b2 = (float) Coordinates.bearing(drone_move.get(drone_move.size() - 3), drone_move.get(drone_move.size() - 1));
                // if (!Welcome.missionType.equals(Mission.MissionType.POLYGON)){
                b1 = Math.round(b1);
                b2 = Math.round(b2);

                /*   }
                   else{

                      // double b1t=b1;
                      // double b2t=b2;

                       b1=  DroneInformation.truncateDecimal(b1,3);
                         b2= DroneInformation.truncateDecimal(b2,3);
                       Log.e(TAG, "keep two decimal only in polygon mission: "+b1+" , "+b2 );


                   }*/
                Log.d(TAG, "drone bearing: " + (drone_move.size() - 2) + ": " + b1 + " " + (drone_move.size() - 3) + ": " + b2);


                if (Double.compare(b1, b2) != 0 || drone_move.get(drone_move.size() - 2).alwaysinclude) {
                    Log.e(TAG, "add to waypoint mission" + (dm - 1));

                        //  mission_points.add(drone_move.get(drone_move.size() - 2));
                        drone_move.get(drone_move.size() - 2).include = true;
                        totalIncluded++;
                    } //   Mark.create_marker(new LatLong(drone_move.get(drone_move.size() - 2).lat,drone_move.get(drone_move.size() - 2).lon),R.drawable.photo,c,null, null);


            }
            arrayy = arrayy.concat(Integer.toString(i) + "");
            arrayy = arrayy.concat(";");

            arrayx = arrayx.concat(Integer.toString(j) + "");
            arrayx = arrayx.concat(";");
            Log.e(TAG, "Path i j" + i + " " + j);
        }

        for (int k1 = 0; k1 < drone_move.size(); k1++) {
            Log.d(TAG, "path include: " + k1 + " , " + drone_move.get(k1).include);

        }

        Log.d(TAG, "array x: " + arrayx);
        Log.d(TAG, "array y: " + arrayy);

        drone_move.get(drone_move.size() - 1).include = true;
        totalIncluded++;

        // mission_points.add(drone_move.get(drone_move.size() - 1));


        return drone_move;
    }



    /**
     * Create a 2d arrat base on corner points (polygon and grid mission only)
     *
     * @param lat         Latitude array
     * @param lon         Longitude array
     * @param drone_point Coordinates points
     * @param drone_num   Set weight to each coordinate to create a grid or polygon
     * @param x           Number of points on x axis
     * @param y           Number of points on y axis
     * @param c3
     * @param c4
     * @return
     */
    private  boolean create_array(double lat[], double lon[], Coordinates[][] drone_point,
                                        int[][] drone_num, int x, int y, Coordinates c3, Coordinates c4) {


        Coordinates[] points = null;
        if (y > 0)
            points = new Coordinates[(2 * y)];

        Coordinates[][] midpoints = new Coordinates[2][x];
        Coordinates[] inpoints;
        //Coordinates[][] drone_point
        // int[][] drone_num;


        Coordinates t1D[] = new Coordinates[1];
        Coordinates t2D[][] = new Coordinates[1][1];
        int sy = get_gps_distance(lon[0], lat[0], Coordinates.bearing(new Coordinates(lon[0], lat[0]), c4), y, r, 1, 2, points, t2D, 0);
        get_gps_distance(lon[0], lat[1], Coordinates.bearing(c4, new Coordinates(lon[1], lat[1])), x, r, 1, 1, t1D, midpoints, 0);//create points for mid points

        get_gps_distance(lon[1], lat[0], Coordinates.bearing(c3, new Coordinates(lon[1], lat[1])), y, r, 0, 2, points, t2D, sy);
        get_gps_distance(lon[0], lat[0], Coordinates.bearing(new Coordinates(lon[0], lat[0]), c3), x, r, 0, 1, t1D, midpoints, 0);//create points for mid points
        Log.e(TAG, "midpoints d: " + midpoints[0].length);


/*
if (Double.compare(midpoints[0][midpoints[0].length-1].lat,lat[1])!=Double.compare(lat[0],lat[1])){

  for (int i=0;i<mi)

}
*/


        inpoints = new Coordinates[(x * y)];


        int in = 0;
        if (midpoints != null && midpoints.length > 0)
            for (int i = 0; i < x; i++)
                in = get_gps_distance(midpoints[0][i].lat, midpoints[0][i].lon, Coordinates.bearing(midpoints[0][i], midpoints[1][i]), y, r, 0, 2, inpoints, t2D, in);


        int n = 1;
        int p = 0, m1 = 0, m2 = 0, mj = drone_point[0].length - 1, k = 0, p1 = 1;
        int c1 = 2;
        int kk = 1000;
        for (int i = 0; i < drone_point.length; i++) {

            for (int j = 0; j < drone_point[0].length; j++) {
                drone_num[i][j] = n++;


                if (j == 0 && i != 0 && i != drone_point.length - 1) {

                    drone_point[i][j] = midpoints[0][m1];
                    m1++;

                } else if (j == mj && i != 0 && i != drone_point.length - 1) {

                    drone_point[i][j] = midpoints[1][m2];
                    m2++;

                } else if (i == 0 && j != mj && j != 0) {

                    drone_point[i][j] = points[p];
                    p++;
                } else if (i == drone_point.length - 1 && j != mj && j != 0) {


                    drone_point[i][j] = points[p];
                    p++;
                } else if (i == 0 && j == 0)
                    drone_point[i][j] = new Coordinates(lon[0], lat[0]);

                else if (i == drone_point.length - 1 && j == 0) {

                    drone_point[i][j] = new Coordinates(lon[1], lat[0]);
                } else if (i == drone_point.length - 1 && j == mj) {

                    drone_point[i][j] = new Coordinates(lon[1], lat[1]);

                } else if (i == 0 && j == mj) {

                    drone_point[i][j] = new Coordinates(lon[0], lat[1]);

                } else {
                    drone_point[i][j] = inpoints[k];
                    k++;
                }
                Log.d("Coordinates", "drone show coordinates: " + drone_point[i][j].toString());
            }
            n = (c1) * (y + 2);
            c1++;

        }

        return true;
    }
    /**
     * Create some coordinates according to the angle and the initial coordinate and saves them to a table
     *
     * @param lat1    This the latitude
     * @param long1   This the latitude
     * @param angle   This the field of view of the camera
     * @param length  This is the number o coordinates which are going to create
     * @param d       This is the distance bewwen the coordinates
     * @param z       This the row of table2
     * @param type    If the type is 1 is going to use table1 otherwise table2
     * @param table1D Table to save the coordinates
     * @param table2D Table to save the coordinates
     * @return int
     */
    public  int get_gps_distance(double lat1, double long1, double angle, int length, double d, int z, int type, Coordinates table1D[], Coordinates table2D[][], int s) {
        // Earth Radious in KM
        double R = 6371;


        // 6 decimal for Leaflet and other system compatibility
        //  double lat2 = Math.round (latitude2,6);
        //double long2 = Math.round (longitude2,6);


        double brng = Math.toRadians(angle);


        Coordinates c;
        double r = d / 1000.0;
        double sum = r;
        for (int i = 0; i < length; i++) {
            //sum = sum+r;

            // Degree to Radian
            double latitude1 = Math.toRadians(lat1);
            double longitude1 = Math.toRadians(long1);

            double latitude2 = Math.asin(Math.sin(latitude1) * Math.cos(sum / R * 1.0) + Math.cos(latitude1) * Math.sin(sum / R * 1.0) * Math.cos(brng));
            double longitude2 = longitude1 + Math.atan2(Math.sin(brng) * Math.sin(sum / R * 1.0) * Math.cos(latitude1), Math.cos(sum / R * 1.0) - Math.sin(latitude1) * Math.sin(latitude2));

            double lon2 = ((longitude2 + 3 * Math.PI) % (2 * Math.PI)) - Math.PI;

            // back to degrees
            latitude2 = Math.toDegrees(latitude2);
            longitude2 = Math.toDegrees(lon2);


            int k = 0;


            c = new Coordinates(latitude2, longitude2);


            if (type == 1) {

                if (i < table2D[0].length) {
                    table2D[z][i] = c;
                    s++;
                    Log.e(TAG, "midPoint " + i + " " + table2D[z][i].toString());
                } else {

                }

            } else {
                if (s >= table1D.length)
                    continue;
                table1D[s] = c;
                Log.e(TAG, "Point " + s + " " + table1D[s].toString());
                s++;
            }


            lat1 = latitude2;
            long1 = longitude2;


        }

        return s;
    }
    /**
     * This function calculates the angle (bearing) between two coordinates
     *
     * @param p1 The first coordinate
     * @param p2 The second coordinate
     * @return double Returns the angle between two coordinates
     */

    public static double bearing(Coordinates p1, Coordinates p2) {

        double degToRad = Math.PI / 180.0;

        double phi1 = p1.lat * degToRad;

        double phi2 = p2.lat * degToRad;

        double lam1 = p1.lon * degToRad;

        double lam2 = p2.lon * degToRad;


        return Math.atan2(Math.sin(lam2 - lam1) * Math.cos(phi2),

                Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(lam2 - lam1)

        ) * 180 / Math.PI;

    }



    /**
     * This function calculates the distance in meters between two coordinates
     *
     * @param point1 The first coordinate
     * @param point2 The second coordinate
     * @return double Returns the distance between the two coordinates in km
     */
    public  double distFrom(Coordinates point1, Coordinates point2) {
        double earthRadius = 6371.0; // miles (or 6371.0 kilometers)
        double dLat = Math.toRadians(point2.lat - point1.lat);
        double dLng = Math.toRadians(point2.lon - point1.lon);
        double sindLat = Math.sin(dLat / 2);
        double sindLng = Math.sin(dLng / 2);
        double a = Math.pow(sindLat, 2) + Math.pow(sindLng, 2)
                * Math.cos(Math.toRadians(point1.lat)) * Math.cos(Math.toRadians(point2.lat));
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        double dist = earthRadius * c;

        return dist;
    }

    /**
     * This function check if the whole array is set false.This function uses to polygon and grid algorithm
     *
     * @param d A boolean array
     * @return boolean Returns false if all places in array are false otherwis returns true
     */
    public static boolean check_array(boolean d[][]) {
        for (int i = 0; i < d.length; i++) {

            for (int j = 0; j < d[0].length; j++) {
                if (d[i][j])
                    return true;

            }
        }
        return false;

    }

    /**
     * This function uses in grid and polygon algorithm.It checks if the neighbours
     * of i,j coordinates in in d array are true  and saves them to an array.
     *
     * @param d A boolean array
     * @param i The row of the table d
     * @param j The column of the table d
     * @return int[][] Returns an array with the coordinates which are set to true
     */
    public static int[][] neighboor(boolean d[][], int i, int j) {
        int flag[][] = new int[2][2];
        int k = 0;


        if (i - 1 >= 0 && d[i - 1][j] == true && k < flag.length) {
            flag[k][0] = i - 1;
            flag[k][1] = j;
            k++;
        }

        if (i + 1 < d.length && d[i + 1][j] == true && k < flag.length) {
            flag[k][0] = i + 1;
            flag[k][1] = j;
            k++;
        }
        if (j + 1 < d[0].length && d[i][j + 1] == true && k < flag.length) {
            flag[k][0] = i;
            flag[k][1] = j + 1;
            k++;
        }
        if (j - 1 >= 0 && d[i][j - 1] == true && k < flag.length) {
            flag[k][0] = i;
            flag[k][1] = j - 1;
            k++;
        }

        if (k == 1)
            flag[1][1] = -1;
        if (flag[0][0] == 0 && flag[0][1] == 0)
            flag[1][1] = -2;
        return flag;

    }



}
