#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFi.h"  // in caso :: al posto di /
#include <cmath>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Qaternion.h>
#include <tf2/LinearMath/Quaternion.h>

class gps_to_odom{
    
    private:
        ros::NodeHandle n; 
        ros::Subscriber sub;
        bool flag;

        //reference system data
        float64 ref_latitude ;
        float64 ref_longitude ;
        float64 ref_altitude ;
        vector<float64> ref_ecef(3, 0); //ecef format of reference system
        vector<vector<float64>> ref_matrix(3, vector<float64>(3)); //matrix to obtain enu

        vector<float64> prec_pose(3, 0); //precedent position of the robot, necessary to calculate orientation

    public:
        gps_to_odom(){ 

            flag=false;
            sub = n.subscribe("/fix", 1, &gps_to_odom::callback, this);

            ros::Rate loop_rate(5);

            while (ros::ok()){
                        
                    ros::spinOnce();
                        
                    loop_rate.sleep();

            }
        }


        void callback(const sensor_msgs::NavSatFi::::ConstPtr& msg){


            nav_msgs:: Odometry odom;

            float64 latitude = msg.latitude;
            float64 longitude = msg.longitude;
            float64 altitude = msg.altitude;

            vector<float64> ecef; 
            vector<float64> enu; //(x, y, z, orientation)

            if(!flag){
                this->set_reference_system(latitude, longitude, altitude);
                prec_pose=this->from_ECEF_to_ENU(this->from_gps_to_ECEF(latitude, longitude, altitude));
                flag=true;
            }

            ecef = this->from_gps_to_ECEF(msg.latitude, msg.longitude, msg.altitude);
            enu = this->from_ECEF_to_ENU(ecef);


            //creating the odometry
            odom.header.stamp= ros:: Time :: now();
            odom.header.frame_id="gps_odom";
            
            odom.pose.pose.position.x=enu[0]
            odom.pose.pose.position.y=enu[1]
            odom.pose.pose.position.z=enu[2]

            odom.pose.pose.orientation= tf2:: toMsg(tf2:: Quaternion(0,0,0,enu[3]))

        }

        vector<float64> from_gps_to_ECEF(float64 latitude, float64 longitude, float64 altitude ) {

            float64 x;
            float64 y;
            float64 z;
            vector<float64> ecef;

            int a = 6378137;
            int b = 6356752;

            float e_square = 1 - ((b*b)/(a*a));

            float n = this->n_calculation(latitude, e_square);

            ecef.push_back( (n + altitude) * cos(latitude) * cos(longitude) ); //x
            ecef.push_back( (n + altitude) * cos(latitude) * sin(longitude) ); //y
            ecef.push_back( (n * (1 - e_square) + altitude) * sin(latitude) ); //z

            return ecef;

        }

        vector<float64> from_ECEF_to_ENU(vector<float64>& ecef ){

            vector<float64> enu;
            vector<float64> pose_diff;

            pose_diff.push_back(ecef[0] - ref_latitude);
            pose_diff.push_back(ecef[1] - ref_longitude);
            pose_diff.push_back(ecef[2] - ref_altitude);

            enu = this->vectorial_mult(this->ref_matrix, pose_diff);

            edu.push_back(get_orientation(enu, prec_pose));
            prec_pose=enu;

            return enu;
        }

        void set_reference_system(float64 latitude, float64 longitude, float64 altitude){
            
            ref_latitude = latitude; //theta
            ref_longitude = longitude; //lambda
            ref_altitude = altitude; //h

            this->ref_ecef=this->from_gps_to_ECEF(ref_latitude, ref_longitude, ref_altitude);

            //da ricontrollare assolutamenteeee!!!!!!!!!!!!!!!!!!!! gradi o radianti ??
            ref_matrix[0][0]=-sin(ref_longitude);
            ref_matrix[0][1]=cos(ref_longitude);
            ref_matrix[0][2]=0;

            ref_matrix[1][0]=-sin(ref_latitude)*cos(ref_longitude);
            ref_matrix[1][1]=-sin(ref_latitude)*cos(ref_longitude);
            ref_matrix[1][2]=cos(ref_latitude);

            ref_matrix[2][0]=cos(ref_latitude)*cos(ref_longitude);
            ref_matrix[2][1]=cos(ref_latitude)*sin(ref_longitude);
            ref_matrix[2][2]=sin(ref_latitude);

        }

        float n_calculation(float64 phi, float e_square){
            
            double phi_double = static_cast<double>(phi);

            float n = a/sqrt(1 - e_square * sin(phi_double) * sin(phi_double));

            return n;
        }

        vector<float> vectorial_mult(vector< vector< float> >& matrix, vector <float>& vect){

            vector<float> res (matrix.size(), 0);

            for ( int i=0; i<matrix.size(); i++){
                for(int j=0; j<matrix.size(); j++){
                    res[i] += matrix[i][j] * vect[j];
                }
            }

            return res;
        }

        float64 get_orientation(vector<float64> curr_pose, vector<float64> prec_pose){

            float64 y_diff=curr_pose[1]-prec_pose[1];
            float64 x_diff=curr_pose[0]-prec_pose[0];

            if(x_diff==0) return 0;

            float64 tan_theta=y_diff/x_diff; //the tangent of theta is equal to (y diff)/(x diff)
            return atan(tan_theta);

        }
}






int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom");
 	gps_to_odom gps_to_odom_node;
 	ros::spin();
 	return 0;

}