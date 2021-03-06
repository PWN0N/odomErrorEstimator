#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace message_filters;

class Quat{
public:
    float x,y,z,w;


    Quat(float x,float y,float z,float w){
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    Quat(geometry_msgs::Quaternion quat){
        this->x = quat.x;
        this->y = quat.y;
        this->z = quat.z;
        this->w = quat.w;
    }
    Quat(){
    }

    Quat diff(const Quat &a)
    {
        Quat inv(this->x,this->y,this->z,this->w);
        inv.inverse();
        return inv * a;
    }

    geometry_msgs::Quaternion toRosQuaternion()
    {
        geometry_msgs::Quaternion q;
        q.x = this->x;
        q.y = this->y;
        q.z = this->z;
        q.w = this->w;
        return q;
    }

    void conjugate()
    {
        Quat q;
        q.x = -this->x;
        q.y = -this->y;
        q.z = -this->z;
        q.w = this->w;

        (*this) = q;
    }




    float dot(const Quat &q1, const Quat &q2)
    {
        return q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
    }


    const Quat operator* ( const Quat &q) const
    {
        Quat qu;
        qu.x = this->w*q.x + this->x*q.w + this->y*q.z - this->z*q.y;
        qu.y = this->w*q.y + this->y*q.w + this->z*q.x - this->x*q.z;
        qu.z = this->w*q.z + this->z*q.w + this->x*q.y - this->y*q.x;
        qu.w = this->w*q.w - this->x*q.x - this->y*q.y - this->z*q.z;
        return qu;
    }


    const Quat operator/ (float s) const
    {
        Quat q = (*this);
        return Quat(q.x / s, q.y / s, q.z / s, q.w / s);
    }
    void inverse()
    {
        Quat q = (*this);
        q.conjugate();
        (*this) = q / dot((*this), (*this));
    }

};

void Quat2EulerAngle(const Quat& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

float positionErrorSquar(geometry_msgs::Point truthPos,geometry_msgs::Point slamPos)
{
   return (truthPos.x-slamPos.x)*(truthPos.x-slamPos.x) + (truthPos.y-slamPos.y)*(truthPos.y-slamPos.y) + (truthPos.z-slamPos.z)*(truthPos.z-slamPos.z);
}

float linearAngularErrorSquar(geometry_msgs::Vector3 truthVec,geometry_msgs::Vector3 slamVec)
{
    return (truthVec.x-slamVec.x)*(truthVec.x-slamVec.x) + (truthVec.y-slamVec.y)*(truthVec.y-slamVec.y) + (truthVec.z-slamVec.z)*(truthVec.z-slamVec.z);
}

void callback(const nav_msgs::Odometry& groundTruth, const nav_msgs::Odometry& slamOdom)
{

    geometry_msgs::Point truthPosition = groundTruth.pose.pose.position;
    geometry_msgs::Quaternion truthQuat = groundTruth.pose.pose.orientation;
    geometry_msgs::Vector3 truthLinear = groundTruth.twist.twist.linear;
    geometry_msgs::Vector3 truthAngular = groundTruth.twist.twist.angular;

    geometry_msgs::Point slamPosition = slamOdom.pose.pose.position;
    geometry_msgs::Quaternion slamQuat = slamOdom.pose.pose.orientation;
    geometry_msgs::Vector3 slamLinear = slamOdom.twist.twist.linear;
    geometry_msgs::Vector3 slamAngular = slamOdom.twist.twist.angular;

    Quat truthQ(truthQuat);
    Quat slamQ(slamQuat);
    Quat diffQ = truthQ.diff(slamQ);

    double roll,pitch,yaw;
    Quat2EulerAngle(diffQ,roll,pitch,yaw);


    float posErrorSquar = positionErrorSquar(truthPosition,slamPosition);
    float linearErrorSquar  = linearAngularErrorSquar(truthLinear,slamLinear);
    float angularErrorSquar  = linearAngularErrorSquar(truthAngular,truthAngular);
    std::cout << "Timestamp: [" << groundTruth.header.stamp.nsec << std::endl;
    std::cout << "the error of rotation: [" << roll << "," << pitch << "," << yaw << "]" << std::endl;
    std::cout << "the error of position:" << posErrorSquar << std:: endl;
    std::cout << "the error of linear:" << linearErrorSquar << std:: endl;
    std::cout << "the error of angular:" << angularErrorSquar << std:: endl;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odomErrorEstimator");

  ros::NodeHandle nh;

  message_filters::Subscriber<nav_msgs::Odometry> groundTruth_sub(nh, "/gazebo/modelstates ", 1);
  message_filters::Subscriber<nav_msgs::Odometry> slamOdom_sub(nh, "odom", 1);
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(groundTruth_sub, slamOdom_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}





