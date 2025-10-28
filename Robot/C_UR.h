#ifndef __C_UR_H__
#define  __C_UR_H__

#include <visp/vpClient.h>
#include <visp/vpColVector.h>
#include <visp/vpPoseVector.h>

#include <string>

#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;

class C_UR
{
public:
	C_UR(std::string hostName, unsigned int portNum, double _delta_t = 0.004);
    ~C_UR();

	int setCameraVelocity(vpColVector &v);
  int setCameraArticularPose(vpColVector &j);
  int setCameraRelativePose(vpColVector &p);
  int getCameraPoseRaw(vpColVector &p);
  int getCameraPose(vpPoseVector &p);
  int getCameraArticularPose(vpColVector &j);
    
private:
	vpClient clientUR;
  RTDEReceiveInterface rtde_control;
  RTDEReceiveInterface rtde_receive;
	vpColVector v;
  double delta_t;
};

#endif // __C_UR_H__
