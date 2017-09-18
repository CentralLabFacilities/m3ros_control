
#include <stdio.h>
#include <m3rt/base/component.h>
#include "m3ros_control/m3ros_control.h"


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define M3_ROS_CONTROL_COMPONENT_NAME "m3ros_control"
///////////////////////////////////////////////////////

m3rt::M3Component* create_m3ros_control(){return new m3ros_control::M3RosControl;}
void destroy_m3ros_control(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3_ROS_CONTROL_COMPONENT_NAME] = create_m3ros_control;
		m3rt::destroyer_factory[M3_ROS_CONTROL_COMPONENT_NAME] =  destroy_m3ros_control;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
