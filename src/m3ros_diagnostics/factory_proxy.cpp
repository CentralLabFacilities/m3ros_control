
#include <stdio.h>
#include <m3rt/base/component.h>
#include "m3ros_diagnostics/m3ros_monitor.h"


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define M3_ROS_MONITOR_COMPONENT_NAME "m3ros_monitor"
///////////////////////////////////////////////////////

m3rt::M3Component* create_m3ros_monitor(){return new m3ros_diagnostics::M3RosMonitor;}
void destroy_m3ros_monitor(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3_ROS_MONITOR_COMPONENT_NAME] = create_m3ros_monitor;
		m3rt::destroyer_factory[M3_ROS_MONITOR_COMPONENT_NAME] =  destroy_m3ros_monitor;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
