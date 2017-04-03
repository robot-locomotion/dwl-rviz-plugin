#ifndef DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H
#define DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H

// Realtime tools include
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <visualization_msgs/MarkerArray.h>


namespace dwl_rviz_plugin
{

class DisplayInterface
{
	enum class userGraphicsType {
		LINE = 0,
		POINT = 1,
		ARROW = 2,
		SPHERE = 3
	};

	public:
		DisplayInterface();
		~DisplayInterface();

		publishMarkerArray(const ros::Time& time);

		drawLine();


	private:
		boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> > markers_pub_;

}
} //@namespace dwl_rviz_plugin

#endif