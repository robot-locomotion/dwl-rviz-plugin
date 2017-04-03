#include <dwl_rviz_plugin/DisplayInterface.h>


namespace dwl_rviz_plugin
{


DisplayInterface::DisplayInterface()
{

}


DisplayInterface::~DisplayInterface()
{

}


void DisplayInterface::publishMarkerArray(const ros::Time& time)
{
	visualization_msgs::MarkerArray markers;
	for (int i = 0; i < graphics_stack_.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.stamp = time;
		marker.header.frame_id = "base_link";
		marker.ns = "dls";
		marker.id = i;
		if (graphics_stack_[i].type == userGraphicsType::LINE) {
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.color.r = graphics_stack_[i].r;
			marker.color.g = graphics_stack_[i].g;
			marker.color.b = graphics_stack_[i].b;
			marker.color.a = 1.0;
			marker.scale.x = 0.02;
			geometry_msgs::Point p;
			p.x = graphics_stack_[i].p1[0];
			p.y = graphics_stack_[i].p1[1];
			p.z = graphics_stack_[i].p1[2];
			marker.points.push_back(p);

			p.x = graphics_stack_[i].p2[0];
			p.y = graphics_stack_[i].p2[1];
			p.z = graphics_stack_[i].p2[2];
			marker.points.push_back(p);
			markers.markers.push_back(marker);
		} else if (graphics_stack_[i].type == userGraphicsType::ARROW){
			marker.type = visualization_msgs::Marker::ARROW;
			marker.color.r = graphics_stack_[i].r;
			marker.color.g = graphics_stack_[i].g;
			marker.color.b = graphics_stack_[i].b;
			marker.color.a = 1.0;
			marker.scale.x = 0.02;
			marker.scale.y = 0.05;
			geometry_msgs::Point p;
			p.x = graphics_stack_[i].p1[0];
			p.y = graphics_stack_[i].p1[1];
			p.z = graphics_stack_[i].p1[2];
			marker.points.push_back(p);

			p.x = graphics_stack_[i].p2[0];
			p.y = graphics_stack_[i].p2[1];
			p.z = graphics_stack_[i].p2[2];
			marker.points.push_back(p);
			markers.markers.push_back(marker);
		} else if (graphics_stack_[i].type == userGraphicsType::POINT) {
			marker.type = visualization_msgs::Marker::POINTS;
			marker.color.r = graphics_stack_[i].r;
			marker.color.g = graphics_stack_[i].g;
			marker.color.b = graphics_stack_[i].b;
			marker.color.a = 1.0;
			marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;
			marker.pose.orientation.w = 1.0;
			marker.pose.position.x = graphics_stack_[i].p1[0];
			marker.pose.position.y = graphics_stack_[i].p1[1];
			marker.pose.position.z = graphics_stack_[i].p1[2];
		} else if (graphics_stack_[i].type == userGraphicsType::SPHERE) {
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.color.r = graphics_stack_[i].r;
			marker.color.g = graphics_stack_[i].g;
			marker.color.b = graphics_stack_[i].b;
			marker.color.a = 1.0;
			marker.scale.x = 0.05;
			marker.scale.y = 0.05;
			marker.scale.z = 0.05;
			marker.pose.orientation.w = 1.0;
			marker.pose.position.x = graphics_stack_[i].p1[0];
			marker.pose.position.y = graphics_stack_[i].p1[1];
			marker.pose.position.z = graphics_stack_[i].p1[2];
			markers.markers.push_back(marker);
		}

	}
	
	if (markers_pub_->trylock()) {
		markers_pub_->msg_ = markers;
		markers_pub_->unlockAndPublish();
	}
}

} //@namespace dwl_rviz_plugin