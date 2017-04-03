#include <dwl_rviz_plugin/DisplayInterface.h>


namespace dwl_rviz_plugin
{


DisplayInterface::DisplayInterface()
{
	markers_pub_.reset(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> (node_, "marker_array", 1));
}


DisplayInterface::~DisplayInterface()
{

}


void DisplayInterface::publishMarkerArray(const ros::Time& time)
{
	visualization_msgs::MarkerArray markers;
	for (int i = 0; i < display_stack_.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.stamp = time;
		marker.header.frame_id = "base_link";
		marker.ns = "dls";
		marker.id = i;
		if (display_stack_[i].type == displayType::LINE) {
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.color.r = display_stack_[i].color.r;
			marker.color.g = display_stack_[i].color.g;
			marker.color.b = display_stack_[i].color.b;
			marker.color.a = display_stack_[i].color.a;
			marker.scale.x = display_stack_[i].scale(0);
			geometry_msgs::Point p;
			p.x = display_stack_[i].p1(0);
			p.y = display_stack_[i].p1(1);
			p.z = display_stack_[i].p1(2);
			marker.points.push_back(p);

			p.x = display_stack_[i].p2(0);
			p.y = display_stack_[i].p2(1);
			p.z = display_stack_[i].p2(2);
			marker.points.push_back(p);
			markers.markers.push_back(marker);
		} else if (display_stack_[i].type == displayType::ARROW){
			marker.type = visualization_msgs::Marker::ARROW;
			marker.color.r = display_stack_[i].color.r;
			marker.color.g = display_stack_[i].color.g;
			marker.color.b = display_stack_[i].color.b;
			marker.color.a = display_stack_[i].color.a;
			marker.scale.x = display_stack_[i].scale(0);
			marker.scale.y = display_stack_[i].scale(1);
			marker.scale.z = display_stack_[i].scale(2);
			geometry_msgs::Point p;
			p.x = display_stack_[i].p1(0);
			p.y = display_stack_[i].p1(1);
			p.z = display_stack_[i].p1(2);
			marker.points.push_back(p);

			p.x = display_stack_[i].p2(0);
			p.y = display_stack_[i].p2(1);
			p.z = display_stack_[i].p2(2);
			marker.points.push_back(p);
			markers.markers.push_back(marker);
		} else if (display_stack_[i].type == displayType::POINT) {
			marker.type = visualization_msgs::Marker::POINTS;
			marker.color.r = display_stack_[i].color.r;
			marker.color.g = display_stack_[i].color.g;
			marker.color.b = display_stack_[i].color.b;
			marker.color.a = display_stack_[i].color.a;
			marker.scale.x = display_stack_[i].scale(0);
			marker.scale.y = display_stack_[i].scale(1);
			marker.scale.z = display_stack_[i].scale(2);
			marker.pose.orientation.w = 1.0;
			marker.pose.position.x = display_stack_[i].p1(0);
			marker.pose.position.y = display_stack_[i].p1(1);
			marker.pose.position.z = display_stack_[i].p1(2);
		} else if (display_stack_[i].type == displayType::SPHERE) {
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.color.r = display_stack_[i].color.r;
			marker.color.g = display_stack_[i].color.g;
			marker.color.b = display_stack_[i].color.b;
			marker.color.a = display_stack_[i].color.a;
			marker.scale.x = display_stack_[i].scale(0);
			marker.scale.y = display_stack_[i].scale(1);
			marker.scale.z = display_stack_[i].scale(2);
			marker.pose.orientation.w = 1.0;
			marker.pose.position.x = display_stack_[i].p1(0);
			marker.pose.position.y = display_stack_[i].p1(1);
			marker.pose.position.z = display_stack_[i].p1(2);
			markers.markers.push_back(marker);
		}
	}
	
	display_stack_.clear();
	if (markers_pub_->trylock()) {
		markers_pub_->msg_ = markers;
		markers_pub_->unlockAndPublish();
	}
}


void DisplayInterface::drawSphere(const Eigen::Vector3d& position,
								  double radiu,
								  const Color& color)
{
	displayData data;
	data.p1 = position;
	data.scale = Eigen::Vector3d(radiu, radiu, radiu);
	data.color = color;
	data.type = displayType::SPHERE;
	display_stack_.push_back(data);
}


void DisplayInterface::drawArrow(const Eigen::Vector3d& begin,
								 const Eigen::Vector3d& end,
								 double shaft_diameter,
								 double head_diameter,
								 double head_length,
								 const Color& color)
{
	displayData data;
	data.p1 = begin;
	data.p2 = end;
	data.scale = Eigen::Vector3d(shaft_diameter, head_diameter, head_length);
	data.color = color;
	data.type = displayType::ARROW;
	display_stack_.push_back(data);
}


void DisplayInterface::drawCone(const Eigen::Vector3d& vertex,
								double height,
								double radiu,
								const Color& color)
{
	displayData data;
	data.p1 = vertex;
	data.p2 = vertex + Eigen::Vector3d(0., 0., height); // TODO orientation
	data.scale = Eigen::Vector3d(0., 2 * radiu, height);
	data.color = color;
	data.type = displayType::ARROW;
	display_stack_.push_back(data);
}


void DisplayInterface::drawCone(const Eigen::Vector3d& vertex,
								double height,
								double angle_rad,
								const Color& color)
{
	double radiu = height * atan(angle_rad);
	drawCone(vertex, height, radiu, color);
}

} //@namespace dwl_rviz_plugin
