#include <dwl_rviz_plugin/DisplayInterface.h>


namespace dwl_rviz_plugin
{


DisplayInterface::DisplayInterface()
{
	markers_pub_.reset(
			new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> (node_, "vis", 1));
	std::cout << "construnction" << std::endl;
}


DisplayInterface::~DisplayInterface()
{

}


void DisplayInterface::publishMarkerArray(const ros::Time& time)
{
	visualization_msgs::MarkerArray markers;
	for (unsigned int i = 0; i < display_stack_.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.stamp = time;
		marker.header.frame_id = display_stack_[i].frame;
		marker.ns = "dls";
		marker.id = i;
		if (display_stack_[i].type == dwl::DisplayType::LINE) {
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
		} else if (display_stack_[i].type == dwl::DisplayType::ARROW){
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
		} else if (display_stack_[i].type == dwl::DisplayType::POINT) {
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
		} else if (display_stack_[i].type == dwl::DisplayType::SPHERE) {
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
	
	if (markers_pub_->trylock()) {
		display_stack_.clear();
		markers_pub_->msg_ = markers;
		markers_pub_->unlockAndPublish();
	}
}


void DisplayInterface::drawLine(const Eigen::Vector3d& point1,
			  	  	  	  	    const Eigen::Vector3d& point2,
								double width,
								const dwl::Color& color,
								std::string frame)
{
	dwl::DisplayData data;
	data.p1 = point1;
	data.p2 = point2;
	data.scale = Eigen::Vector3d(width, 0., 0.);
	data.color = color;
	data.type = dwl::DisplayType::LINE;
	data.frame = frame;
	display_stack_.push_back(data);
}


void DisplayInterface::drawSphere(const Eigen::Vector3d& position,
								  double radius,
								  const dwl::Color& color,
								  std::string frame)
{
	dwl::DisplayData data;
	data.p1 = position;
	data.scale = Eigen::Vector3d(radius, radius, radius);
	data.color = color;
	data.type = dwl::DisplayType::SPHERE;
	data.frame = frame;
	display_stack_.push_back(data);
}


void DisplayInterface::drawArrow(const Eigen::Vector3d& begin,
								 const Eigen::Vector3d& end,
								 double shaft_diameter,
								 double head_diameter,
								 double head_length,
								 const dwl::Color& color,
								 std::string frame)
{
	dwl::DisplayData data;
	data.p1 = begin;
	data.p2 = end;
	data.scale = Eigen::Vector3d(shaft_diameter, head_diameter, head_length);
	data.color = color;
	data.type = dwl::DisplayType::ARROW;
	data.frame = frame;
	display_stack_.push_back(data);
}


void DisplayInterface::drawArrow(const Eigen::Vector3d& origin,
								 const Eigen::Quaterniond& orientation,
								 double arrow_length,
								 double shaft_diameter,
								 double head_diameter,
								 double head_length,
								 const dwl::Color& color,
								 std::string frame)
{
	Eigen::Vector3d end = origin +
			dwl::math::getDirectionCosineMatrix(orientation).transpose() *
			Eigen::Vector3d(0., 0., arrow_length);
	drawArrow(origin, end, shaft_diameter, head_diameter, head_length, color, frame);
}


void DisplayInterface::drawCone(const Eigen::Vector3d& vertex,
								const Eigen::Quaterniond& orientation,
								double height,
								double radius,
								const dwl::Color& color,
								std::string frame)
{
	dwl::DisplayData data;
	Eigen::Vector3d end = dwl::math::getDirectionCosineMatrix(orientation).transpose() *
			Eigen::Vector3d(0., 0., height);
	data.p1 = vertex + end;
	data.p2 = vertex;
	data.scale = Eigen::Vector3d(0., 2 * radius, height);
	data.color = color;
	data.type = dwl::DisplayType::ARROW;
	data.frame = frame;
	display_stack_.push_back(data);
}


void DisplayInterface::drawCone(const Eigen::Vector3d& vertex,
		  	  	  	  	  	    const Eigen::Vector3d& rpy,
								double height,
								double radius,
								const dwl::Color& color,
								std::string frame)
{
	drawCone(vertex, dwl::math::getQuaternion(rpy), height, radius, color, frame);
}
} //@namespace dwl_rviz_plugin
