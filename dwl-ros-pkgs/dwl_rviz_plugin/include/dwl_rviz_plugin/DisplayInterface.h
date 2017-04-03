#ifndef DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H
#define DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H

#include <ros/NodeHandle.h>

// Realtime tools include
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <visualization_msgs/MarkerArray.h>


namespace dwl_rviz_plugin
{

enum ColorType {
	RED,
	GREEN,
	BLUE,
	YELLOW,
	WHITE,
	BLACK
};

struct Color
{
	Color(double _r, double _g, double _b, double _a) :
		r(_r), g(_g), b(_b), a(_a) { }
	Color(ColorType color, double _a) {
		r = 0.;
		g = 0.;
		b = 0.;
		a = _a;
 		if (color == ColorType::RED) {
			r = 1.;
		} else if (color == ColorType::GREEN) {
			g = 1.;
		} else if (color == ColorType::BLUE) {
			b = 1.;
		} else if (color == ColorType::YELLOW) {
			r = 1.;
			g = 1.;
		} else if (color == ColorType::WHITE) {
			r = 1.;
			g = 1.;
			b = 1.;
		}
	}

	double r;
	double g;
	double b;
	double a;
}

enum class displayType {
	LINE = 0,
	POINT = 1,
	ARROW = 2,
	SPHERE = 3
};

struct displayData {
	Eigen::Vector3d p1;
	Eigen::Vector3d p2;
	Eigen::Vector3d scale;
	Color color;
	userGraphicsType type;
};

class DisplayInterface
{
	public:
		DisplayInterface();
		~DisplayInterface();

		publishMarkerArray(const ros::Time& time);

		void drawSphere(const Eigen::Vector3d& position,
						double radiu,
						const Color& color);
		void drawArrow(const Eigen::Vector3d& begin,
					   const Eigen::Vector3d& end,
					   double shaft_diameter,
					   double head_diameter,
					   double head_length,
					   const Color& color);
		void drawCone(const Eigen::Vector3d& vertex,
					  double height,
					  double radiu,
					  const Color& color);
		void drawCone(const Eigen::Vector3d& vertex,
					  double height,
					  double angle,
					  const Color& color);


	private:
		ros::NodeHandle node_;
		boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> > markers_pub_;

		/** @brief user stack for arbitrary graphics drawing */
		std::vector<displayData> display_stack_;
}

} //@namespace dwl_rviz_plugin

#endif
