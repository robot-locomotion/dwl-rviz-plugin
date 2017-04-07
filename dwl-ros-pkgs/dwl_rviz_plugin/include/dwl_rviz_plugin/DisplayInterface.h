#ifndef DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H
#define DWL_RVIZ_PLUGIN__DISPLAY_INTERFACE__H

#include <ros/ros.h>

#include <dwl/utils/Orientation.h>

// Realtime tools include
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <visualization_msgs/MarkerArray.h>


namespace dwl {

enum ColorType { Red, Green, Blue, Yellow, White, Black };

struct Color
{
	Color() : r(0.), g(0.), b(0.), a(1.) { }
	Color(double _r, double _g, double _b, double _a) :
		r(_r), g(_g), b(_b), a(_a) { }
	Color(ColorType color, double _a) {
		r = 0.;
		g = 0.;
		b = 0.;
		a = _a;
 		if (color == ColorType::Red) {
			r = 1.;
		} else if (color == ColorType::Green) {
			g = 1.;
		} else if (color == ColorType::Blue) {
			b = 1.;
		} else if (color == ColorType::Yellow) {
			r = 1.;
			g = 1.;
		} else if (color == ColorType::White) {
			r = 1.;
			g = 1.;
			b = 1.;
		}
	}

	double r;
	double g;
	double b;
	double a;
};

enum class DisplayType {
	LINE = 0,
	POINT = 1,
	ARROW = 2,
	SPHERE = 3,
	TEXT = 11
};

struct DisplayData {
	DisplayData() : p1(Eigen::Vector3d::Zero()), p2(Eigen::Vector3d::Zero()),
			scale(Eigen::Vector3d::Zero()), color(Color()),
			type(DisplayType::LINE) { }
	DisplayData(const Eigen::Vector3d& _p1,
				const Eigen::Vector3d& _p2,
				const Eigen::Vector3d& _scale,
				const Color& _color,
				DisplayType _type) : p1(_p1), p2(_p2), scale(_scale),
						color(_color), type(_type) { }

	Eigen::Vector3d p1;
	Eigen::Vector3d p2;
	Eigen::Vector3d scale;
	Color color;
	DisplayType type;
	std::string text;
	std::string frame;
};

} //@namespace dwl


namespace dwl_rviz_plugin
{

class DisplayInterface
{
	public:
		/** @brief Constructor function */
		DisplayInterface();

		/** @brief Destructor function */
		~DisplayInterface();

		/**
		 * @brief Publishes the set of added markers
		 * @param const ros::Time& Actual time
		 */
		void publishMarkerArray(const ros::Time& time);

		/**
		 * @brief Draws a line in defined position w.r.t. to certain frame
		 * @param const Eigen::Vector3d& Point 1 of the line
		 * @param const Eigen::Vector3d& Point 2 of the line
		 * @param double Width of the line
		 * @param const dwl::Color& Color of the line
		 * @param std::string Frame where is described the line
		 */
		void drawLine(const Eigen::Vector3d& point1,
					  const Eigen::Vector3d& point2,
					  double width,
					  const dwl::Color& color,
					  std::string frame);

		/**
		 * @brief Draws a sphere in defined position w.r.t. to certain frame
		 * @param const Eigen::Vector3d& Position of the sphere
		 * @param double Radius of the sphere
		 * @param const dwl::Color& Color of the sphere
		 * @param std::string Frame where is described the sphere
		 */
		void drawSphere(const Eigen::Vector3d& position,
						double radius,
						const dwl::Color& color,
						std::string frame);

		/**
		 * @brief Draws an arrow in a certain frame
		 * @param const Eigen::Vector3d& Start point of the arrow
		 * @param const Eigen::Vector3d& End point of the arrow
		 * @param double Diameter of the shaft
		 * @param double Diameter of the head
		 * @param double Length of the head
		 * @param const dwl::Color& Color of the arrow
		 * @param std::string Frame where is described the arrow
		 */
		void drawArrow(const Eigen::Vector3d& begin,
					   const Eigen::Vector3d& end,
					   double shaft_diameter,
					   double head_diameter,
					   double head_length,
					   const dwl::Color& color,
					   std::string frame);

		/**
		 * @brief Draws an arrow in a certain frame
		 * @param const Eigen::Vector3d& Start point of the arrow
		 * @param const Eigen::Vector3d& Direction of the arrow
		 * @param double Diameter of the shaft
		 * @param double Diameter of the head
		 * @param double Length of the head
		 * @param const dwl::Color& Color of the arrow
		 * @param std::string Frame where is described the arrow
		 */
		void drawArrow(const Eigen::Vector3d& origin,
					   const Eigen::Vector3d& direction,
					   double arrow_length,
					   double shaft_diameter,
					   double head_diameter,
					   double head_length,
					   const dwl::Color& color,
					   std::string frame);

		/**
		 * @brief Draws an arrow in a certain frame
		 * @param const Eigen::Vector3d& Origin of the arrow
		 * @param const Eigen::Quaterniond& Orientation of the arrow
		 * @param double Length of the arrow
		 * @param double Diameter of the shaft
		 * @param double Diameter of the head
		 * @param double Length of the head
		 * @param const dwl::Color& Color of the arrow
		 * @param std::string Frame where is described the arrow
		 */
		void drawArrow(const Eigen::Vector3d& origin,
					   const Eigen::Quaterniond& orientation,
					   double arrow_length,
					   double shaft_diameter,
					   double head_diameter,
					   double head_length,
					   const dwl::Color& color,
					   std::string frame);

		/**
		 * @brief Draws a cone in a certain frame
		 * @param const Eigen::Vector3d& Vertex of the cone
		 * @param const Eigen::Quaterniond& Orientation of the cone
		 * @param double Height of the cone
		 * @param double Radius of the cone
		 * @param const dwl::Color& Color of the cone
		 * @param std::string Frame where is described the cone
		 */
		void drawCone(const Eigen::Vector3d& vertex,
					  const Eigen::Quaterniond& orientation,
					  double height,
					  double radius,
					  const dwl::Color& color,
					  std::string frame);

		/**
		 * @brief Draws a cone in a certain frame
		 * @param const Eigen::Vector3d& Vertex of the cone
		 * @param const Eigen::Vector3d& RPY orientation of the cone
		 * @param double Height of the cone
		 * @param double Radius of the cone
		 * @param const dwl::Color& Color of the cone
		 * @param std::string Frame where is described the cone
		 */
		void drawCone(const Eigen::Vector3d& vertex,
				  	  const Eigen::Vector3d& rpy,
					  double height,
					  double radius,
					  const dwl::Color& color,
					  std::string frame);

		/**
		 * @brief Draws a text in a certain frame
		 * @param const std::string The text to be printed
		 * @param const Eigen::Vector3d& Position of the text
		 * @param double Height of the upper-case
		 * @param const dwl::Color& Color of the cone
		 * @param std::string Frame where is described the cone
		 */
		void drawText(std::string text,
					  const Eigen::Vector3d& position,
					  double uppercase_height,
					  const dwl::Color& color,
					  std::string frame);


	private:
		/** @brief ROS node handle */
		ros::NodeHandle node_;

		/** @brief Marker publisher */
		boost::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> > markers_pub_;

		/** @brief user stack for arbitrary graphics drawing */
		std::vector<dwl::DisplayData> display_stack_;
};

} //@namespace dwl_rviz_plugin

#endif
