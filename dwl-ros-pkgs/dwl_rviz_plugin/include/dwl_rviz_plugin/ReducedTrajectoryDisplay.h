#ifndef DWL_RVIZ_PLUGIN__REDUCED_TRAJECTORY_DISPLAY__H
#define DWL_RVIZ_PLUGIN__REDUCED_TRAJECTORY_DISPLAY__H

#include <rviz/message_filter_display.h>
#include <dwl_rviz_plugin/PointVisual.h>
#include <dwl_rviz_plugin/ArrowVisual.h>
#include <dwl_rviz_plugin/PolygonVisual.h>
#include <dwl_msgs/ReducedTrajectory.h>


namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;

} //@namespace rviz


namespace dwl_rviz_plugin
{

/**
 * @class ReducedTrajectoryDisplay
 * @brief Displays a dwl_msgs::ReducedTrajectory message
 */
class ReducedTrajectoryDisplay :
		public rviz::MessageFilterDisplay<dwl_msgs::ReducedTrajectory>
{
	Q_OBJECT
	public:
		/** @brief Constructor function */
		ReducedTrajectoryDisplay();

		/** @brief Destructor function */
		~ReducedTrajectoryDisplay();

		/** @brief Clear the visuals by deleting their objects */
		void reset();

		/** @brief Overridden from Display. */
		void onInitialize();

		/**
		 * @brief Function to handle an incoming ROS message
		 * This is our callback to handle an incoming message
		 * @param const dwl_msgs::ReducedTrajectory::ConstPtr& Reduced trajectory msg
		 */
		void processMessage(const dwl_msgs::ReducedTrajectory::ConstPtr& msg);


	private Q_SLOTS:
		/** @brief Helper function to apply color and alpha to all visuals.
		/* Set the current color and alpha values for each visual */
		void updateCoMRadiusAndAlpha();
		void updateCoPRadiusAndAlpha();
		void updatePendulumArrowGeometry();



	private:
		/** Destroy all the objects for visualization */
		void destroyObjects();

		/**
		 * @brief Generate a set of colors given a number of points
		 * @param std::vector<Ogre::ColourValue>& Set of colors
		 * @param unsigned int Number of points
		 */
		void generateSetOfColors(std::vector<Ogre::ColourValue>& colors,
								 unsigned int num_points);

		/** @brief Properties to show on side panel */
		rviz::Property* com_category_;
		rviz::Property* cop_category_;
		rviz::Property* support_category_;
		rviz::Property* pendulum_category_;


		/** @brief Object for visualization of the data */
		std::vector<boost::shared_ptr<PointVisual> > com_visual_;
		std::vector<boost::shared_ptr<PointVisual> > cop_visual_;
		std::vector<boost::shared_ptr<PolygonVisual> > support_visual_;
		std::vector<boost::shared_ptr<ArrowVisual> > pendulum_visual_;


		/** @brief Property objects for user-editable properties */
		//		rviz::EnumProperty* com_style_property_;
		//		rviz::ColorProperty* com_color_property_;
		rviz::FloatProperty* com_alpha_property_;
		rviz::FloatProperty* com_radius_property_;

		//		rviz::ColorProperty* cop_color_property_;
		rviz::FloatProperty* cop_alpha_property_;
		rviz::FloatProperty* cop_radius_property_;

		rviz::ColorProperty* support_color_property_;
		rviz::FloatProperty* support_alpha_property_;

		rviz::FloatProperty* pendulum_head_radius_property_;
		rviz::FloatProperty* pendulum_head_length_property_;
		rviz::FloatProperty* pendulum_shaft_radius_property_;
		rviz::FloatProperty* pendulum_shaft_length_property_;

		float com_radius_;
		float com_alpha_;
		float cop_radius_;
		float cop_alpha_;
};

} //@namespace dwl_rviz_plugin

#endif
