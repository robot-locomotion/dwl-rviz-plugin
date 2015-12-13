#ifndef DWL_RVIZ_PLUGIN__WHOLE_BODY_STATE_DISPLAY__H
#define DWL_RVIZ_PLUGIN__WHOLE_BODY_STATE_DISPLAY__H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <rviz/message_filter_display.h>
#include <dwl_rviz_plugin/PointVisual.h>
#include <dwl_msgs/WholeBodyState.h>
#include <dwl/model/WholeBodyDynamics.h>


namespace Ogre
{
class SceneNode;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class PointStampedVisual;

} //@namespace rviz


namespace dwl_rviz_plugin
{


class WholeBodyStateDisplay: public rviz::MessageFilterDisplay<dwl_msgs::WholeBodyState>
{
	Q_OBJECT
	public:
		// Constructor.  pluginlib::ClassLoader creates instances by calling
		// the default constructor, so make sure you have one.
		WholeBodyStateDisplay();
		virtual ~WholeBodyStateDisplay();
		void clear();


	protected:
		// Overrides of public virtual functions from the Display class.
		virtual void onInitialize();

		// Clear the visuals by deleting their objects.
		virtual void reset();
		/** @brief Loads a URDF from the ros-param named by our
		 * "Robot Description" property, iterates through the links, and
		 * loads any necessary models. */
		virtual void load();

		std::string robot_model_;


	private Q_SLOTS:
		// Helper function to apply color and alpha to all visuals.
		// Set the current color and alpha values for each visual.
		void updateRobotModel();
		void updateCoPColorAndAlpha();


	// Function to handle an incoming ROS message.
	private:
		// This is our callback to handle an incoming message.
		void processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg);

		// properties to show on side panel
		rviz::Property* cop_category_;

		// Property objects for user-editable properties.
		rviz::StringProperty* robot_model_property_;
		rviz::ColorProperty* cop_color_property_;
        rviz::FloatProperty* cop_alpha_property_;
        rviz::FloatProperty* cop_radius_property_;

        boost::shared_ptr<PointVisual> visual_;

        dwl::model::WholeBodyDynamics dynamics_;
};

} //@namespace dwl_rviz_plugin

#endif
