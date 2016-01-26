#ifndef DWL_RVIZ_PLUGIN__LINE_VISUAL__H
#define DWL_RVIZ_PLUGIN__LINE_VISUAL__H

#include <rviz/properties/quaternion_property.h>
#include <Eigen/Dense>


namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Arrow;
}

namespace dwl_rviz_plugin
{

/**
 * @class LineVisual
 * @brief Visualizes 3d line
 * Each instance of LineVisual represents the visualization of
 * a single arrow data. Currently it just shows an line with
 * the initial and final points
 */
class LineVisual
{
	public:
		/**
		 * @brief Constructor that creates the visual stuff and puts it into the scene
		 * @param Ogre::SceneManager* Manager the organization and rendering of the scene
		 * @param Ogre::SceneNode* Represent the arrow as node in the scene
		 */
		LineVisual(Ogre::SceneManager* scene_manager,
				   Ogre::SceneNode* parent_node);

		/** @brief Destructor that removes the visual stuff from the scene */
		~LineVisual();

		/**
		 * @brief Configure the visual to show the line from their points
		 * @param const Ogre::Vector3& initial_point
		 * @param const Ogre::Vector3& final point
		 */
		void setArrow(const Ogre::Vector3& initial_point,
					  const Ogre::Vector3& final_point);

		/**
		 * @brief Set the position of the coordinate frame
		 * @param const Ogre::Vector3& Frame position
		 */
		void setFramePosition(const Ogre::Vector3& position);

		/**
		 * @brief Set the orientation of the coordinate frame
		 * @param const Ogre::Quaternion& Frame orientation
		 */
		void setFrameOrientation(const Ogre::Quaternion& orientation);

		/**
		 * @brief Set the color and alpha of the visual, which are user-editable
		 * @param float Red value
		 * @param float Green value
		 * @param float Blue value
		 * @param float Alpha value
		 */
		void setColor(float r, float g, float b, float a);

		/**
		 * @brief Set the parameters for this arrow
		 * @param shaft_diameter Diameter of the arrow's shaft
		 * @param head_length Length of the arrow's head
		 * @param head_diameter Diameter of the arrow's head
		 */
		void setProperties(float shaft_diameter,
						   float head_length = 0.,
						   float head_diameter = 0.);


	private:
		/** @brief The object implementing the arrow */
        rviz::Arrow* arrow_;

        /** @brief A SceneNode whose pose is set to match the coordinate frame */
		Ogre::SceneNode* frame_node_;

		/** @brief The SceneManager, kept here only so the destructor can ask it to destroy
		 * the ``frame_node_``.
		 */
		Ogre::SceneManager* scene_manager_;

		/** @brief Distance of the line */
		float distance_;
};

} //@namespace dwl_rviz_plugin

#endif
