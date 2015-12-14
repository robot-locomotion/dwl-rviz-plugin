#ifndef DWL_RVIZ_PLUGIN__POLYGON_VISUAL__H
#define DWL_RVIZ_PLUGIN__POLYGON_VISUAL__H

#include <rviz/properties/quaternion_property.h>


namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Line;
}

namespace dwl_rviz_plugin
{

/**
 * @class PolygonVisual
 * @brief Visualizes the whole set of polygons given a vector of 3d points
 * Each instance of PolygonwVisual represents the visualization of a set of polygons data.
 * Currently it just shows a set of polygons
 */
class PolygonVisual
{
	public:
		/**
		 * @brief Constructor that creates the visual stuff and puts it into the scene
		 * @param Ogre::SceneManager* Manager the organization and rendering of the scene
		 * @param Ogre::SceneNode* Represent the arrow as node in the scene
		 */
		PolygonVisual(Ogre::SceneManager* scene_manager,
					  Ogre::SceneNode* parent_node);

		/** @brief Destructor that removes the visual stuff from the scene */
		~PolygonVisual();

		/**
		 * @brief Configure the visual to show the polygons
		 * @param const std::vector<Ogre::Vector3>& Vertex of the polygon
		 */
		void setVertexs(std::vector<Ogre::Vector3>& vertexs);

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
		 * @brief Set the scale of the line
		 * @param const Ogre::Vector3d& Line scale
		 */
		void setScale(const Ogre::Vector3& scale);


	private:
		/** @brief The object implementing the lines */
        std::vector<boost::shared_ptr<rviz::Line> > line_;

        /** @brief A SceneNode whose pose is set to match the coordinate frame */
		Ogre::SceneNode* frame_node_;

		/** @brief The SceneManager, kept here only so the destructor can ask it to destroy
		 * the ``frame_node_``.
		 */
		Ogre::SceneManager* scene_manager_;
};

} //@namespace dwl_rviz_plugin

#endif
