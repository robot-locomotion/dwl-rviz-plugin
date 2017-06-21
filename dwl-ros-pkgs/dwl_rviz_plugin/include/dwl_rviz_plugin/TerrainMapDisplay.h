#ifndef DWL_RVIZ_PLUGIN__TERRAIN_MAP_DISPLAY__H
#define DWL_RVIZ_PLUGIN__TERRAIN_MAP_DISPLAY__H

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <dwl_terrain/TerrainMap.h>
#include <dwl_rviz_plugin/ArrowVisual.h>

#include <rviz/display.h>
#include <rviz/ogre_helpers/point_cloud.h>


namespace rviz
{
class RosTopicProperty;
class IntProperty;
class FloatProperty;
class ColorProperty;
class EnumProperty;
} //@namespace rviz


namespace dwl_rviz_plugin
{

struct Normal
{
	void setNormal(Ogre::Vector3 position,
				   Ogre::Quaternion quaternion) {
		origin = position;
		orientation = quaternion;
	}

	Ogre::Vector3 origin;
	Ogre::Quaternion orientation;
};

/**
 * @class TerrainMapDisplay
 * @brief Rviz plugin for visualization of terrain map
 */
class TerrainMapDisplay : public rviz::Display
{
	Q_OBJECT
	public:
		/** @brief Constructor function */
		TerrainMapDisplay();

		/** @brief Destructor function */
		virtual ~TerrainMapDisplay();

		/**
		 * @brief Updates the information to display
		 * @param float wall_dt Wall delta time
		 * @param float ros_dt Ros delta time
		 */
		virtual void update(float wall_dt, float ros_dt);

		/** @brief Resets the information to display */
		virtual void reset();


	protected:
		/** @brief Method for initialization of the plugin */
		virtual void onInitialize();

		/** @brief Enables the display */
		virtual void onEnable();

		/** @brief Disable the display */
		virtual void onDisable();

		/** Destroy all the objects for visualization */
		void destroyObjects();

		/** @brief Subscribes to the topic */
		void subscribe();

		/** @brief Unsubscribes to the topic */
		void unsubscribe();

		/** @brief Processing of the incoming message */
		void incomingMessageCallback(const dwl_terrain::TerrainMapConstPtr& msg);

		/**
		 * @brief Sets the color of the reward value
		 * @param double Cost value of the cell
		 * @param double Maximum cost value of the map
		 * @param double Minimum cost value of the map
		 * @param double color_factor Color factor
		 * @param rviz::PointCloud::Point& point Point with color information
		 */
		void setColor(double cost_value,
					  double max_cost, double min_cost,
					  double color_factor, rviz::PointCloud::Point& point);

		/** Clears the display data */
		void clear();

		/** @brief Vector of points */
		typedef std::vector<rviz::PointCloud::Point> VPoint;
		typedef std::vector<Normal> VNormal;

		/** @brief Subscriber to the ObstacleMap messages */
		boost::shared_ptr<message_filters::Subscriber<dwl_terrain::TerrainMap> > sub_;

		/** @brief Mutex of thread */
		boost::mutex mutex_;

		/** @brief Ogre-rviz point clouds */
		rviz::PointCloud* cloud_;

		/** @brief Properties to show on side panel */
		rviz::Property* cost_category_;
		rviz::Property* normal_category_;

		/** @brief Property objects for user-editable properties */
		rviz::IntProperty* queue_size_property_;
		rviz::RosTopicProperty* topic_property_;
		rviz::EnumProperty* voxel_color_property_;
        rviz::ColorProperty* normal_color_property_;
        rviz::FloatProperty* normal_alpha_property_;
        rviz::FloatProperty* normal_head_radius_property_;
        rviz::FloatProperty* normal_head_length_property_;
        rviz::FloatProperty* normal_shaft_radius_property_;
        rviz::FloatProperty* normal_shaft_length_property_;

		/** @brief Max tree areas */
		int max_tree_areas_;

		/** @brief New points */
		VPoint new_points_;

		/** @brief Point buffer */
		VPoint point_buf_;

		/** @brief Array of normal vectors */
		VNormal normal_buf_;

		/** @brief Array of surface normals */
		std::vector<boost::shared_ptr<ArrowVisual> > arrow_cloud_;

		/** @brief Indicates if the new points was received */
		bool new_points_received_;

		/** @brief Queue size */
		u_int32_t queue_size_;

		/** @brief Number of received messages */
		uint32_t messages_received_;

		/** @brief Color factor */
		double color_factor_;

		/** @brief Grid size */
		double grid_size_;

		/** @brief Height size */
		double height_size_;


	private Q_SLOTS:
		/** @brief Updates queue size */
		void updateQueueSize();

		/** @brief Updates the topic name */
		void updateTopic();

		/** @brief Updates surface normal properties */
		void updateColorMode();
		void updateNormalArrowGeometry();


	private:
		/** @brief Current position and orientation */
		Ogre::Vector3 position_;
		Ogre::Quaternion orientation_;

		/** @brief Current terrain message */
		dwl_terrain::TerrainMapConstPtr terrain_msg_;

		/** @brief Terrain minimum and maximum values */
		double max_cost_;
		double min_cost_;
		unsigned int min_key_z_;
};

} //@namespace dwl_rviz_plugin


#endif
