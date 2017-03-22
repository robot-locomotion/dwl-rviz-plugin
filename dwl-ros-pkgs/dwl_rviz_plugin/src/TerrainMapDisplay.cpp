#include <dwl_rviz_plugin/TerrainMapDisplay.h>

#include <QObject>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>

#include <dwl/environment/SpaceDiscretization.h>

#include <sstream>


using namespace rviz;

namespace dwl_rviz_plugin
{

enum VoxelColorMode {FULL_COLOR, GREY};

TerrainMapDisplay::TerrainMapDisplay() : rviz::Display(), messages_received_(0),
		color_factor_(0.8),	grid_size_(std::numeric_limits<double>::max()),
		max_cost_(0.), min_cost_(std::numeric_limits<double>::max()),
		min_key_z_(std::numeric_limits<unsigned int>::max())
{
	topic_property_ =
			new RosTopicProperty("Topic",
								 "",
								 QString::fromStdString(ros::message_traits::datatype<dwl_terrain::TerrainMap>()),
								 "dwl_terrain::TerrainMap topic to subscribe to terrain map",
								 this, SLOT(updateTopic()));

	queue_size_property_ =
			new IntProperty("Queue Size",
							queue_size_,
							"Advanced: set the size of the incoming message queue. Increasing this "
							"is useful if your incoming TF data is delayed significantly from your"
							" image data, but it can greatly increase memory usage if the messages are big.",
							this, SLOT(updateQueueSize()));

	queue_size_property_->setMin(1);

	// Category Groups
	cost_category_ = new rviz::Property("CostMap", QVariant(), "", this);
	normal_category_ = new rviz::Property("Surface Normal", QVariant(), "", this);


	// Voxel color properties
	voxel_color_property_ =
			new rviz::EnumProperty("Color", "Full Color",
								   "Select voxel coloring.",
								   cost_category_,
								   SLOT(updateColorMode()), this);
	voxel_color_property_->addOption("Full Color", FULL_COLOR);
	voxel_color_property_->addOption("Grey", GREY);


	// Surface normal vector properties
	normal_color_property_ =
			new rviz::ColorProperty("Color", QColor(255, 85, 127),
									"Color of a point",
									normal_category_, SLOT(updateNormalArrowGeometry()), this);

	normal_alpha_property_ =
			new rviz::FloatProperty("Alpha", 0.6,
									"0 is fully transparent, 1.0 is fully opaque.",
									normal_category_, SLOT(updateNormalArrowGeometry()), this);
	normal_alpha_property_->setMin(0);
	normal_alpha_property_->setMax(1);

	normal_shaft_length_property_ =
			new FloatProperty("Shaft Length", 0.04,
							  "Length of the arrow's shaft, in meters.",
							  normal_category_, SLOT(updateNormalArrowGeometry()), this);

	normal_shaft_radius_property_ =
			new FloatProperty("Shaft Radius", 0.005,
							  "Radius of the arrow's shaft, in meters.",
							  normal_category_, SLOT(updateNormalArrowGeometry()), this);

	normal_head_length_property_ =
			new FloatProperty("Head Length", 0.05,
							  "Length of the arrow's head, in meters.",
							  normal_category_, SLOT(updateNormalArrowGeometry()), this);

	normal_head_radius_property_ =
			new FloatProperty("Head Radius", 0.012,
							  "Radius of the arrow's head, in meters.",
							  normal_category_, SLOT(updateNormalArrowGeometry()), this);

	new_points_received_ = false;
}


TerrainMapDisplay::~TerrainMapDisplay()
{
	unsubscribe();

	delete cloud_;
	destroyObjects();

	if (scene_node_)
		scene_node_->detachAllObjects();
}


void TerrainMapDisplay::update(float wall_dt, float ros_dt)
{
	if (new_points_received_) {
		boost::mutex::scoped_lock lock(mutex_);

		// Drawing the terrain cost values
		cloud_->clear();
		cloud_->setDimensions(grid_size_, grid_size_, height_size_);
		cloud_->addPoints(&new_points_.front(), new_points_.size());

		// Drawing the normal vectors
		arrow_cloud_.clear();
		arrow_cloud_.resize(normal_buf_.size());
		for (unsigned int j = 0; j < normal_buf_.size(); j++) {
			boost::shared_ptr<ArrowVisual> arrow;
			arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));

			arrow->setArrow(normal_buf_[j].origin, normal_buf_[j].orientation);
			arrow->setFramePosition(position_);
			arrow->setFrameOrientation(orientation_);

			// Setting the arrow color and properties
			Ogre::ColourValue color = normal_color_property_->getOgreColor();
			color.a = normal_alpha_property_->getFloat();
			arrow->setColor(color.r, color.g, color.b, color.a);
			float shaft_length = normal_shaft_length_property_->getFloat();
			float shaft_radius = normal_shaft_radius_property_->getFloat();
			float head_length = normal_head_length_property_->getFloat();
			float head_radius = normal_head_radius_property_->getFloat();
			arrow->setProperties(shaft_length, shaft_radius,
								 head_length, head_radius);

			arrow_cloud_[j] = arrow;
		}

		new_points_received_ = false;
	}
}


void TerrainMapDisplay::reset()
{
	clear();
	messages_received_ = 0;
	setStatus(StatusProperty::Ok, "Messages",
			QString("0 terrain map messages received"));
}


void TerrainMapDisplay::onInitialize()
{
	boost::mutex::scoped_lock lock(mutex_);

	std::stringstream sname;
	sname << "PointCloud Nr.";// << i;
	cloud_ = new rviz::PointCloud();
	cloud_->setName(sname.str());
	cloud_->setRenderMode(rviz::PointCloud::RM_BOXES);
	scene_node_->attachObject((Ogre::MovableObject*) cloud_);
}


void TerrainMapDisplay::onEnable()
{
	scene_node_->setVisible(true);
	subscribe();
}


void TerrainMapDisplay::onDisable()
{
	scene_node_->setVisible(false);
	unsubscribe();

	clear();
}


void TerrainMapDisplay::destroyObjects()
{
	point_buf_.clear();
	new_points_.clear();
	normal_buf_.clear();
}


void TerrainMapDisplay::subscribe()
{
	if (!isEnabled())
		return;

	try {
		unsubscribe();

		const std::string& topicStr = topic_property_->getStdString();

		if (!topicStr.empty()) {
			sub_.reset(new message_filters::Subscriber<dwl_terrain::TerrainMap>());

			sub_->subscribe(threaded_nh_, topicStr, queue_size_);
			sub_->registerCallback(boost::bind(&TerrainMapDisplay::incomingMessageCallback, this, _1));
		}
	}
	catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic",
				(std::string("Error subscribing: ") + e.what()).c_str());
	}
}


void TerrainMapDisplay::unsubscribe()
{
	clear();

	try {
		// reset filters
		sub_.reset();
	}
	catch (ros::Exception& e) {
		setStatus(StatusProperty::Error, "Topic",
				(std::string("Error unsubscribing: ") + e.what()).c_str());
	}
}


void TerrainMapDisplay::incomingMessageCallback(const dwl_terrain::TerrainMapConstPtr& msg)
{
	++messages_received_;
	setStatus(StatusProperty::Ok, "Messages",
			QString::number(messages_received_) + " terrain map messages received");

	boost::mutex::scoped_lock lock(mutex_);
	terrain_msg_ = msg;

	// Destroy all the old elements
	destroyObjects();

	// Getting tf transform
	if (!context_->getFrameManager()->getTransform(terrain_msg_->header,
												   position_,
												   orientation_)) {
		std::stringstream ss;
		ss << "Failed to transform from frame [";
		ss << terrain_msg_->header.frame_id << "] to frame ["
		   << context_->getFrameManager()->getFixedFrame() << "]";
		this->setStatusStd(StatusProperty::Error, "Message", ss.str());

		return;
	}
	scene_node_->setOrientation(orientation_);
	scene_node_->setPosition(position_);

	// Getting the number of cells
	unsigned int num_cells = terrain_msg_->cell.size();

	// Computing the maximum and minimum cost of the map, and minimum key
	// of the height
	max_cost_ = 0.;
	min_cost_ = std::numeric_limits<double>::max();
	min_key_z_ = std::numeric_limits<unsigned int>::max();
	for (unsigned int i = 0; i < num_cells; i++) {
		double cost = terrain_msg_->cell[i].cost;
		if (max_cost_ < cost)
			max_cost_ = cost;

		if (min_cost_ > cost)
			min_cost_ = cost;

		if (min_key_z_ > terrain_msg_->cell[i].key_z)
			min_key_z_ = terrain_msg_->cell[i].key_z;
	}
	grid_size_ = terrain_msg_->plane_size;
	height_size_ = terrain_msg_->height_size;


	// Getting terrain values and size of the pixel
	dwl::environment::SpaceDiscretization space_discretization(grid_size_);
	space_discretization.setEnvironmentResolution(height_size_, false);
	PointCloud::Point new_point;
	for (unsigned int i = 0; i < num_cells; i++) {
		// Getting the Cartesian information of the terrain map
		double x, y, z;
		space_discretization.keyToCoord(x, terrain_msg_->cell[i].key_x, true);
		space_discretization.keyToCoord(y, terrain_msg_->cell[i].key_y, true);
		space_discretization.keyToCoord(z, terrain_msg_->cell[i].key_z, false);
		Ogre::Vector3 cell_position(x, y, z);

		unsigned int key_z = terrain_msg_->cell[i].key_z;
		while (key_z >= min_key_z_) {
			space_discretization.keyToCoord(z, key_z, false);
			Ogre::Vector3 cell_position(x, y, z);
			new_point.position = cell_position;

			// Setting the color of the cell according the reward value
			setColor(terrain_msg_->cell[i].cost,
					 max_cost_, min_cost_,
					 color_factor_, new_point);
			key_z -= 1;

			point_buf_.push_back(new_point);
		}


		// Defining the surface normal orientation
		Eigen::Vector3d ref_dir = -Eigen::Vector3d::UnitZ();
		Eigen::Quaterniond normal_q;
		Eigen::Vector3d normal(msg->cell[i].normal.x,
							   msg->cell[i].normal.y,
							   msg->cell[i].normal.z);
		normal_q.setFromTwoVectors(ref_dir, normal);
		Ogre::Quaternion normal_orientation(normal_q.w(), normal_q.x(),
											normal_q.y(), normal_q.z());

		Normal new_normal;
		new_normal.setNormal(cell_position, normal_orientation);
		normal_buf_.push_back(new_normal);
	}

	// Recording the data from the buffers
	new_points_.swap(point_buf_);

	new_points_received_ = true;
}


void TerrainMapDisplay::setColor(double cost_value,
								 double max_cost,
								 double min_cost,
								 double factor,
								 rviz::PointCloud::Point& point)
{
	VoxelColorMode color_mode =
			static_cast<VoxelColorMode>(voxel_color_property_->getOptionInt());

	switch (color_mode)
	{
	case FULL_COLOR:
	{
		int i;
		double m, n, f;

		double s = 1.0;
		double v = 1.0;

		double h = (1.0 -
				std::min(std::max(
						(cost_value - max_cost) / (min_cost - max_cost), 0.0), 1.0)) * factor;

		h -= floor(h);
		h *= 4;
		i = floor(h);
		f = h - i;
		if (!(i & 1))
			f = 1 - f; // if i is even
		m = v * (1 - s);
		n = v * (1 - s * f);

		switch (i)
		{
		case 0:
			point.setColor(m, n, v);
			break;
		case 1:
			point.setColor(m, v, 1-n);
			break;
		case 2:
			point.setColor(n, v, m);
			break;
		case 3:
			point.setColor(v, 1-n, m);
			break;
		default:
			point.setColor(1, 0.5, 0.5);
			break;
		}
		break;
	} case GREY:
	{
		double v = (cost_value - max_cost) / (min_cost - max_cost);
		point.setColor(v, v, v);
		break;
	} default:
		break;
	}
}


void TerrainMapDisplay::clear()
{
	boost::mutex::scoped_lock lock(mutex_);

	cloud_->clear();
	normal_buf_.clear();
}


void TerrainMapDisplay::updateQueueSize()
{
	queue_size_ = queue_size_property_->getInt();

	subscribe();
}


void TerrainMapDisplay::updateTopic()
{
	unsubscribe();
	reset();
	subscribe();
	context_->queueRender();
}


void TerrainMapDisplay::updateColorMode()
{
	if (messages_received_ != 0) {
		boost::mutex::scoped_lock lock(mutex_);

		// Clearing the voxel buffers
		point_buf_.clear();
		new_points_.clear();

		// Getting the number of cells
		unsigned int num_cells = terrain_msg_->cell.size();

		// Getting terrain values and size of the pixel
		dwl::environment::SpaceDiscretization space_discretization(grid_size_);
		space_discretization.setEnvironmentResolution(height_size_, false);
		PointCloud::Point new_point;
		for (unsigned int i = 0; i < num_cells; i++) {
			// Getting the Cartesian information of the terrain map
			double x, y, z;
			space_discretization.keyToCoord(x, terrain_msg_->cell[i].key_x, true);
			space_discretization.keyToCoord(y, terrain_msg_->cell[i].key_y, true);
			space_discretization.keyToCoord(z, terrain_msg_->cell[i].key_z, false);
			Ogre::Vector3 cell_position(x, y, z);

			unsigned int key_z = terrain_msg_->cell[i].key_z;
			while (key_z >= min_key_z_) {
				space_discretization.keyToCoord(z, key_z, false);
				Ogre::Vector3 cell_position(x, y, z);
				new_point.position = cell_position;

				// Setting the color of the cell according the cost value
				setColor(terrain_msg_->cell[i].cost,
						 max_cost_, min_cost_,
						 color_factor_, new_point);
				key_z -= 1;

				point_buf_.push_back(new_point);
			}
		}

		// Recording the data from the buffers
		new_points_.swap(point_buf_);

		// Drawing the terrain cost values
		if (new_points_.size() != 0) {
			cloud_->clear();
			cloud_->setDimensions(grid_size_, grid_size_, height_size_);
			cloud_->addPoints(&new_points_.front(), new_points_.size());
		}

		context_->queueRender();
	}
}


void TerrainMapDisplay::updateNormalArrowGeometry()
{
	Ogre::ColourValue color = normal_color_property_->getOgreColor();
	color.a = normal_alpha_property_->getFloat();
	float shaft_length = normal_shaft_length_property_->getFloat();
	float shaft_radius = normal_shaft_radius_property_->getFloat();
	float head_length = normal_head_length_property_->getFloat();
	float head_radius = normal_head_radius_property_->getFloat();

	for (unsigned int j = 0; j < arrow_cloud_.size(); j++) {
		arrow_cloud_[j]->setColor(color.r, color.g, color.b, color.a);
		arrow_cloud_[j]->setProperties(shaft_length, shaft_radius,
									   head_length, head_radius);
	}

	context_->queueRender();
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::TerrainMapDisplay, rviz::Display)
