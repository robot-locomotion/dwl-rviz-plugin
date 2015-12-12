#include <dwl_rviz_plugin/WholeBodyStateDisplay.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>

#include <rviz/default_plugin/point_visual.h>


using namespace rviz;

namespace dwl_rviz_plugin
{

WholeBodyStateDisplay::WholeBodyStateDisplay()
{
	color_property_ =
			new rviz::ColorProperty("Color", QColor(204, 41, 204),
									"Color of a point",
									this, SLOT(updateColorAndAlpha()));

	alpha_property_ =
			new rviz::FloatProperty("Alpha", 1.0,
									"0 is fully transparent, 1.0 is fully opaque.",
									this, SLOT(updateColorAndAlpha()));

	radius_property_ =
			new rviz::FloatProperty("Radius", 0.2,
									"Radius of a point",
									this, SLOT(updateColorAndAlpha()));

	history_length_property_ =
			new rviz::IntProperty("History Length", 1,
								  "Number of prior measurements to display.",
								  this, SLOT(updateHistoryLength()));
	history_length_property_->setMin(1);
	history_length_property_->setMax(100000);



	robot_model_property_ = new StringProperty("Robot Description", "robot_model",
												"Name of the parameter to search for to load"
												" the robot description.",
												this, SLOT(updateRobotDescription()));
}


WholeBodyStateDisplay::~WholeBodyStateDisplay()
{

}


void WholeBodyStateDisplay::clear()
{
//	robot_->clear();
	clearStatuses();
	robot_model_.clear();
}


void WholeBodyStateDisplay::onInitialize()
{
	MFDClass::onInitialize();
	updateHistoryLength();
}


void WholeBodyStateDisplay::reset()
{
	MFDClass::reset();
	visuals_.clear();
}


void WholeBodyStateDisplay::load()
{
	std::string content;
	if (!update_nh_.getParam(robot_model_property_->getStdString(), content)) {
		std::string loc;
		if (update_nh_.searchParam(robot_model_property_->getStdString(), loc)) {
			update_nh_.getParam(loc, content);
		} else {
			clear();
			setStatus(StatusProperty::Error, "URDF",
					  "Parameter [" + robot_model_property_->getString()
					  + "] does not exist, and was not found by searchParam()");
			return;
		}
	}

	if (content.empty()) {
		clear();
		setStatus(StatusProperty::Error, "URDF", "URDF is empty");
		return;
	}

	if (content == robot_model_) {
		return;
	}

	robot_model_ = content;

//	TiXmlDocument doc;
//	doc.Parse(robot_model_.c_str());
//	if (!doc.RootElement()) {
//		clear();
//		setStatus(StatusProperty::Error, "URDF", "URDF failed XML parse");
//		return;
//	}

//	urdf::Model descr;
//	if (!descr.initXml(doc.RootElement())) {
//		clear();
//		setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
//		return;
//	}

	setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
//	robot_->load( descr );
//	robot_->update( TFLinkUpdater(context_->getFrameManager(),
//								  boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
//								  tf_prefix_property_->getStdString() ));
}



void WholeBodyStateDisplay::updateColorAndAlpha()
{
	float alpha = alpha_property_->getFloat();
	float radius = radius_property_->getFloat();
	Ogre::ColourValue color = color_property_->getOgreColor();

	for (size_t i = 0; i < visuals_.size(); i++) {
		visuals_[i]->setColor(color.r, color.g, color.b, alpha);
		visuals_[i]->setRadius(radius);
	}
}


void WholeBodyStateDisplay::updateHistoryLength()
{
	visuals_.rset_capacity(history_length_property_->getInt());
}


void WholeBodyStateDisplay::updateRobotDescription()
{
	if (isEnabled()) {
		load();
		context_->queueRender();
	}
}


void WholeBodyStateDisplay::processMessage(const dwl_msgs::WholeBodyState::ConstPtr& msg)
{
/*	if (!rviz::validateFloats(msg->point)) {
		setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)");
		return;
	}

	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Point message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
												   msg->header.stamp,
												   position, orientation)) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
				  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
		return;
	}

	// We are keeping a circular buffer of visual pointers.  This gets
	// the next one, or creates and stores it if the buffer is not full
	boost::shared_ptr<PointStampedVisual> visual;
	if (visuals_.full()) {
		visual = visuals_.front();
	} else {
		visual.reset(new PointStampedVisual(context_->getSceneManager(), scene_node_));
	}


	// Now set or update the contents of the chosen visual.
	visual->setMessage(msg);
	visual->setFramePosition(position);
	visual->setFrameOrientation(orientation);
	float alpha = alpha_property_->getFloat();
	float radius = radius_property_->getFloat();
	Ogre::ColourValue color = color_property_->getOgreColor();
	visual->setColor(color.r, color.g,  color.b, alpha);
	visual->setRadius(radius);


	// And send it to the end of the circular buffer
	visuals_.push_back(visual);*/
}

} //@namespace dwl_rviz_plugin


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dwl_rviz_plugin::WholeBodyStateDisplay, rviz::Display)
