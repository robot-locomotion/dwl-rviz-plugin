#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>
#include <dwl_rviz_plugin/PolygonVisual.h>


namespace dwl_rviz_plugin
{

unsigned int factorial(unsigned int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}


PolygonVisual::PolygonVisual(Ogre::SceneManager* scene_manager,
							 Ogre::SceneNode* parent_node)
{
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the transform (position and
	// orientation) of itself relative to its parent. Ogre does the math of combining those
	// transforms when it is time to render.
	// Here we create a node to store the pose of the Point's header frame relative to the RViz
	// fixed frame.
	frame_node_ = parent_node->createChildSceneNode();
}


PolygonVisual::~PolygonVisual()
{
	// Delete the line to make it disappear.
	line_.clear();

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode(frame_node_);
}


void PolygonVisual::setVertexs(std::vector<Ogre::Vector3>& vertexs)
{
	line_.clear();

	unsigned int num_vertex = vertexs.size();
	unsigned int num_line = factorial(num_vertex - 1);
	line_.resize(num_line);

	unsigned int counter = 0;
	unsigned int tree = num_vertex;
	while (tree > 1) {
		unsigned int current_it = num_vertex - tree;
		for (unsigned int i = current_it; i < num_vertex - 1; i++) {
			// We create the line object within the frame node so that we can set its position and
			// direction relative to its header frame.
			line_[counter].reset(new rviz::Line(scene_manager_, frame_node_));

			line_[counter]->setPoints(vertexs[current_it], vertexs[i+1]);

			counter++;
		}
		tree--;
	}
}


void PolygonVisual::setFramePosition(const Ogre::Vector3& position)
{
	frame_node_->setPosition(position);
}


void PolygonVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
	frame_node_->setOrientation(orientation);
}


void PolygonVisual::setColor(float r, float g, float b, float a)
{
	unsigned int num_line = line_.size();
	for (unsigned int i = 0; i < num_line; i++) {
		line_[i]->setColor(r, g, b, a);
	}
}


void PolygonVisual::setScale(const Ogre::Vector3& scale)
{
	unsigned int num_line = line_.size();
	for (unsigned int i = 0; i < num_line; i++) {
		line_[i]->setScale(scale);
	}
}

} //@namespace dwl_rviz_plugin
