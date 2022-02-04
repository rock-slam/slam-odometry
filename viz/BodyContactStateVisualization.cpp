#include "BodyContactStateVisualization.hpp"

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Shape>

using namespace vizkit3d;
using namespace odometry;

BodyContactStateVisualization::BodyContactStateVisualization()
    : contactColor( osg::Vec4(1,0,0,1) ), contactChannel(3)
{
}

BodyContactStateVisualization::~BodyContactStateVisualization()
{
}

void BodyContactStateVisualization::updateContactState( const odometry::BodyContactState& state )
{
    updateData( state );
}

osg::ref_ptr<osg::Node> BodyContactStateVisualization::createMainNode()
{
    return new osg::Group();
}

void BodyContactStateVisualization::updateMainNode(osg::Node* node)
{
    // clear node first
    node->asGroup()->removeChildren(0, node->asGroup()->getNumChildren());

    for(size_t j=0;j<state.points.size();j++)
    {
	const odometry::BodyContactPoint &cp(state.points[j]);
	const double radius = 0.01;
	const double visible = cp.contact;
	const osg::Vec3 pos( cp.position.x(), cp.position.y(), cp.position.z() ); 

	osg::Geode* geode = new osg::Geode;

	osg::Sphere* sphereShape = new osg::Sphere( pos, radius );
	osg::ShapeDrawable *sphere = new osg::ShapeDrawable( sphereShape ); 

	osg::Vec4 col = contactColor;
	col[contactChannel] = visible;

	sphere->setColor( col );
	geode->addDrawable( sphere );

	node->asGroup()->addChild( geode );
    }
}

void BodyContactStateVisualization::updateDataIntern(odometry::BodyContactState const& state)
{
    this->state = state;
}

QColor BodyContactStateVisualization::getContactColor() const
{
    QColor color;
    color.setRgbF(contactColor.x(), contactColor.y(), contactColor.z(), contactColor.w());
    return color;
}

void BodyContactStateVisualization::setContactColor(QColor color)
{
    contactColor.x() = color.redF();
    contactColor.y() = color.greenF();
    contactColor.z() = color.blueF();
    contactColor.w() = color.alphaF();
    emit propertyChanged("contact_color");
}

int BodyContactStateVisualization::getContactChannel() const
{
    return contactChannel;
}

void BodyContactStateVisualization::setContactChannel(int channel)
{
    contactChannel = std::max( 0, std::min( channel, 3 ) );
    emit propertyChanged("contact_channel");
}

