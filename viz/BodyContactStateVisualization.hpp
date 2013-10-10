#ifndef ESLAM_BODYCONTACTSTATEVISUALIZATION_H
#define ESLAM_BODYCONTACTSTATEVISUALIZATION_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <odometry/ContactState.hpp>

namespace vizkit3d
{
    class BodyContactStateVisualization
        : public vizkit3d::Vizkit3DPlugin<odometry::BodyContactState>
        , boost::noncopyable
    {
    Q_OBJECT

    Q_PROPERTY(QColor contact_color READ getContactColor WRITE setContactColor)
    Q_PROPERTY(int contact_channel READ getContactChannel WRITE setContactChannel)

    public:
        BodyContactStateVisualization();
        ~BodyContactStateVisualization();

	Q_INVOKABLE void updateContactState( const odometry::BodyContactState& state );

    protected:
	virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(odometry::BodyContactState const& plan);
        
    public slots:
        QColor getContactColor() const;
        void setContactColor(QColor color);

	/** get the color channel on which to put the contact value */
	int getContactChannel() const;
	void setContactChannel(int channel);

    private:
	odometry::BodyContactState state;
	osg::Vec4 contactColor;
	int contactChannel;
    };
}
#endif
