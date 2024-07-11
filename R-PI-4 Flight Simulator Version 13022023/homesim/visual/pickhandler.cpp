#include "PickHandler.h"

#include <osg/io_utils>

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <iostream>

PickHandler::PickHandler(double devicePixelRatio)
  : devicePixelRatio_(devicePixelRatio)
{
}

PickHandler::~PickHandler()
{
}

bool   Printing = false;

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
		    switch (ea.getKey())
			{
			    case 'p':
				case 'P':
				    Printing = true;
    			return false;
			}
    
        default:
            return false;
    }
}
