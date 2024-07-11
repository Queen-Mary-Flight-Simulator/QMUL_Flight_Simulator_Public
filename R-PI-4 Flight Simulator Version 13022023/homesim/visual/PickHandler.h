#ifndef PickHandler_h__
#define PickHandler_h__

#include <osgGA/GUIEventHandler>

extern bool   Printing;
 
class PickHandler : public osgGA::GUIEventHandler
{
public:
  PickHandler( double devicePixelRatio = 1.0 );
  virtual ~PickHandler();

  virtual bool handle( const osgGA::GUIEventAdapter&  ea,
                             osgGA::GUIActionAdapter& aa );

private:
  double devicePixelRatio_;
};

#endif
