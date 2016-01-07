/////////////////////////////////////////////////////////////////////////////
//
// CXAnimation.cpp
//
// Remi Coulom
//
// December, 2003
//
/////////////////////////////////////////////////////////////////////////////
#include "CXAnimation.h"

#ifdef SAVEXPMFILES
#include <X11/xpm.h>
#include <sstream>
#include <iomanip>
#endif

/////////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////////
CXAnimation::CXAnimation(double xMinInit,
                         double yMinInit,
                         double xMaxInit,
                         double yMaxInit,
                         double ScaleInit,
                         double TimeStepInit) :
 CXWindow(int((xMaxInit - xMinInit) * ScaleInit),
          int((yMaxInit - yMinInit) * ScaleInit)),
 CAnimation(xMinInit, yMinInit, xMaxInit, yMaxInit, ScaleInit, TimeStepInit),
 Width(int((xMaxInit - xMinInit) * ScaleInit)),
 Height(int((yMaxInit - yMinInit) * ScaleInit))
{
 pixmap = XCreatePixmap(display,
                        win,
                        Width,
                        Height,
                        DefaultDepth(display, screen));

 pixmapBuffer = XCreatePixmap(display,
                              win,
                              Width,
                              Height,
                              DefaultDepth(display, screen));

 Erase();
 SwitchBuffers();
 Erase();

 XSelectInput(display, win, ExposureMask | ButtonPressMask);
}

/////////////////////////////////////////////////////////////////////////////
// Destructor
/////////////////////////////////////////////////////////////////////////////
CXAnimation::~CXAnimation()
{
 XFreePixmap(display, pixmap);
 XFreePixmap(display, pixmapBuffer);
}

/////////////////////////////////////////////////////////////////////////////
// Events
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::Events()
{
 XEvent event;
// while (XCheckWindowEvent(display, win, ExposureMask | ButtonPressMask, &event))
 while (XCheckIfEvent(display, &event, Predicate, XPointer(win)))
 {
  switch(event.type)
  {
   case Expose: /////////////////////////////////////////////////////////////
    while (XCheckWindowEvent(display, win, ExposureMask, &event));
    if (event.xexpose.count == 0)
     XCopyArea(display, pixmap, win, gc, 0, 0, Width, Height, 0, 0);
   break;

   case ButtonPress: ////////////////////////////////////////////////////////
   {
    double x, y;
    ConvertFromPixels(event.xbutton.x, event.xbutton.y, x, y);
    ButtonPressed(x, y);
   }
   break;

   case ClientMessage: //////////////////////////////////////////////////////
    if (Atom(event.xclient.data.l[0]) == wm_delete_window)
     SetStopFlag();
   break;
  }
 }
}

/////////////////////////////////////////////////////////////////////////////
// Color
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::SetColor(double r, double g, double b)
{
 unsigned long c = int(r * RMax) * RMult +
                   int(g * GMax) * GMult +
                   int(b * BMax) * BMult;
 XSetForeground(display, gc, c);
}

/////////////////////////////////////////////////////////////////////////////
// Dot
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::Dot(double x, double y)
{
 int ix;
 int iy;
 ConvertToPixels(x, y, ix, iy);
 XFillRectangle(display, pixmapBuffer, gc, ix - 2, iy - 2, 5, 5);
}

/////////////////////////////////////////////////////////////////////////////
// Line
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::Line(double x0, double y0, double x1, double y1)
{
 int ix0, iy0, ix1, iy1;
 ConvertToPixels(x0, y0, ix0, iy0);
 ConvertToPixels(x1, y1, ix1, iy1);
 XDrawLine(display, pixmapBuffer, gc, ix0, iy0, ix1, iy1);
}

/////////////////////////////////////////////////////////////////////////////
// Erase the background of the current buffer
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::Erase()
{
 XSetForeground(display, gc, White.pixel);
 XFillRectangle(display, pixmapBuffer, gc, 0, 0, Width, Height);
 XSetForeground(display, gc, Black.pixel);
}

/////////////////////////////////////////////////////////////////////////////
// Switch buffers
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::SwitchBuffers()
{
 Pixmap pixmapTmp = pixmap;
 pixmap = pixmapBuffer;
 pixmapBuffer = pixmapTmp;
}

/////////////////////////////////////////////////////////////////////////////
// New frame
/////////////////////////////////////////////////////////////////////////////
void CXAnimation::NewFrame()
{
 SwitchBuffers();

 //
 // Wait and display the buffer
 //
 clktimer.WaitInterval(int(GetTimeStep() * 1000));
 XCopyArea(display, pixmap, win, gc, 0, 0, Width, Height, 0, 0);
 XSync(display, 0);
 Erase();

 //
 // Save frames to files to create a movie
 //
#ifdef SAVEXPMFILES
 static int Counter = 0;
 std::ostringstream ossFileName;
 ossFileName << "./frame-";
 ossFileName << std::setfill('0') << std::setw(6) << Counter;
 ossFileName << ".xpm.gz";
 Counter++;
 XpmWriteFileFromPixmap(display, (char *)ossFileName.str().c_str(), pixmap, 0, 0);
#endif
}
