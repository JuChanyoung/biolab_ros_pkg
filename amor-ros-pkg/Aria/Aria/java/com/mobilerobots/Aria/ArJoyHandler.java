/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 1.3.39
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package com.mobilerobots.Aria;

public class ArJoyHandler {
  /* (begin code from javabody typemap) */

  private long swigCPtr;
  protected boolean swigCMemOwn;

  /* for internal use by swig only */
  public ArJoyHandler(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArJoyHandler obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  /* (end code from javabody typemap) */

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      AriaJavaJNI.delete_ArJoyHandler(swigCPtr);
    }
    swigCPtr = 0;
  }

  public ArJoyHandler(boolean useOSCal, boolean useOldJoystick) {
    this(AriaJavaJNI.new_ArJoyHandler__SWIG_0(useOSCal, useOldJoystick), true);
  }

  public ArJoyHandler(boolean useOSCal) {
    this(AriaJavaJNI.new_ArJoyHandler__SWIG_1(useOSCal), true);
  }

  public ArJoyHandler() {
    this(AriaJavaJNI.new_ArJoyHandler__SWIG_2(), true);
  }

  public boolean init() {
    return AriaJavaJNI.ArJoyHandler_init(swigCPtr, this);
  }

  public boolean haveJoystick() {
    return AriaJavaJNI.ArJoyHandler_haveJoystick(swigCPtr, this);
  }

  public void getDoubles(SWIGTYPE_p_double x, SWIGTYPE_p_double y, SWIGTYPE_p_double z) {
    AriaJavaJNI.ArJoyHandler_getDoubles__SWIG_0(swigCPtr, this, SWIGTYPE_p_double.getCPtr(x), SWIGTYPE_p_double.getCPtr(y), SWIGTYPE_p_double.getCPtr(z));
  }

  public void getDoubles(SWIGTYPE_p_double x, SWIGTYPE_p_double y) {
    AriaJavaJNI.ArJoyHandler_getDoubles__SWIG_1(swigCPtr, this, SWIGTYPE_p_double.getCPtr(x), SWIGTYPE_p_double.getCPtr(y));
  }

  public boolean getButton(long button) {
    return AriaJavaJNI.ArJoyHandler_getButton(swigCPtr, this, button);
  }

  public boolean haveZAxis() {
    return AriaJavaJNI.ArJoyHandler_haveZAxis(swigCPtr, this);
  }

  public void setSpeeds(int x, int y, int z) {
    AriaJavaJNI.ArJoyHandler_setSpeeds__SWIG_0(swigCPtr, this, x, y, z);
  }

  public void setSpeeds(int x, int y) {
    AriaJavaJNI.ArJoyHandler_setSpeeds__SWIG_1(swigCPtr, this, x, y);
  }

  public void getAdjusted(SWIGTYPE_p_int x, SWIGTYPE_p_int y, SWIGTYPE_p_int z) {
    AriaJavaJNI.ArJoyHandler_getAdjusted__SWIG_0(swigCPtr, this, SWIGTYPE_p_int.getCPtr(x), SWIGTYPE_p_int.getCPtr(y), SWIGTYPE_p_int.getCPtr(z));
  }

  public void getAdjusted(SWIGTYPE_p_int x, SWIGTYPE_p_int y) {
    AriaJavaJNI.ArJoyHandler_getAdjusted__SWIG_1(swigCPtr, this, SWIGTYPE_p_int.getCPtr(x), SWIGTYPE_p_int.getCPtr(y));
  }

  public long getNumAxes() {
    return AriaJavaJNI.ArJoyHandler_getNumAxes(swigCPtr, this);
  }

  public double getAxis(long axis) {
    return AriaJavaJNI.ArJoyHandler_getAxis(swigCPtr, this, axis);
  }

  public long getNumButtons() {
    return AriaJavaJNI.ArJoyHandler_getNumButtons(swigCPtr, this);
  }

  public void setUseOSCal(boolean useOSCal) {
    AriaJavaJNI.ArJoyHandler_setUseOSCal(swigCPtr, this, useOSCal);
  }

  public boolean getUseOSCal() {
    return AriaJavaJNI.ArJoyHandler_getUseOSCal(swigCPtr, this);
  }

  public void startCal() {
    AriaJavaJNI.ArJoyHandler_startCal(swigCPtr, this);
  }

  public void endCal() {
    AriaJavaJNI.ArJoyHandler_endCal(swigCPtr, this);
  }

  public void getUnfiltered(SWIGTYPE_p_int x, SWIGTYPE_p_int y, SWIGTYPE_p_int z) {
    AriaJavaJNI.ArJoyHandler_getUnfiltered__SWIG_0(swigCPtr, this, SWIGTYPE_p_int.getCPtr(x), SWIGTYPE_p_int.getCPtr(y), SWIGTYPE_p_int.getCPtr(z));
  }

  public void getUnfiltered(SWIGTYPE_p_int x, SWIGTYPE_p_int y) {
    AriaJavaJNI.ArJoyHandler_getUnfiltered__SWIG_1(swigCPtr, this, SWIGTYPE_p_int.getCPtr(x), SWIGTYPE_p_int.getCPtr(y));
  }

  public void getStats(SWIGTYPE_p_int maxX, SWIGTYPE_p_int minX, SWIGTYPE_p_int maxY, SWIGTYPE_p_int minY, SWIGTYPE_p_int cenX, SWIGTYPE_p_int cenY) {
    AriaJavaJNI.ArJoyHandler_getStats(swigCPtr, this, SWIGTYPE_p_int.getCPtr(maxX), SWIGTYPE_p_int.getCPtr(minX), SWIGTYPE_p_int.getCPtr(maxY), SWIGTYPE_p_int.getCPtr(minY), SWIGTYPE_p_int.getCPtr(cenX), SWIGTYPE_p_int.getCPtr(cenY));
  }

  public void setStats(int maxX, int minX, int maxY, int minY, int cenX, int cenY) {
    AriaJavaJNI.ArJoyHandler_setStats(swigCPtr, this, maxX, minX, maxY, minY, cenX, cenY);
  }

  public void getSpeeds(SWIGTYPE_p_int x, SWIGTYPE_p_int y, SWIGTYPE_p_int z) {
    AriaJavaJNI.ArJoyHandler_getSpeeds(swigCPtr, this, SWIGTYPE_p_int.getCPtr(x), SWIGTYPE_p_int.getCPtr(y), SWIGTYPE_p_int.getCPtr(z));
  }

}
