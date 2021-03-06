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

public class ArVersalogicIO {
  /* (begin code from javabody typemap) */

  private long swigCPtr;
  protected boolean swigCMemOwn;

  /* for internal use by swig only */
  public ArVersalogicIO(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArVersalogicIO obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  /* (end code from javabody typemap) */

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      AriaJavaJNI.delete_ArVersalogicIO(swigCPtr);
    }
    swigCPtr = 0;
  }

  public ArVersalogicIO(String dev) {
    this(AriaJavaJNI.new_ArVersalogicIO__SWIG_0(dev), true);
  }

  public ArVersalogicIO() {
    this(AriaJavaJNI.new_ArVersalogicIO__SWIG_1(), true);
  }

  public boolean closeIO() {
    return AriaJavaJNI.ArVersalogicIO_closeIO(swigCPtr, this);
  }

  public boolean isEnabled() {
    return AriaJavaJNI.ArVersalogicIO_isEnabled(swigCPtr, this);
  }

  public boolean isAnalogSupported() {
    return AriaJavaJNI.ArVersalogicIO_isAnalogSupported(swigCPtr, this);
  }

  public boolean getAnalogValue(int port, SWIGTYPE_p_double val) {
    return AriaJavaJNI.ArVersalogicIO_getAnalogValue(swigCPtr, this, port, SWIGTYPE_p_double.getCPtr(val));
  }

  public boolean getAnalogValueRaw(int port, SWIGTYPE_p_int val) {
    return AriaJavaJNI.ArVersalogicIO_getAnalogValueRaw(swigCPtr, this, port, SWIGTYPE_p_int.getCPtr(val));
  }

  public ArVersalogicIO.Direction getDigitalBankDirection(int bank) {
    return ArVersalogicIO.Direction.swigToEnum(AriaJavaJNI.ArVersalogicIO_getDigitalBankDirection(swigCPtr, this, bank));
  }

  public boolean setDigitalBankDirection(int bank, ArVersalogicIO.Direction dir) {
    return AriaJavaJNI.ArVersalogicIO_setDigitalBankDirection(swigCPtr, this, bank, dir.swigValue());
  }

  public boolean getDigitalBankInputs(int bank, SWIGTYPE_p_unsigned_char val) {
    return AriaJavaJNI.ArVersalogicIO_getDigitalBankInputs(swigCPtr, this, bank, SWIGTYPE_p_unsigned_char.getCPtr(val));
  }

  public boolean getDigitalBankOutputs(int bank, SWIGTYPE_p_unsigned_char val) {
    return AriaJavaJNI.ArVersalogicIO_getDigitalBankOutputs(swigCPtr, this, bank, SWIGTYPE_p_unsigned_char.getCPtr(val));
  }

  public boolean setDigitalBankOutputs(int bank, short val) {
    return AriaJavaJNI.ArVersalogicIO_setDigitalBankOutputs(swigCPtr, this, bank, val);
  }

  public boolean getSpecialControlRegister(SWIGTYPE_p_unsigned_char val) {
    return AriaJavaJNI.ArVersalogicIO_getSpecialControlRegister(swigCPtr, this, SWIGTYPE_p_unsigned_char.getCPtr(val));
  }

  public int lock() {
    return AriaJavaJNI.ArVersalogicIO_lock(swigCPtr, this);
  }

  public int unlock() {
    return AriaJavaJNI.ArVersalogicIO_unlock(swigCPtr, this);
  }

  public int tryLock() {
    return AriaJavaJNI.ArVersalogicIO_tryLock(swigCPtr, this);
  }

  public final static class Direction {
    public final static Direction DIGITAL_INPUT = new Direction("DIGITAL_INPUT");
    public final static Direction DIGITAL_OUTPUT = new Direction("DIGITAL_OUTPUT");

    public final int swigValue() {
      return swigValue;
    }

    public String toString() {
      return swigName;
    }

    public static Direction swigToEnum(int swigValue) {
      if (swigValue < swigValues.length && swigValue >= 0 && swigValues[swigValue].swigValue == swigValue)
        return swigValues[swigValue];
      for (int i = 0; i < swigValues.length; i++)
        if (swigValues[i].swigValue == swigValue)
          return swigValues[i];
      throw new IllegalArgumentException("No enum " + Direction.class + " with value " + swigValue);
    }

    private Direction(String swigName) {
      this.swigName = swigName;
      this.swigValue = swigNext++;
    }

    private Direction(String swigName, int swigValue) {
      this.swigName = swigName;
      this.swigValue = swigValue;
      swigNext = swigValue+1;
    }

    private Direction(String swigName, Direction swigEnum) {
      this.swigName = swigName;
      this.swigValue = swigEnum.swigValue;
      swigNext = this.swigValue+1;
    }

    private static Direction[] swigValues = { DIGITAL_INPUT, DIGITAL_OUTPUT };
    private static int swigNext = 0;
    private final int swigValue;
    private final String swigName;
  }

}
