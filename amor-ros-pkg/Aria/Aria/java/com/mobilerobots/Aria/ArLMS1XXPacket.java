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

public class ArLMS1XXPacket extends ArBasePacket {
  /* (begin code from javabody_derived typemap) */

  private long swigCPtr;

  /* for internal use by swig only */
  public ArLMS1XXPacket(long cPtr, boolean cMemoryOwn) {
    super(AriaJavaJNI.SWIGArLMS1XXPacketUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  /* for internal use by swig only */
  public static long getCPtr(ArLMS1XXPacket obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  /* (end code from javabody_derived typemap) */

  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if(swigCPtr != 0 && swigCMemOwn) {
      swigCMemOwn = false;
      AriaJavaJNI.delete_ArLMS1XXPacket(swigCPtr);
    }
    swigCPtr = 0;
    super.delete();
  }

  public ArLMS1XXPacket() {
    this(AriaJavaJNI.new_ArLMS1XXPacket(), true);
  }

  public String getCommandType() {
    return AriaJavaJNI.ArLMS1XXPacket_getCommandType(swigCPtr, this);
  }

  public String getCommandName() {
    return AriaJavaJNI.ArLMS1XXPacket_getCommandName(swigCPtr, this);
  }

  public void finalizePacket() {
    AriaJavaJNI.ArLMS1XXPacket_finalizePacket(swigCPtr, this);
  }

  public void resetRead() {
    AriaJavaJNI.ArLMS1XXPacket_resetRead(swigCPtr, this);
  }

  public ArTime getTimeReceived() {
    return new ArTime(AriaJavaJNI.ArLMS1XXPacket_getTimeReceived(swigCPtr, this), true);
  }

  public void setTimeReceived(ArTime timeReceived) {
    AriaJavaJNI.ArLMS1XXPacket_setTimeReceived(swigCPtr, this, ArTime.getCPtr(timeReceived), timeReceived);
  }

  public void duplicatePacket(ArLMS1XXPacket packet) {
    AriaJavaJNI.ArLMS1XXPacket_duplicatePacket(swigCPtr, this, ArLMS1XXPacket.getCPtr(packet), packet);
  }

  public void empty() {
    AriaJavaJNI.ArLMS1XXPacket_empty(swigCPtr, this);
  }

  public void byteToBuf(char val) {
    AriaJavaJNI.ArLMS1XXPacket_byteToBuf(swigCPtr, this, val);
  }

  public void byte2ToBuf(short val) {
    AriaJavaJNI.ArLMS1XXPacket_byte2ToBuf(swigCPtr, this, val);
  }

  public void byte4ToBuf(int val) {
    AriaJavaJNI.ArLMS1XXPacket_byte4ToBuf(swigCPtr, this, val);
  }

  public void uByteToBuf(short val) {
    AriaJavaJNI.ArLMS1XXPacket_uByteToBuf(swigCPtr, this, val);
  }

  public void uByte2ToBuf(int val) {
    AriaJavaJNI.ArLMS1XXPacket_uByte2ToBuf(swigCPtr, this, val);
  }

  public void uByte4ToBuf(long val) {
    AriaJavaJNI.ArLMS1XXPacket_uByte4ToBuf(swigCPtr, this, val);
  }

  public void strToBuf(String str) {
    AriaJavaJNI.ArLMS1XXPacket_strToBuf(swigCPtr, this, str);
  }

  public char bufToByte() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToByte(swigCPtr, this);
  }

  public short bufToByte2() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToByte2(swigCPtr, this);
  }

  public int bufToByte4() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToByte4(swigCPtr, this);
  }

  public short bufToUByte() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToUByte(swigCPtr, this);
  }

  public int bufToUByte2() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToUByte2(swigCPtr, this);
  }

  public long bufToUByte4() {
    return AriaJavaJNI.ArLMS1XXPacket_bufToUByte4(swigCPtr, this);
  }

  public void bufToStr(String buf, int len) {
    AriaJavaJNI.ArLMS1XXPacket_bufToStr(swigCPtr, this, buf, len);
  }

  public void rawCharToBuf(short c) {
    AriaJavaJNI.ArLMS1XXPacket_rawCharToBuf(swigCPtr, this, c);
  }

}
