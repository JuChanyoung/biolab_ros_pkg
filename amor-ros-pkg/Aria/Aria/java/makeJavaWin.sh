#!/bin/bash
# Run this in the MINGW shell
set -x
set -e
mkdir -p com/mobilerobots/Aria
rm -f com/mobilerobots/Aria/*
/c/swig/swig.exe -Fmicrosoft -c++ -package com.mobilerobots.Aria -java -module AriaJava -DWIN32 -DAREXPORT -outdir 'com\mobilerobots\Aria' -o AriaJava_wrap.cpp '-I..\include' '..\include\wrapper.i'
sed 's/protected static long getCPtr/public ArFunctor() { this(0, false); }    public static long getCPtr/' com/mobilerobots/Aria/ArFunctor.java > ArFunctor.java.tmp  && mv ArFunctor.java.tmp com/mobilerobots/Aria/ArFunctor.java
javac -classpath com/mobilerobots/Aria com/mobilerobots/Aria/*.java 
jar cf Aria.jar 'com\mobilerobots\Aria\*.class'

mkdir -p com/mobilerobots/ArNetworking
rm -f com/mobilerobots/ArNetworking/*

/c/swig/swig.exe -Fmicrosoft -c++ -package com.mobilerobots.ArNetworking -java -module ArNetworkingJava -DWIN32 -DAREXPORT -outdir 'com\mobilerobots\ArNetworking' -o ArNetworkingJava_wrap.cpp '-I..\include' '-I../ArNetworking/include'  '..\ArNetworking\include\wrapper.i'

javac -classpath 'Aria.jar;com/mobilerobots/ArNetworking' com/mobilerobots/ArNetworking/*.java 
jar cf ArNetworking.jar 'com\mobilerobots\ArNetworking\*.class'
