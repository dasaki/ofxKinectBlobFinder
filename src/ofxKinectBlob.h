/****************************************************************************
*
* ofxKinectBlob.h
* openFrameworks
*
* A blob is a set of 3D points that belong to the same continuous surface
*
* Purpose: 3D blob to be used by ofxKinectBlobFinder
*
* Author: David Sanz Kirbis, January 2012
*
* Comments: part of the cvCinema project
*
******************************************************************************/
#pragma once

#include "ofMain.h"

class ofxKinectBlob:public ofNode {
    public:
        virtual void customDraw() {
            mesh.drawVertices();
            ofTranslate(massCenter);//centroid);
            ofDrawAxis(0.5f);
            ofScale(maxX.x - minX.x, maxY.y - minY.y, maxZ.z - minZ.z);
            //ofBox(1);
            // if (bDrawAxis)
            // if (bDrawBox) drawGlWireBox();
        }
      ofMesh mesh;
      ofVec3f centroid;
      ofVec3f minX, minY, minZ; // points with minimum x / y / z
      ofVec3f maxX, maxY, maxZ; // points with maximum x / y / z
      ofVec3f boundingBoxMax, boundingBoxMin; // min bounding xyz
      ofVec3f dimensions; //dimensions
      ofVec3f massCenter;
      float volume; // volume
};

