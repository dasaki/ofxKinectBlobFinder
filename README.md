ofxKinectBlobFinder
===================

3D blob finder to be used with ofxKinect. Avoids occlusion issues when using a 2D contourfinder

See github.com/dasaki/ofxKinectBlobTracker for a working example code::blocks project/code.

Watch it in action here: https://www.youtube.com/watch?v=uwvYgiwrTwU

Comments:

Part of the cvCinema project: www.cvcinema.com

The technique used is Euclidean Cluster Extraction, as explained in:
http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php

To imporve performance the neighbor points are searched in a box (X/Y/Z distances) instead of a
sphere (radius/distance). As the kinect cloud is ordered, the adjacency is first checked in the XY plane,
within a specified pixel area around each point.

See example of usage in the ofxKinectBlobTracker addon.

The finder function is called findBlobs, and it's parameters are:

ofImage * maskImage
a binary grayscale image where black pixels indicates non wanted / background pixels. 

const ofVec3f boundingBoxMin
first/minimum vertex defining the crop box that delimites the space where points shoud be to be considered in the
search for blobs.

const ofVec3f boundingBoxMax,
secnd/maximum vertex defining the crop box that delimites the space where points shoud be to be considered in the
search for blobs.

const ofVec3f thresh3D
X/Y/Z threshold ranges for two points to be considered as adjacent

const int thresh2D
xy threshold ranges for two pixels to be considered as adjacent

const float minVol
minimum volume that a blob shold have to be included in the result

const float maxVol
maximum volume that a blob shold have to be included in the result

const int minPoints
minimum number of volume that a blob shold have to be included in the result

const unsigned int maxBlobs
maximum number of blobs to return as result
