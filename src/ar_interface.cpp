#include <stdio.h>
#include <string.h>
#include <sstream>

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Image.h"
#include "ar_interface/mouse.h"
#include "rail_manipulation_msgs/SegmentedObjectList.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreEntity.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>
#include <OgreTexture.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMeshManager.h>
#include <OgreMatrix4.h>
#include <OgreVector3.h>
#include <OgreMatrix3.h>
#include <OgreMath.h>
#include <OgreQuaternion.h>
#include <OgreSceneQuery.h>
#include <OgreRay.h>
#include <OgreMovableObject.h>

using namespace std;


// OGRE Root object
Ogre::Root* lRoot;

// ROS messages
sensor_msgs::Image ar_image_msg;
sensor_msgs::Image image_msg;
ar_interface::mouse mouse_msg;
rail_manipulation_msgs::SegmentedObjectList recognized_objects;




// Debug varibles
bool MOUSE_DEBUG = false;
bool IMAGES_DEBUG = false;
bool TF_DEBUG = false;

// Segmentation
// if this flag is true, then node will call segmentation topic
bool fSegmentation = true;


// Flow control varibles
bool fUpdateObjects = false;
bool fUpdateMouse = false;


Ogre::MovableObject* getClickedNode(Ogre::SceneManager* ogreSystem , float mouseScreenX, float mouseScreenY){
	Ogre::Ray mouseRay = ogreSystem->getCamera("ARCamera")->getCameraToViewportRay(mouseScreenX/640, mouseScreenY/480);
	Ogre::RaySceneQuery *mRaySceneQuery = ogreSystem->createRayQuery(Ogre::Ray(), Ogre::SceneManager::ENTITY_TYPE_MASK);
	mRaySceneQuery->setRay(mouseRay);
	mRaySceneQuery->setSortByDistance(true);
	Ogre::RaySceneQueryResult &result = mRaySceneQuery->execute(); // result is vector
	Ogre::MovableObject *closestObject = NULL;
	Ogre::Real closestDistance = 100000;

	Ogre::RaySceneQueryResult::iterator rayIterator;

	for(rayIterator = result.begin(); rayIterator != result.end(); rayIterator++ ){
		if ((*rayIterator).movable !=NULL &&
			closestDistance>(*rayIterator).distance &&
			(*rayIterator).movable->getMovableType() != "TerrainMipMap"){

			closestObject = ( *rayIterator ).movable;
			closestDistance = ( *rayIterator ).distance;
			Ogre::Vector3 oldpos = mouseRay.getPoint((*rayIterator).distance);
			Ogre::Vector3 originalPos = oldpos;

		}
	}
	mRaySceneQuery->clearResults();
	return closestObject;
}



// ========== Callback functions ==========
void mjpegCallback(const sensor_msgs::Image& msg)
{
	if(IMAGES_DEBUG){
		cout << ">> MESSAGE FROM: mjpeg" << endl
			 << "Width: " << msg.width << endl
			 << "Height: " << msg.height << endl
			 << "Encoding: " << msg.encoding << endl
			 << "is_bigendian: " << msg.is_bigendian << endl
			 << "Step: " << msg.step << endl
			 << "=================================" << endl;
	}
	image_msg = msg;
}

void web_mouseCallback(const ar_interface::mouse& msg)
{

	if(MOUSE_DEBUG){
		ROS_INFO("On click: %d, %d", msg.x, msg.y);
	}
	mouse_msg = msg;
	fUpdateMouse = true;
}

void recognized_objectsCallback(const rail_manipulation_msgs::SegmentedObjectList& msg)
{
	cout << ">> MESSAGE FROM: recognized objects" << endl
		 << "Number of objects: " << msg.objects.size() << endl;

	for(int i = 0; i < msg.objects.size(); i++){
		geometry_msgs::Point centroid =  msg.objects.at(i).centroid;
		cout << "----------------------------------------" << endl
			 << "Name: " << msg.objects.at(i).name << endl
			 << "Confidence: " << msg.objects.at(i).confidence << endl
			 << "Centroid: " << centroid.x << ", " << centroid.y << ", " << centroid.z << endl;
	}
	if(msg.objects.size() != 0){
		recognized_objects = msg;
		fUpdateObjects = true;
	}
}

// OGRE Render one frame function
bool Ogre::Root::renderOneFrame(void)
{
	if(!_fireFrameStarted())
		return false;
	if(!_updateAllRenderTargets())
		return false;
	return _fireFrameEnded();
}

// ========== Main function ==========
int main(int argc, char **argv)
{
	std::cout<<"AR Interface node"<<std::endl;

	ros::init(argc, argv, "ar_interface");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 100, mjpegCallback);
	ros::Subscriber subWebMouse = n.subscribe("/web_mouse", 100, web_mouseCallback);
	ros::Subscriber subRecognizedObjects = n.subscribe("/object_recognition_listener/recognized_objects",
														10, recognized_objectsCallback);

	ros::Publisher pubAugmentedImage = n.advertise<sensor_msgs::Image>("augmented_image_raw", 100);


	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	// Segmentation service
	ros::ServiceClient clientSegment = n.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
	std_srvs::Empty request;
	if(fSegmentation){
		clientSegment.call(request);
	}

	ros::Rate loop_rate(30);


	try{
		// OGRE root
		Ogre::String lConfigFileName = "";
		Ogre::String lPluginsFileName = "";
		Ogre::String lLogFileName = "ar_interface_ogre.LOG";
		lRoot = new Ogre::Root(lConfigFileName, lPluginsFileName, lLogFileName);

		// Load OGRE plagins
		lRoot->loadPlugin("Plugin_OctreeSceneManager");
		lRoot->loadPlugin("RenderSystem_GL");
		lRoot->loadPlugin("Plugin_ParticleFX");


		// Create render system and window
		const Ogre::RenderSystemList& lRenderSystemList = lRoot->getAvailableRenderers();
		if( lRenderSystemList.size() == 0 ){
			Ogre::LogManager::getSingleton().logMessage("Sorry, no rendersystem was found.");
			return 1;
		}
 
		Ogre::RenderSystem *lRenderSystem = lRenderSystemList[0];
		lRoot->setRenderSystem(lRenderSystem);
		Ogre::RenderWindow* m_pRenderWnd = lRoot->initialise(false);
		Ogre::RenderWindow *window = lRoot->createRenderWindow(
			"AR Interface Test",  // window name
			640,                   // window width, in pixels
			480,                   // window height, in pixels
			false,                 // fullscreen or not
			0);

		// Create scene and roof scene node
		Ogre::SceneManager* Scene = lRoot->createSceneManager(Ogre::ST_GENERIC, "MyFirstSceneManager");
		Ogre::SceneNode* lRootSceneNode = Scene->getRootSceneNode();
		
		// Create camera
		Ogre::Camera* Camera = Scene->createCamera("ARCamera");
		Camera->setNearClipDistance(1.0f);
		Camera->setFarClipDistance(1000.0f);

		// Create plane object, which will use like label
		Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
		Ogre::MeshManager::getSingleton().createPlane(
			"label",
		 	Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			plane, 
			1, 1, // Size
			1, 1, // Segments
			true, 
			1, 5, 5, 
			Ogre::Vector3::UNIT_Z
		);



		// Light
		Ogre::Light* light = Scene->createLight("MainLight");
		light->setPosition(10, 40, 20);

		// View point for window
		Ogre::Viewport* vp = window->addViewport(Camera);
		vp->setBackgroundColour(Ogre::ColourValue(0,0,0,0));
		Camera->setAspectRatio( Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

		// Create texture for randering
		Ogre::TexturePtr rttTexture = Ogre::TextureManager::getSingleton().createManual(
			"RttTex", 
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
			Ogre::TEX_TYPE_2D, 
			window->getWidth(), window->getHeight(), 
			0, 
			Ogre::PF_R8G8B8A8, 
			Ogre::TU_RENDERTARGET);
		Ogre::RenderTexture* mRT = rttTexture->getBuffer()->getRenderTarget();
		mRT->removeAllViewports();
    	mRT->addViewport(Camera);
 
    	//Wiew point for rander texture
    	Ogre::Viewport *vp_s = mRT->getViewport(0);
    	vp_s->setClearEveryFrame(true);    
    	vp_s->setOverlaysEnabled(false);
    	vp_s->setBackgroundColour(Ogre::ColourValue(0,0,0,0));
        mRT->update();


		int count = 0;
		while (ros::ok()){
			try{
				geometry_msgs::TransformStamped transformStamped;

				transformStamped = tfBuffer.lookupTransform("base_footprint", "camera_rgb_frame", 
					image_msg.header.stamp);

				geometry_msgs::Vector3 cTranslation = transformStamped.transform.translation;
				geometry_msgs::Quaternion cRotation = transformStamped.transform.rotation;

				if(TF_DEBUG){
					cout<< "CAMERA tf " << endl
						<< "Translation: " << cTranslation.x <<", "
										   << cTranslation.y <<", "
										   << cTranslation.z << endl
						<< "Rotation: " << "x: " << cRotation.x <<", "
										<< "y: " << cRotation.y <<", "
										<< "z: " << cRotation.z <<", "
										<< "w: " << cRotation.w << endl;
				}
				Ogre::Quaternion cOrientation(cRotation.w, cRotation.x, cRotation.y, cRotation.z);

				cOrientation = 
					cOrientation
					*Ogre::Quaternion(0.70710678118,0,-0.70710678118,0)
					*Ogre::Quaternion(0.70710678118,0,0,-0.70710678118);

				Ogre::Vector3 cPosition(cTranslation.x, cTranslation.y, cTranslation.z);
				Camera->setOrientation(cOrientation);
				Camera->setPosition(cPosition);
			}
			catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				ros::spinOnce();
				loop_rate.sleep();
			    ++count;
				continue;
			}

			// Create Scene
			if(fUpdateObjects = true){
				for(int i = 0; i < recognized_objects.objects.size(); i++){
					geometry_msgs::Point centroid =  recognized_objects.objects.at(i).centroid;

					Ogre::SceneNode* mNode = lRootSceneNode->createChildSceneNode();
					mNode->setPosition(centroid.x, centroid.y, centroid.z + 0.15);
					mNode->setScale(0.1, 0.1, 0.1);
					Ogre::Matrix3 rMx;
					rMx.FromAngleAxis(Ogre::Vector3(0, 0, 1), Ogre::Radian(Ogre::Degree(90)));
					Ogre::Quaternion rQx(rMx);
					mNode->setOrientation(rMx);
					Ogre::Entity* labelEntity = Scene->createEntity("label");
					mNode->attachObject(labelEntity);
				}
				fUpdateObjects = false;
			}

			// Mouse event
			if(fUpdateMouse == true){
				fUpdateMouse = false;
				Ogre::MovableObject* clickedObject = getClickedNode(Scene, (float)mouse_msg.x, (float)mouse_msg.y);
				if(clickedObject != NULL){
					Ogre::SceneNode *node = clickedObject->getParentSceneNode();
					// Set new scale value of clicked objet
					node->setScale(0.2, 0.2, 0.2);
					if(MOUSE_DEBUG){
						cout << "MOUSE: Node selected" << endl;
					}
				}
			}

	        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = rttTexture->getBuffer();
	        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
			const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
			Ogre::uint8* pDest = static_cast< Ogre::uint8* >(pixelBox.data);

			// CREATING image message
			ar_image_msg.width = window->getWidth();
			ar_image_msg.height = window->getHeight();
			ar_image_msg.encoding = "rgba8";
			ar_image_msg.step = 4*window->getWidth();


			ar_image_msg.data.clear();
			for (size_t j = 0; j < window->getWidth(); j++)
			{
			    for(size_t i = 0; i < window->getHeight(); i++)
			    {
			    	ar_image_msg.data.push_back(*pDest++);
			    	ar_image_msg.data.push_back(*pDest++);
			    	ar_image_msg.data.push_back(*pDest++);
			    	ar_image_msg.data.push_back(*pDest++);
			    }
			    pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
			}

			// Unlock the pixel buffer
			pixelBuffer->unlock();

			// Copy AR image over real image
			for(int i=0; i < image_msg.data.size()/3; i++){
				if(ar_image_msg.data.at(4*i + 3) != 0 ){
					image_msg.data.at(3*i) = ar_image_msg.data.at(4*i);
					image_msg.data.at(3*i + 1) = ar_image_msg.data.at(4*i+1);
					image_msg.data.at(3*i + 2) = ar_image_msg.data.at(4*i+2);
				}
			}

			pubAugmentedImage.publish(image_msg);

			ros::spinOnce();
			loop_rate.sleep();
			++count;

			Ogre::WindowEventUtilities::messagePump();
			if(window->isClosed()) return false;
			if(!lRoot->renderOneFrame()) return false;

		}

	}
	catch(Ogre::Exception &e){
		std::cout<<"!!!!Ogre::Exception!!!!"<<e.what()<<std::endl;
	}
	catch(std::exception &e){
		std::cout<<"!!!!std::exception!!!!"<<e.what()<<std::endl;
	}


	ros::spin();
	return 0;
}
