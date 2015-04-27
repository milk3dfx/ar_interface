#include <stdio.h>
#include <string.h>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "sensor_msgs/Image.h"
#include "ar_interface/mouse.h"

/*
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreEntity.h"
#include "OGRE/OgreViewport.h"
*/

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

#define DEBUG_MSGS

Ogre::Root* lRoot;
sensor_msgs::Image ar_image_msg;
sensor_msgs::Image image_msg;

using namespace std;

// Callback functions
void mjpegCallback(const sensor_msgs::Image& msg)
{
	cout << ">> IMAGE MSG" << endl
		 << "Width: " << msg.width << endl
		 << "Height: " << msg.height << endl
		 << "Encoding: " << msg.encoding << endl
		 << "is_bigendian: " << msg.is_bigendian << endl
		 << "Step: " << msg.step << endl
		 << "=================================" << endl;

		image_msg = msg;
}

void web_mouseCallback(const ar_interface::mouse& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	ROS_INFO("On click: %d, %d", msg.x, msg.y);
}

bool Ogre::Root::renderOneFrame(void){
	if(!_fireFrameStarted())
		return false;
	if(!_updateAllRenderTargets())
		return false;
	return _fireFrameEnded();
}

// Main function
int main(int argc, char **argv)
{
	std::cout<<"AR Interface node"<<std::endl;

	ros::init(argc, argv, "ar_interface");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, mjpegCallback);
	ros::Subscriber subWebMouse = n.subscribe("/web_mouse", 100, web_mouseCallback);
	ros::Publisher pubAugmentedImage = n.advertise<sensor_msgs::Image>("augmented_image_raw", 100);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	try{
		Ogre::String lConfigFileName = "";
		Ogre::String lPluginsFileName = "";
		Ogre::String lLogFileName = "ar_interface_ogre.LOG";
		lRoot = new Ogre::Root(lConfigFileName, lPluginsFileName, lLogFileName);
		//if(!lRoot->showConfigDialog())
		//	return false;

		lRoot->loadPlugin("Plugin_OctreeSceneManager");
		lRoot->loadPlugin("RenderSystem_GL");
		//lRoot->loadPlugin("Plugin_BSPSceneManager");        
    		//
		lRoot->loadPlugin("Plugin_ParticleFX");

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


		Ogre::SceneManager* Scene = lRoot->createSceneManager(Ogre::ST_GENERIC, "MyFirstSceneManager");
		Ogre::SceneNode* lRootSceneNode = Scene->getRootSceneNode();
		
		std::cout<<"CAMERA:"<<std::endl;
		Ogre::Camera* Camera = Scene->createCamera("ARCamera");
		Camera->setNearClipDistance(1.0f);
		Camera->setFarClipDistance(1000.0f);
		//Camera->setFOVy(45.0f);
		Camera->setPosition(0,0,120);
		//Camera->lookAt(Ogre::Vector3(0, 0, 0));
		


		//Ogre::SceneNode* nCamera = lRootSceneNode->createChildSceneNode();
		//nCamera->setPosition(0,0,5);
		//nCamera->attachObject(Camera);

		Ogre::SceneNode* mNode = lRootSceneNode->createChildSceneNode();
		mNode->setPosition(0,0,0);
		mNode->setScale(0.1, 0.1, 0.1);
		//Ogre::Entity* planeEnt = Scene->createEntity( "Cube", Ogre::SceneManager::PT_CUBE );
		Ogre::Entity* planeEnt = Scene->createEntity("mySphere", Ogre::SceneManager::PT_PLANE);
		///planeEnt->setMaterialName("Examples/BumpyMetal");
		mNode->attachObject(planeEnt);


		Ogre::Light* light = Scene->createLight("MainLight");
		light->setPosition(10, 40, 20);

		Ogre::Viewport* vp = window->addViewport(Camera);
		vp->setBackgroundColour(Ogre::ColourValue(0,0,0,0));
		Camera->setAspectRatio( Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

		Ogre::TexturePtr rttTexture = Ogre::TextureManager::getSingleton().createManual(
			"RttTex", 
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
			Ogre::TEX_TYPE_2D, 
			window->getWidth(), window->getHeight(), 
			0, 
			Ogre::PF_R8G8B8A8, 
			Ogre::TU_RENDERTARGET);
		Ogre::RenderTexture* mRT = rttTexture->getBuffer()->getRenderTarget();
		//           HardwarePixelBufferSharedPtr---^
		mRT->removeAllViewports();
    	mRT->addViewport(Camera);
 
    	//set the viewport settings
    	Ogre::Viewport *vp_s = mRT->getViewport(0);
    	vp_s->setClearEveryFrame(true);    
    	vp_s->setOverlaysEnabled(false);
    	vp_s->setBackgroundColour(Ogre::ColourValue(0,0,0,0));
        mRT->update();        //render

        // Write image to file
        // mRT->writeContentsToFile("image1.jpg");


        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = rttTexture->getBuffer();
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
		const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
 
		Ogre::uint8* pDest = static_cast< Ogre::uint8* >(pixelBox.data);



			std::cout<<"CREATING MSG"<<std::endl;
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
			    	//ar_image_msg.data.push_back(*pDest++);
			    	cout << (int)*pDest++ << " ";
			    }
			    pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
			}

			// Unlock the pixel buffer
			pixelBuffer->unlock();
			std::cout<<"TOPIC"<<std::endl;

			int count = 0;
			while (ros::ok()){

				for(int i=0; i < image_msg.data.size()/3; i++){
					if(ar_image_msg.data.at(4*i + 3) != 0 ){
						image_msg.data.at(3*i) = ar_image_msg.data.at(4*i);
						image_msg.data.at(3*i + 1) = ar_image_msg.data.at(4*i+1);
						image_msg.data.at(3*i + 2) = ar_image_msg.data.at(4*i+2);
					}
				}

				pubAugmentedImage.publish(image_msg);
				cout<< ">> ar_image_msg published" << count <<std::endl;


				ros::spinOnce();
				loop_rate.sleep();
				++count;
		   	}


 
		//renderTexture->addViewport(camera);
		//renderTexture->getViewport(0)->setClearEveryFrame(true);
		//renderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
		//renderTexture->getViewport(0)->setOverlaysEnabled(false);
		
		//renderTexture->update();
		//renderTexture->writeContentsToFile("start.png");

		//Ogre::Image finalImage;



		while(true){
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
