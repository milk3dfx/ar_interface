#include "ros/ros.h"
#include "std_msgs/String.h"
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

void mjpegCallback(const sensor_msgs::Image& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	ROS_INFO("I heard: %d, %d", msg.width, msg.height);
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

int main(int argc, char **argv)
{
	std::cout<<"AR Interface node"<<std::endl;

	ros::init(argc, argv, "ar_interface");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, mjpegCallback);
	ros::Subscriber subWebMouse = n.subscribe("/web_mouse", 100, web_mouseCallback);
	ros::Publisher augmented_pub = n.advertise<sensor_msgs::Image>("augmented_image_raw", 1000);

	Ogre::Root* lRoot;

	std::cout<<" === AR interface ==="<<std::endl;
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
/*
			lPluginNames.push_back("RenderSystem_GL");
			lPluginNames.push_back("Plugin_ParticleFX");
			//lPluginNames.push_back("Plugin_CgProgramManager");
			//lPluginNames.push_back("Plugin_BSPSceneManager");
			lPluginNames.push_back("Plugin_OctreeSceneManager");
*/
		Ogre::RenderWindow* m_pRenderWnd = lRoot->initialise(false);
		Ogre::RenderWindow *window = lRoot->createRenderWindow(
			"AR Interface Test",  // window name
			800,                   // window width, in pixels
			600,                   // window height, in pixels
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
		mNode->setScale(1, 1, 1);
		//Ogre::Entity* planeEnt = Scene->createEntity( "Cube", Ogre::SceneManager::PT_CUBE );
		Ogre::Entity* planeEnt = Scene->createEntity("mySphere", Ogre::SceneManager::PT_SPHERE);
		///planeEnt->setMaterialName("Examples/BumpyMetal");
		mNode->attachObject(planeEnt);

		//Ogre::Entity* ogreEntity = Scene->createEntity("ogrehead.mesh");

		Ogre::Light* light = Scene->createLight("MainLight");
		light->setPosition(10, 40, 20);

		Ogre::Viewport* vp = window->addViewport(Camera);
		vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
		Camera->setAspectRatio( Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));

		Ogre::TexturePtr rttTexture = Ogre::TextureManager::getSingleton().createManual(
			"RttTex", 
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, 
			Ogre::TEX_TYPE_2D, 
			window->getWidth(), window->getHeight(), 
			0, 
			Ogre::PF_R8G8B8, 
			Ogre::TU_RENDERTARGET);
		Ogre::RenderTexture* mRT = rttTexture->getBuffer()->getRenderTarget();
		//           HardwarePixelBufferSharedPtr---^
		mRT->removeAllViewports();
    		mRT->addViewport(Camera);
 
    		//set the viewport settings
    		Ogre::Viewport *vp_s = mRT->getViewport(0);
    		vp_s->setClearEveryFrame(true);    
    		vp_s->setOverlaysEnabled(false);
 
    		// remind current overlay flag
    		//bool enableOverlayFlag = Ogre::Root::getSingletonPtr()->getDefaultViewport()->getOverlaysEnabled();
 
    		// we disable overlay rendering if it is set in config file and the viewport setting is enabled
    		//if(mDisableOverlays && enableOverlayFlag)
        	//	Ogre::Root::getSingletonPtr()->getDefaultViewport()->setOverlaysEnabled(false);
 
   
        	// Simple case where the contents of the screen are taken directly
        	// Also used when an invalid value is passed within gridSize (zero or negative grid size)
        	mRT->update();        //render
 
        	//write the file on the Harddisk
        	mRT->writeContentsToFile("image1.jpg");


 
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
