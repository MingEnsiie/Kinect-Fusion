#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

// System includes
#include <windows.h>
#include <stdexcept>
#include <iostream>
#include <functional>


#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

#include "vtkImageRender.h"
//#include "vtkTimeCallBack.h"

// For timing calls
#include "Timer.h"

#include "KeyPressInteractorStyle.h"
#include "FusionHelper.h"

using namespace std;

class DepthSensor
{


	static const int			cBytesPerPixel = 4;
	static const int			cResetOnTimeStampSkippedMilliseconds = 1000;
	static const int			cResetOnNumberOfLostFrames = 100;
	static const int			cTimeDisplayInterval = 10;

private:
	

	int							cDepthWidth;
    int							cDepthHeight;
	int							cDepthImagePixels;
	//Kinect 
	INuiSensor*					mNuiSensor;

	//INuiFusionColorReconstruction * m_pVolume;
	INuiFusionReconstruction*	m_pVolume;
	CRITICAL_SECTION            m_lockVolume;



	NUI_IMAGE_RESOLUTION        mdepthImageResolution;
	//VTK image render
	vtkImageRender *			mDrawDepth;


	HANDLE						mNextDepthFrameEvent;
	HANDLE						mDepthStreamHandle;
	
	LARGE_INTEGER				m_cLastDepthFrameTimeStamp;
	//HANDLE			mNextColorFrameEvent;
	//HANDLE			mColorStreamHandle;

	//FUSION parameter

	/// <summary>
	/// Frames from the depth input
	/// </summary>
	NUI_DEPTH_IMAGE_PIXEL*      m_pDepthImagePixelBuffer;
	NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;

	/// Frames generated from ray-casting the Reconstruction Volume
	NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;

	/// Images for display
	NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;
	NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParams;

	/// <summary>
	/// Camera Tracking parameters
	/// </summary>
	int							m_cLostFrameCounter;
	bool						m_bTrackingFailed;


	/// <summary>
	/// Parameter to turn automatic reset of the reconstruction when camera tracking is lost on or off.
	/// Set to true in the constructor to enable auto reset on cResetOnNumberOfLostFrames lost frames,
	/// or set false to never automatically reset.
	/// </summary>
	bool                        m_bAutoResetReconstructionWhenLost;

	/// <summary>
	/// Parameter to enable automatic reset of the reconstruction when there is a large
	/// difference in timestamp between subsequent frames. This should usually be set true as 
	/// default to enable recorded .xed files to generate a reconstruction reset on looping of
	/// the playback or scrubbing, however, for debug purposes, it can be set false to prevent
	/// automatic reset on timeouts.
	/// </summary>
	bool                        m_bAutoResetReconstructionOnTimeout;

	BYTE*						m_depthRGBX;

	Matrix4						m_worldToCameraTransform;

	/// <summary>
	// The default Kinect Fusion World to Volume Transform
	/// </summary>
	Matrix4						m_defaultWorldToVolumeTransform;


	/// <summary>
	/// Processing parameters
	/// </summary>
	unsigned short				m_cMaxIntegrationWeight;
	float                       m_fMinDepthThreshold;
	float                       m_fMaxDepthThreshold;
	bool                        m_bMirrorDepthFrame;
	int                         m_cFrameCounter;
	double                      m_fStartTime;
	Timing::Timer               m_timer;
	bool                        m_isInit;
	KinectFusionMeshTypes       m_saveMeshFormat;


	//file Name
	string						filename;
	


	/// <summary>
	/// Parameter to translate the reconstruction based on the minimum depth setting. When set to
	/// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
	/// Setting this true in the constructor will move the volume forward along +Z away from the
	/// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
	/// by setting a non-identity camera transformation in the ResetReconstruction call.
	/// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
	/// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
	/// when the majority of a small volume is inside this distance.
	/// </summary>
	bool                        m_bTranslateResetPoseByMinDepthThreshold;

	/// <summary>
	/// Copy the extended depth data out of a Kinect image frame
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT						CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame);


	/// <summary>
	/// Reset the reconstruction camera pose and clear the volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                     ResetReconstruction();


	void						initKinectFusion();
	void						createInstance();
    const Matrix4&				IdentityMatrix();

	/// <summary>
	/// Calculate a mesh for the current volume.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT						CalculateMesh(INuiFusionMesh** ppMesh);

	/// <summary>
	/// The reconstruction processor
	/// </summary>
	HRESULT						SaveFile(INuiFusionMesh* pMesh, KinectFusionMeshTypes *saveMeshType);
	
	

public:
	DepthSensor();	
	void init();
	/// <summary>
	/// Main processing function
	/// </summary>
	void Update();
	void processDepth();

	//Creating and saving mesh of reconstruction as .obj file
	bool SaveMesh();
	~DepthSensor();
};


//class vtkLambdaCommand from vtkCallbackCommand
class vtkLambdaCommand : public vtkCallbackCommand
{
public:
	typedef std::function< void(vtkObject *, unsigned long, void *) > Callback;

	vtkTypeMacro(vtkLambdaCommand, vtkCallbackCommand);

	/**
	* @brief returns a new vtkLambdaCommand
	*/
	static vtkLambdaCommand *New()
	{
		return new vtkLambdaCommand;
	}

	/**
	* @brief vtkCommand::Execute implementation
	*/
	virtual void Execute(vtkObject *caller, unsigned long eid, void *callData)
	{
		if (m_callback)
		{
			m_callback(caller, eid, callData);
		}
	}

	/**
	* @brief Sets callback
	*/
	virtual void SetCallback(Callback callback)
	{
		this->m_callback = callback;
	}

protected:

	vtkLambdaCommand()
	{
	}
	~vtkLambdaCommand()
	{
	}

	Callback m_callback;
};




class vtkTimerCallback : public vtkCommand
{
public:
	vtkTimerCallback(DepthSensor* sensor) : m_sensor(sensor)
	{
	}

	virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData))
	{

		if (vtkCommand::TimerEvent == eventId)
		{
			/*std::cout << "timer eventId:" << eventId << std::endl;*/
			m_sensor->processDepth();
		}
	}

private:
	DepthSensor* m_sensor;
};

#endif

