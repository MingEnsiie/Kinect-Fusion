
#include "DepthSensor.h"
#include <functional>


// add  Properties->Debugging ->environment  PATH = %PATH%; D:\VTK_bin\bin\Debug
DepthSensor::DepthSensor()
    : mNuiSensor(NULL)
    , mNextDepthFrameEvent(INVALID_HANDLE_VALUE)
    , mDepthStreamHandle(INVALID_HANDLE_VALUE)
    , mDrawDepth(NULL)
    , mdepthImageResolution(NUI_IMAGE_RESOLUTION_640x480)
    , m_pVolume(NULL)
    , cDepthImagePixels(0)
    , m_pDepthImagePixelBuffer(NULL)
    , m_pDepthFloatImage(NULL)
    , m_pPointCloud(NULL)
    , m_pShadedSurface(NULL)
    , m_bMirrorDepthFrame(false)
    , m_bTranslateResetPoseByMinDepthThreshold(true)
    , m_cLostFrameCounter(0)
    , m_bTrackingFailed(false)
    , m_bAutoResetReconstructionWhenLost(false)
    , m_bAutoResetReconstructionOnTimeout(true)
    , m_fStartTime(0)
    , m_isInit(false)
    , m_saveMeshFormat(Stl)
    , filename()
{
    // Get the depth frame size from the NUI_IMAGE_RESOLUTION enum
    DWORD WIDTH = 0, HEIGHT = 0;
    NuiImageResolutionToSize(mdepthImageResolution, WIDTH, HEIGHT);
    cDepthWidth = WIDTH;
    cDepthHeight = HEIGHT;
    cDepthImagePixels = cDepthWidth*cDepthHeight;

    //create heap storage for depth pixel data in RGBX format
    m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];

    // Define a cubic Kinect Fusion reconstruction volume,
    // with the Kinect at the center of the front face and the volume directly in front of Kinect.
    reconstructionParams.voxelsPerMeter = 256;	// 1000mm / 256vpm = ~3.9mm/voxel
    reconstructionParams.voxelCountX = 512;		// 512 / 256vpm = 2m wide reconstruction
    reconstructionParams.voxelCountY = 384;		// Memory = 512*384*512 * 4bytes per voxel
    reconstructionParams.voxelCountZ = 512;		// Require 512MB GPU memory

    // These parameters are for optionally clipping the input depth image 
    m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
    m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters

    // This parameter is the temporal averaging parameter for depth integration into the reconstruction
    m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes   

    SetIdentityMatrix(m_worldToCameraTransform);
    SetIdentityMatrix(m_defaultWorldToVolumeTransform);

    m_cLastDepthFrameTimeStamp.QuadPart = 0;

    // Initialize synchronization objects
    InitializeCriticalSection(&m_lockVolume);
}



void DepthSensor::createInstance()
{
    int count = 0;
    NuiGetSensorCount(&count);
    if (count == 0)
    {
        throw std::runtime_error("no valid Kinect is connected");
    }
    // creat kinect
    NuiCreateSensorByIndex(0, &mNuiSensor);
    // get kinect status
    HRESULT status = mNuiSensor->NuiStatus();
    if (status != S_OK)
    {
        throw std::runtime_error("Kinect is not ready to work");
    }

    // Create and initialize a new vtk image renderer 
    // We'll use this to draw the data we receive from the Kinect to the screen
    mDrawDepth = new vtkImageRender();
    
    if (!mDrawDepth->Initialize(cDepthWidth, cDepthHeight, cDepthWidth * sizeof(long)))
    {
        throw std::runtime_error("Failed to initialize the vtk draw device.");
    }
}

void DepthSensor::init()
{
    HRESULT hr;
    createInstance();

    hr=mNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
    //NUI_INITIALIZE_FLAG_USES_COLOR |
    if (SUCCEEDED(hr))
    {
        // Create an event that will be signaled when depth data  is available
        mNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        // Open a depth image stream to receive depth frames
        hr = mNuiSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_DEPTH,
            NUI_IMAGE_RESOLUTION_640x480,
            0,
            2, 
            mNextDepthFrameEvent,
            &mDepthStreamHandle);
        if (FAILED(hr))
        {
            throw std::runtime_error("NuiImageStreamOpen failed");
        }
    }
    else 
    {
        throw std::runtime_error("Initial failed");
    }
    std::cout << "Intial Kinect Finish" << std::endl;

    //Initialize Kinect Fusion 
    initKinectFusion();
}


//Initialize Kinect Fusion volume and images for processing
void DepthSensor::initKinectFusion()
{
    HRESULT hr = S_OK;

    
    // Create the Kinect Fusion Reconstruction Volume
    hr = NuiFusionCreateReconstruction(
        &reconstructionParams,
        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP,
        -1, 
        &m_worldToCameraTransform,
        &m_pVolume);
    if (FAILED(hr))
    {
        throw std::runtime_error("NuiFusionCreateReconstruction failed.");
    }



    // Save the default world to volume transformation to be optionally used in ResetReconstruction
    hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
    if (FAILED(hr))
    {
        throw std::runtime_error("Failed in call to GetCurrentWorldToVolumeTransform.");
    }



    if (m_bTranslateResetPoseByMinDepthThreshold)
    {
        // This call will set the world-volume transformation
        hr = ResetReconstruction();
        if (FAILED(hr))
        {
            return ;
        }
    }



    // DepthFloatImage  Frames generated from the depth input
    hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, cDepthWidth, cDepthHeight, nullptr, &m_pDepthFloatImage);
    if (FAILED(hr))
        throw std::runtime_error("NuiFusionCreateImageFrame failed (Float).");

    // PointCloud   Create images to raycast the Reconstruction Volume
    hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, cDepthWidth, cDepthHeight, nullptr, &m_pPointCloud);
    if (FAILED(hr))
        throw std::runtime_error("NuiFusionCreateImageFrame failed (PointCloud).");

    // ShadedSurface  Create images to raycast the Reconstruction Volume(color)
    hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, cDepthWidth, cDepthHeight, nullptr, &m_pShadedSurface);
    if (FAILED(hr))
        throw std::runtime_error("NuiFusionCreateImageFrame failed (Color).");

    m_pDepthImagePixelBuffer = new(std::nothrow) NUI_DEPTH_IMAGE_PIXEL[cDepthImagePixels];


    /////////////////////
    if (nullptr == m_pDepthImagePixelBuffer)
    {
        throw std::runtime_error("Failed to initialize Kinect Fusion depth image pixel buffer.");
    }

    m_fStartTime = m_timer.AbsoluteTime();

    std::cout << "Intial Finish"<<std::endl;
}


void DepthSensor::processDepth()
{    
    HRESULT hr;	
    NUI_IMAGE_FRAME imageFrame;
    //get the depth frame 
    
    hr = mNuiSensor->NuiImageStreamGetNextFrame(mDepthStreamHandle, 500, &imageFrame);
    if (FAILED(hr))
    {
        throw std::runtime_error("NuiImageStreamGetNextFrame failed");
    }

    hr = CopyExtendedDepth(imageFrame);


    LARGE_INTEGER currentDepthFrameTime = imageFrame.liTimeStamp;

    // Release the Kinect camera frame
    mNuiSensor->NuiImageStreamReleaseFrame(mDepthStreamHandle, &imageFrame);
    

    // To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
    // if the .xed loops, we test for when the frame timestamp has skipped a large number. 
    // Note: this will potentially continually reset live reconstructions on slow machines which
    // cannot process a live frame in less time than the reset threshold. Increase the number of
    // milliseconds in cResetOnTimeStampSkippedMilliseconds if this is a problem.
    if (m_bAutoResetReconstructionOnTimeout && m_cFrameCounter != 0
        && abs(currentDepthFrameTime.QuadPart - m_cLastDepthFrameTimeStamp.QuadPart) > cResetOnTimeStampSkippedMilliseconds)
    {
        ResetReconstruction();

        if (FAILED(hr))
        {
            return;
        }
    }

    m_cLastDepthFrameTimeStamp = currentDepthFrameTime;

    // Return if the volume is not initialized
    if (nullptr == m_pVolume)
    {
        throw std::runtime_error("Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting.");
        return;
    }


    ////////////////////////////////////////////////////////
    // Depth to DepthFloat

    // Convert the pixels describing extended depth as unsigned short type in millimeters to depth
    // as floating point type in meters.
    hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);
    if (FAILED(hr))
    {
        throw std::runtime_error("Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed.");
        return;
    }


    ////////////////////////////////////////////////////////
    // ProcessFrame

    // Perform the camera tracking and update the Kinect Fusion Volume
    // This will create memory on the GPU, upload the image, run camera tracking and integrate the
    // data into the Reconstruction Volume if successful. Note that passing nullptr as the final 
    // parameter will use and update the internal camera pose.
    hr = m_pVolume->ProcessFrame(m_pDepthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, &m_worldToCameraTransform);
    if (SUCCEEDED(hr))
    {
        Matrix4 calculatedCameraPose;
        hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);

        if (SUCCEEDED(hr))
        {
            // Set the pose
            m_worldToCameraTransform = calculatedCameraPose;
            m_cLostFrameCounter = 0;
            m_bTrackingFailed = false;
        }
    }
    else
    {
        if (hr == E_NUI_FUSION_TRACKING_ERROR)
        {
            m_cLostFrameCounter++;
            m_bTrackingFailed = true;
            std::cout << "Kinect Fusion camera tracking failed! Align the camera to the last tracked position. " << std::endl;
        }
        else
        {
            throw std::runtime_error("Kinect Fusion ProcessFrame call failed!");
            return;
        }
    }



    if (m_bAutoResetReconstructionWhenLost && m_bTrackingFailed && m_cLostFrameCounter >= cResetOnNumberOfLostFrames)
    {
        // Automatically clear volume and reset tracking if tracking fails
        hr = ResetReconstruction();

        if (FAILED(hr))
        {
            return;
        }

        // Set bad tracking message
        std::cout << "Kinect Fusion camera tracking failed, automatically reset volume" << std::endl; 
    
    }


    ////////////////////////////////////////////////////////
    // CalculatePointCloud
    // Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
    hr = m_pVolume->CalculatePointCloud(m_pPointCloud, &m_worldToCameraTransform);

    if (FAILED(hr))
    {
        throw std::runtime_error("Kinect Fusion CalculatePointCloud call failed.");
        return;
    }


    ////////////////////////////////////////////////////////
    // ShadePointCloud and render

    hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, nullptr, m_pShadedSurface, nullptr);

    if (FAILED(hr))
    {
        throw std::runtime_error("Kinect Fusion NuiFusionShadePointCloud call failed.");
        return;
    }

    // Draw the shaded raycast volume image
    INuiFrameTexture * pShadedImageTexture = m_pShadedSurface->pFrameTexture;
    NUI_LOCKED_RECT ShadedLockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    hr = pShadedImageTexture->LockRect(0, &ShadedLockedRect, nullptr, 0);
    if (FAILED(hr))
    {
        return;
    }


    // Make sure we've received valid data
    if (ShadedLockedRect.Pitch != 0)
    {
        BYTE * pBuffer = (BYTE *)ShadedLockedRect.pBits;

        // Draw the data with vtk
        mDrawDepth->Draw(pBuffer, cDepthWidth , cDepthHeight , cBytesPerPixel);
        if (!m_isInit)
        {
            mDrawDepth->Actor->GetMapper()->SetInputData(mDrawDepth->image);
            mDrawDepth->renderer->AddActor(mDrawDepth->Actor);
            m_isInit = true;
        }
        mDrawDepth->renWin->Render();
    }

    // We're done with the texture so unlock it
    pShadedImageTexture->UnlockRect(0);


    //////////////////////////////////////////////////////////
    //// Periodically Display Fps
    //// Update frame counter
    //m_cFrameCounter++;
    //// Display fps count approximately every cTimeDisplayInterval seconds
    //double elapsed = m_timer.AbsoluteTime() - m_fStartTime;
    //if ((int)elapsed >= cTimeDisplayInterval)
    //{
    //	double fps = (double)m_cFrameCounter / elapsed;
    //	// Update status display
    //	if (!m_bTrackingFailed)
    //	{
    //		WCHAR str[MAX_PATH];
    //		swprintf_s(str, ARRAYSIZE(str), L"Fps: %5.2f", fps);
    //		
    //		cout<<str<<endl;
    //	}
    //	m_cFrameCounter = 0;
    //	m_fStartTime = m_timer.AbsoluteTime();
    //}
}


HRESULT DepthSensor::ResetReconstruction()
{
    if (nullptr == m_pVolume)
    {
        return E_FAIL;
    }

    HRESULT hr = S_OK;
    SetIdentityMatrix(m_worldToCameraTransform);
    // Translate the reconstruction volume location away from the world origin by an amount equal
    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
    // If set false, the default world origin is set to the center of the front face of the 
    // volume, which has the effect of locating the volume directly in front of the initial camera
    // positionto the with the +Z axis in volume along the initial camera direction of view.
    if (m_bTranslateResetPoseByMinDepthThreshold)
    {
        Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

        // Translate the volume in the Z axis by the minDepthThreshold distance
        float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
        worldToVolumeTransform.M43 -= (minDist * reconstructionParams.voxelsPerMeter);

        hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
    }
    else
    {
        hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
    }

    m_cLostFrameCounter = 0;
    m_cFrameCounter = 0;
    m_fStartTime = m_timer.AbsoluteTime();

    if (SUCCEEDED(hr))
    {
        m_bTrackingFailed = false;

        cout << "Reconstruction has been reset.\n" << endl;
        }
    else
    {
        throw std::runtime_error("Failed to reset reconstruction.");
    }
    return hr;
}


HRESULT DepthSensor::CopyExtendedDepth(NUI_IMAGE_FRAME &imageFrame)
{
    HRESULT hr = S_OK;

    if (nullptr == m_pDepthImagePixelBuffer)
    {
        throw std::runtime_error("Error depth image pixel buffer is nullptr.");
        return E_FAIL;
    }

    INuiFrameTexture *extendedDepthTex = nullptr;
    // Extract the extended depth in NUI_DEPTH_IMAGE_PIXEL format from the frame
    BOOL nearModeOperational = FALSE;

    hr = mNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(mDepthStreamHandle, &imageFrame, &nearModeOperational, &extendedDepthTex);
    if (FAILED(hr))
    {
        throw std::runtime_error("Error getting extended depth texture.");
        return hr;
    }


    NUI_LOCKED_RECT extendedDepthLockedRect;

    // Lock the frame data to access the un-clamped NUI_DEPTH_IMAGE_PIXELs
    hr = extendedDepthTex->LockRect(0, &extendedDepthLockedRect, nullptr, 0);
    if (FAILED(hr) || extendedDepthLockedRect.Pitch == 0)
    {
        throw std::runtime_error("Error getting extended depth texture pixels.");
        return hr;
    }

    /////////////////////////////////////////////////////////////////memcpy
    // Copy the depth pixels so we can return the image frame
    errno_t err = memcpy_s(m_pDepthImagePixelBuffer, cDepthImagePixels * sizeof(NUI_DEPTH_IMAGE_PIXEL), extendedDepthLockedRect.pBits, extendedDepthTex->BufferLen());

    extendedDepthTex->UnlockRect(0);

    if (0 != err)
    {
        throw std::runtime_error("Error copying extended depth texture pixels.");
        return hr;
    }
    return hr;
}






//void DepthSensor::KeypressCallbackFunction(vtkObject* caller, long unsigned int eventId, void* clientData)
//{
//
//	// Get the keypress
//	vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
//
//	std::string key = iren->GetKeySym();
//
//	// Output the key that was pressed
//	std::cout << "Pressed " << key << std::endl;
//
//	// Handle an arrow key
//	if (key == "s")
//	{
//		if (this->SaveMesh())
//		{
//			std::cout << "save Mesh successed." << std::endl;
//		}
//	}
//
//
//}

void DepthSensor::Update()
{
    // Initialize must be called prior to creating timer events.
    mDrawDepth->interactor->Initialize();
    // Sign up to receive TimerEvent
    vtkTimerCallback* timer = new vtkTimerCallback(this);
    mDrawDepth->interactor->AddObserver(vtkCommand::TimerEvent, timer);
    int timerId = mDrawDepth->interactor->CreateRepeatingTimer(100);

    vtkSmartPointer<vtkLambdaCommand> keypressCallback = vtkSmartPointer<vtkLambdaCommand>::New();
    
    keypressCallback->SetCallback(
        [&](vtkObject* caller, long unsigned int eventId, void* clientData)
        {
            // Get the keypress
            vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
            std::string key = iren->GetKeySym();
            // Output the key that was pressed
            std::cout << "Pressed " << key << std::endl;
            // Handle an arrow key
            if (key == "s")
            {
                if (this->SaveMesh())
                {
                    std::cout << "Saving Mesh successed." << std::endl;
                }
                //After save the mesh ,reset Recconstruction
                ResetReconstruction();
            }

            if (key == "r")
            {
                ResetReconstruction();
            }

            //press t and read STL File just have created
            if (key == "t")
            {
                if (m_saveMeshFormat == Stl || m_saveMeshFormat == Obj)
                {
                    cout << "Reading the mesh , waiting...... " << endl;
                    ReadModelFile((char*)filename.c_str(), m_saveMeshFormat);
                }
                else
                {
                    cout << "Read model file failed" << endl;
                }
            }
        }
    );
    mDrawDepth->interactor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);
    mDrawDepth->interactor->Start();

    
}



/// <summary>
/// Calculate a mesh for the current volume
/// </summary>
/// <param name="ppMesh">returns the new mesh</param>
HRESULT DepthSensor::CalculateMesh(INuiFusionMesh** ppMesh)
{
    EnterCriticalSection(&m_lockVolume);
    HRESULT hr = E_FAIL;
    if (m_pVolume != nullptr)
    {
        hr = m_pVolume->CalculateMesh(1, ppMesh);

        // Set the frame counter to 0 to prevent a reset reconstruction call due to large frame 
        // timestamp change after meshing. Also reset frame time for fps counter.
        m_cFrameCounter = 0;
        m_fStartTime = m_timer.AbsoluteTime();
    }
    LeaveCriticalSection(&m_lockVolume);
    return hr;
}

// save the mesh 
bool DepthSensor::SaveMesh()
{
    INuiFusionMesh *mesh = nullptr;
    HRESULT hr = this->CalculateMesh(&mesh);
    if (SUCCEEDED(hr))
    {
        // Save mesh
        hr = SaveFile(mesh, &m_saveMeshFormat);

        if (SUCCEEDED(hr))
        {
            return true;
        }
        else
        {
            throw std::runtime_error("Error saving Kinect Fusion mesh!");
        }

        // Release the mesh
        SafeRelease(mesh);
    }
    
    return false;
    

}



/// <summary>
/// Save Mesh to disk.
/// </summary>
/// <param name="mesh">The mesh to save.</param>
/// <returns>indicates success or failure</returns>
HRESULT DepthSensor::SaveFile(INuiFusionMesh* pMesh, KinectFusionMeshTypes *saveMeshType)
{
    HRESULT hr = S_OK;

    if (nullptr == pMesh)
    {
        return E_INVALIDARG;
    }

    //string filename;
    cout << "Please enter a file name to write: ";
    getline(cin, filename);
    

    if (filename.substr(filename.find_last_of(".") + 1) == "stl")
    {
        *saveMeshType = Stl;
    }
    else if (filename.substr(filename.find_last_of(".") + 1) == "obj")
    {
        *saveMeshType = Obj;
    }
    //with no extension name ,set default  type ***a.stl
    else
    {
        filename = filename +"a"+".stl";		
    }

    //change file name from string to char*
    char *cfilename = (char*)filename.c_str();

    
    if (Stl == *saveMeshType)
    {
        cout << "Creating and saving mesh in format stl,please waiting...... " << endl;
        hr = WriteBinarySTLMeshFile(pMesh, cfilename);
    }
    else if (Obj == *saveMeshType)
    {
        cout << "Creating and saving mesh in format obj,please waiting...... " << endl;
        hr = WriteAsciiObjMeshFile(pMesh, cfilename);
    }
    else
    {
        hr = E_FAIL;
    }
    return hr;
}


DepthSensor::~DepthSensor()
{
    if (mNuiSensor != 0)
    {
        mNuiSensor->NuiShutdown();
        mNuiSensor->Release();
    }

    if (mNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(mNextDepthFrameEvent);
    }
    delete[] m_depthRGBX;

    // clean up vtk renderer
    delete mDrawDepth;
    mDrawDepth = NULL;

    DeleteCriticalSection(&m_lockVolume);
}


int main()
{
    DepthSensor Fusion;
    Fusion.init();
    Fusion.Update();
    return 0;
}
