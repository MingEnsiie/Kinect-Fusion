#pragma once
//VTK
#include <windows.h>
#include <NuiApi.h>

//VTK headers
#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>
#include <vtkImageData.h>
#include <vtkImageMapper3D.h>
#include <vtkImageMapToColors.h>
#include <vtkPointData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>

#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>


class vtkImageRender
{
public:
	vtkImageRender();
	~vtkImageRender();

	/// <summary>
	/// Set the window to draw to as well as the video format
	/// </summary>
	/// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
	/// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
	/// <param name="sourceStride">length (in bytes) of a single scanline</param>
	/// <returns>indicates success or failure</returns>
	bool Initialize( int sourceWidth, int sourceHeight, int sourceStride);


	void Draw(BYTE* pImage, int sourceWidth, int sourceHeight, int cBytesPerPixel);

	//image data
	vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();

	//actor to represent 
	vtkSmartPointer<vtkImageActor> Actor = vtkSmartPointer<vtkImageActor>::New();

	//Renderer
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

	//Render windows
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();

	//Interactor
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

private:

	// Format information
	UINT                     m_sourceHeight;
	UINT                     m_sourceWidth;
	LONG                     m_sourceStride;
};

