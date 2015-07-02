#include "vtkImageRender.h"


vtkImageRender::vtkImageRender():
m_sourceWidth(0),
m_sourceHeight(0),
m_sourceStride(0)
{
} 


vtkImageRender::~vtkImageRender()
{
}

bool vtkImageRender::Initialize(int sourceWidth, int sourceHeight, int sourceStride)
{
	// Get the frame size
	m_sourceWidth = sourceWidth;
	m_sourceHeight = sourceHeight;
	m_sourceStride = sourceStride;

	//set windows and renderer 
	renWin->AddRenderer(renderer);
	renWin->SetSize(800, 600);
	renWin->SetPosition(500, 200);

	//set interactor
    interactor->SetRenderWindow(renWin);

	return true;
}





void vtkImageRender::Draw(BYTE* pImage, int sourceWidth, int sourceHeight, int cBytesPerPixel)
{
	
	
	image->SetDimensions(sourceWidth, sourceHeight, cBytesPerPixel);
	
	image->AllocateScalars(VTK_UNSIGNED_CHAR, 4);	

	int* dims = image->GetDimensions();

	//std::cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << std::endl;

	// Fill every entry of the image 
	for (int y = dims[1]-1; y >=0; y--)
	{
		for (int x = 0; x < dims[0]; x++)
		{			
				unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x, y, 0));
				pixel[0] = *pImage++;
				pixel[1] = *pImage++;
				pixel[2] = *pImage++;
				*pImage++;					
		}
	}
	image->Modified();
}

