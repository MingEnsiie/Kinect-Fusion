

#include "vtkImageRender.h"

class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
	public:
		static KeyPressInteractorStyle* New();
		vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

		virtual void OnKeyPress()
		{
			// Get the keypress
			vtkRenderWindowInteractor *rwi = this->Interactor;
			std::string key = rwi->GetKeySym();

			// Output the key that was pressed
			std::cout << "Pressed " << key << std::endl;

			// Handle an arrow key
			if (key == "Up")
			{
				std::cout << "The up arrow was pressed." << std::endl;
			}

			// Forward events
			vtkInteractorStyleTrackballCamera::OnKeyPress();
		}
};
