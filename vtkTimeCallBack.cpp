//
//#include "vtkTimeCallBack.h"
//
//
//
//vtkTimerCallback::vtkTimerCallback()
//{
//}
//
//vtkTimerCallback::vtkTimerCallback(DepthSensor* sensor) :
//	m_sensor(sensor)
//{
//}
//
//void vtkTimerCallback::Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId, void *vtkNotUsed(callData))
//{
//	std::cout << "timer" << std::endl;
//	if (vtkCommand::TimerEvent == eventId)
//	{
//		std::cout << "timer eventId:" << eventId << std::endl;
//		m_sensor->processDepth();
//	}
//}
//
//vtkTimerCallback::~vtkTimerCallback(){};
//
//	
