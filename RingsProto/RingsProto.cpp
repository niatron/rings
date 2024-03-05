
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
//#include <CAM/CAMAll.h>
#include "RingsProtoCreator.h"

using namespace adsk::core;
using namespace adsk::fusion;
//using namespace adsk::cam;

Ptr<Application> app;
Ptr<UserInterface> ui;
Ptr<Document> doc;
Ptr<Design> des;
Ptr<Component> rootComp;

bool Init();

extern "C" XI_EXPORT bool run(const char* context)
{
	app = Application::get();
	if (!app)
		return false;

	ui = app->userInterface();
	if (!ui)
		return false;
	
	if (!Init())
	{
		ui->messageBox("Init Error");
		return false;
	}
	
	ui->messageBox("Start");
	RingsProtoCreator ringsCreator(4, 3, 12, 0.1, 0.3, 0.3, 0.25, 0.03, 0.01);
	ringsCreator.createBody(rootComp);

	ui->messageBox("Ok");
	

	return true;
}

bool Init()
{
	doc = app->activeDocument();
	if (!doc)
		return false;

	Ptr<Products> prods = doc->products();
	if (!prods)
		return false;

	Ptr<Product> prod = prods->itemByProductType("DesignProductType");
	if (!prod)
		return false;
	
	des = prod->cast<Design>();
	if (!des)
		return false;
	// get the entry for custom graphics
	Ptr<Product> activeProd = app->activeProduct();
	if (!activeProd)
		return false;

	rootComp = des->rootComponent();
	if (!rootComp)
		return false;

	/*Ptr<CustomGraphicsGroups> _cgGroups;
	Ptr<CAM> cam = activeProd->cast<CAM>();
	if (cam) {
		_cgGroups = cam->customGraphicsGroups();
	}
	else
	{
		_cgGroups = rootComp->customGraphicsGroups();
	}
	if (!_cgGroups)
		return false;*/


	return true;
}

#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
