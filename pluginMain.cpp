//
// Copyright (C) Toolchefs 
// 
// File: pluginMain.cpp
//
// Author: Maya Plug-in Wizard 2.0
//

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include "computeWeightsCmd.h"
#include "harmonicDeformer.h"
#include "harmonicDeformerCmd.h"

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{ 
	MStatus   status;
	MFnPlugin plugin( obj, "Toolchefs", "2015", "Any");

	// Add plug-in feature registration here
	//
	status = plugin.registerCommand("tcComputeHarmonicWeights", ComputeWeightsCmd::creator, ComputeWeightsCmd::newSyntax);
	status = plugin.registerCommand("tcCreateHarmonicDeformer", HarmonicDeformerCmd::creator, HarmonicDeformerCmd::newSyntax);

	status = plugin.registerNode("tcHarmonicDeformer", HarmonicDeformer::id, HarmonicDeformer::creator,
		HarmonicDeformer::initialize, MPxNode::kDeformerNode);


	MString addMenu;
	addMenu += 
	"global proc addTcHarmonicDeformerToMenu()\
	{\
		global string $gMainWindow;\
		global string $showToolochefsMenuCtrl;\
		if (!(`menu - exists $showToolochefsMenuCtrl`))\
		{\
			string $name = \"Toolchefs\";\
			$showToolochefsMenuCtrl = `menu -p $gMainWindow -to true -l $name`;\
			string $tcDeformerMenu = `menuItem -subMenu true -label \"Deformers\" -p $showToolochefsMenuCtrl \"tcDeformerMenu\"`;\
			menuItem -label \"tcHarmonicDeformer\" -p $tcDeformerMenu -c \"tcCreateHarmonicDeformer\" \"tcCreateHarmonicDeformerItem\";\
		}\
		else\
		{\
			int $deformerMenuExist = false;\
			string $defMenu = \"\";\
			string $subitems[] = `menu - q - itemArray $showToolochefsMenuCtrl`;\
				for ($item in $subitems)\
				{\
					if ($item == \"tcDeformerMenu\")\
					{\
						$deformerMenuExist = true;\
						$defMenu = $item;\
						break;\
					}\
				}\
			if (!($deformerMenuExist))\
			{\
				string $tcDeformerMenu = `menuItem -subMenu true -label \"Deformers\" -p $showToolochefsMenuCtrl \"tcDeformerMenu\"`;\
				menuItem -label \"tcHarmonicDeformer\" -p $tcDeformerMenu -c \"tcCreateHarmonicDeformer\" \"tcCreateHarmonicDeformerItem\";\
			}\
			else\
			{\
				string $subitems2[] = `menu -q -itemArray \"tcDeformerMenu\"`;\
					int $deformerExists = 0;\
				for ($item in $subitems2)\
				{\
					if ($item == \"tcCreateHarmonicDeformerItem\")\
					{\
						$deformerExists = true;\
						break;\
					}\
				}\
				if (!$deformerExists)\
				{\
					menuItem -label \"tcHarmonicDeformer\" -p $defMenu -c \"tcCreateHarmonicDeformer\" \"tcCreateHarmonicDeformerItem\";\
				}\
			}\
		}\
	};addTcHarmonicDeformerToMenu();";
	MGlobal::executeCommand(addMenu, false, false);

	return status;
}

MStatus uninitializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

	// Add plug-in feature deregistration here
	//
	status = plugin.deregisterCommand("tcComputeHarmonicWeights");
	status = plugin.deregisterCommand("tcCreateHarmonicDeformer");
	status = plugin.deregisterNode(HarmonicDeformer::id);
	return status;
}
