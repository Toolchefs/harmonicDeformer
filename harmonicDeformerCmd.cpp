#include "harmonicDeformerCmd.h"
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MSelectionList.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MStringArray.h>
#include <maya/MArgDatabase.h>



HarmonicDeformerCmd::HarmonicDeformerCmd(){}

HarmonicDeformerCmd::~HarmonicDeformerCmd(){}

void* HarmonicDeformerCmd::creator(){
	return new HarmonicDeformerCmd();
}

MSyntax HarmonicDeformerCmd::newSyntax(){
	MSyntax syntax;
	
	return syntax;
}

MStatus HarmonicDeformerCmd::doIt(const MArgList& args)
{
	MStatus stat = MS::kSuccess;

	MSyntax syntax = newSyntax();

	MArgDatabase argData(syntax, args);


	MDagPath cage, model;
	MDagPathArray PathArray;
	MSelectionList	list;
	MGlobal::getActiveSelectionList(list);

	if (list.length() < 2)
	{
		MGlobal::displayError("Please select first the model and then the cage");
		return MS::kFailure;
	}

	for (unsigned int index = 0; index < list.length(); index++)
	{
		MDagPath currentPath;
		list.getDagPath(index, currentPath, MObject::kNullObj);
		PathArray.insert(currentPath, index);
	}
	model = PathArray[0];
	cage = PathArray[1];
	

	cage.extendToShape();

	MObject cageShape = cage.node();
	MObject cageTransf = cage.transform();
	MObject	ModelObj = model.node();

	MFnMesh cageFn(cage, &stat);
	MObject CageBase = cageFn.copy(cageShape, cageTransf, &stat);
	if (stat != MS::kSuccess) MGlobal::displayError("Error on copy shape");

	MFnDependencyNode cageDef(cageShape);
	MFnDependencyNode cageDefTrans(cageTransf);
	MString cageTransfName = cageDefTrans.name();
	MString nameDef = cageDef.name();
	MString nameBase = nameDef + "_Base";
	MFnDependencyNode cageBase(CageBase);
	cageBase.setName(nameBase, &stat);
	MGlobal::executeCommand("setAttr \"" + nameBase + ".intermediateObject\" 1", false, true);

	MFnDependencyNode ModelNode(ModelObj);
	MString modelName = ModelNode.name();
	MObject	modelTransf = model.transform();
	MFnDependencyNode ModelNodeTransf(modelTransf);
	MString modelNameTransf = ModelNodeTransf.name();

	MStringArray resultArr;
	MGlobal::executeCommand("deformer -type tcHarmonicDeformer " + modelName, resultArr, false, true);

	//add file attribute
	//MGlobal::executeCommand("addAttr -ln \"WeightsFile\" -dt \"string\" " + resultArr[0], false, true);
	//MGlobal::executeCommand("setAttr -type \"string\" " + resultArr[0] + ".WeightsFile \"" + FileName + "\"", false, true);

	MGlobal::executeCommand("connectAttr -f " + nameBase + ".worldMesh[0] " + resultArr[0] + ".refCage", false, true);
	MGlobal::executeCommand("connectAttr -f " + nameDef + ".worldMesh[0] " + resultArr[0] + ".cage", false, true);

	MGlobal::executeCommand("setAttr \"" + cageTransfName + ".overrideEnabled\" 1", false, true);
	MGlobal::executeCommand("setAttr \"" + cageTransfName + ".overrideShading\" 0", false, true);

	MGlobal::executeCommand("select -r " + cageTransfName + " " + modelNameTransf, false, true);
	MGlobal::executeCommand("parentConstraint -mo -weight 1", false, true);


	if (resultArr.length() > 0)
		setResult(resultArr[0]);

	return MS::kSuccess;
}
