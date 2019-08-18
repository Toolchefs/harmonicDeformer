#include "computeWeightsCmd.h"
#include "grid.h"
#include "cell.h"
#include <maya/MSelectionList.h>
#include <maya/MFnMesh.h>
#include <maya/MDagPath.h>
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPlug.h>
#include <maya/MArgDatabase.h>
#include <maya/MPlugArray.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MTimer.h>

#define cellSizeFlagShort "-cs"
#define cellSizeFlagLong "-cellSize"

#define maxIterationFlagShort "-mi"
#define maxIterationFlagLong "-maxIteration"

#define deformerFlagShort "-d"
#define deformerFlagLong "-deformer"

#define saveGridFlagShort "-sg"
#define saveGridFlagLong "-saveGrid"

#define thresholdFlagShort "-ts"
#define thresholdFlagLong "-threshold"

MSyntax ComputeWeightsCmd::newSyntax(){
	MSyntax syntax;
	syntax.addFlag(cellSizeFlagShort, cellSizeFlagLong, MSyntax::kDouble);
	syntax.addFlag(maxIterationFlagShort, maxIterationFlagLong, MSyntax::kLong);
	syntax.addFlag(deformerFlagShort, deformerFlagLong, MSyntax::kString);
	syntax.addFlag(saveGridFlagShort, saveGridFlagLong, MSyntax::kBoolean);
	syntax.addFlag(thresholdFlagShort, thresholdFlagLong, MSyntax::kDouble);
	return syntax;
}


MStatus ComputeWeightsCmd::doIt(const MArgList& args)
{
	MStatus status;
	MSyntax syntax = newSyntax();
	MArgDatabase argData(syntax, args);

	double cellSize = 1.0;
	if (argData.isFlagSet(cellSizeFlagShort))
		argData.getFlagArgument(cellSizeFlagShort, 0, cellSize);

	double threshold = 0.00001;
	if (argData.isFlagSet(thresholdFlagShort))
		argData.getFlagArgument(thresholdFlagShort, 0, threshold);

	int iterations = 50;
	if (argData.isFlagSet(maxIterationFlagShort))
		argData.getFlagArgument(maxIterationFlagShort, 0, iterations);

	bool saveGrid = false;
	if (argData.isFlagSet(saveGridFlagShort))
		argData.getFlagArgument(saveGridFlagShort, 0, saveGrid);

	MString deformerPath;
	if (argData.isFlagSet(deformerFlagShort))
	{
		argData.getFlagArgument(deformerFlagShort, 0, deformerPath);
	}
	else
	{
		MGlobal::displayError("Please add the harmonic deformer name with the -d/-deformer flag");
		return MS::kFailure;
	}

	MSelectionList currList;
	currList.add(deformerPath);

	//MDagPath meshPath;
	//currList.getDagPath(0, meshPath);

	MObject defomerNode;
	currList.getDependNode(0, defomerNode);
	MFnDependencyNode defNode(defomerNode);
	MPlug gridPlug = defNode.findPlug("gridData");


	MPlug refCagePlug = defNode.findPlug("refCage");
	MPlugArray connections;
	refCagePlug.connectedTo(connections, true, false, &status);
	if (connections.length() == 0)
	{
		MGlobal::displayError("No cage mesh find on "+refCagePlug.name());
		return MS::kFailure;
	}

	MDagPath meshPath = MDagPath::getAPathTo(connections[0].node(), &status);
	if (status != MS::kSuccess)
	{
		MGlobal::displayError("Error getting a path to the cage mesh: " + status);
		return MS::kFailure;
	}

	MTimer timer;
	timer.beginTimer();

	MFnMesh meshFn(meshPath);
	MPointArray points;
	meshFn.getPoints(points, MSpace::kWorld);

	MIntArray faceVtx, numVtxPerFace;
	meshFn.getVertices(numVtxPerFace, faceVtx);

	tc::Grid grid(cellSize);
	grid.setThreshold(threshold);

	
	std::vector<tc::Vector> pointsVec(points.length());
	for (unsigned int i = 0; i < points.length(); ++i)
	{
		pointsVec[i].x = points[i].x;
		pointsVec[i].y = points[i].y;
		pointsVec[i].z = points[i].z;
	}

	std::vector<unsigned int> faceVtxVec(faceVtx.length());
	for (unsigned int i = 0; i < faceVtx.length(); ++i)
	{
		faceVtxVec[i] = faceVtx[i];
	}

	std::vector<unsigned int> numVtxPerFaceVec(numVtxPerFace.length());
	for (unsigned int i = 0; i < numVtxPerFace.length(); ++i)
	{
		numVtxPerFaceVec[i] = numVtxPerFace[i];
	}

	grid.addBoundary(pointsVec, faceVtxVec, numVtxPerFaceVec);
	grid.parallelSolveLaplace(pointsVec, iterations);

	if (saveGrid)
	{
		std::string serialise = grid.serialise();

		gridPlug.setValue(MString(serialise.c_str()));
	}

	MStringArray result;
	MGlobal::executeCommand("deformer -q -g "+defNode.name(), result, false, false);
	
	if (result.length() == 0)
	{
		MGlobal::displayError("Error getting mesha ttathed to the deformer: " + status);
		return MS::kFailure;
	}

	MSelectionList tempList;
	tempList.add(result[0]);
	MObject node;
	tempList.getDependNode(0, node);

	MDagPath modelPath = MDagPath::getAPathTo(node, &status);
	if (status != MS::kSuccess)
	{
		MGlobal::displayError("Error getting a path to the model mesh: " + status);
		return MS::kFailure;
	}

	MFnMesh modelFn(modelPath);
	MPointArray modelPoints;
	modelFn.getPoints(modelPoints, MSpace::kWorld);

	std::vector<tc::Vector> outPoints;
	outPoints.resize(modelPoints.length());
	for (unsigned int v = 0; v < modelPoints.length(); ++v)
	{
		outPoints[v] = tc::Vector(modelPoints[v].x, modelPoints[v].y, modelPoints[v].z);
	}
	std::vector<std::map<unsigned int, double> > weights = grid.getWeights(outPoints);

	MDoubleArray serializedWeights;
	serializedWeights.append(static_cast<double>(weights.size()));
	for (unsigned int i = 0; i < weights.size(); ++i)
	{
		unsigned int weightsCounter = 0;
        std::map<unsigned int, double> tmpMap;
		for (std::map<unsigned int, double>::iterator it = weights[i].begin(); it != weights[i].end(); ++it)
		{
			if (it->second > threshold)
            {
				weightsCounter += 1;
                tmpMap[it->first] =it->second;
            }
		}
		serializedWeights.append(static_cast<double>(weightsCounter));
		for (std::map<unsigned int, double>::iterator it = tmpMap.begin(); it != tmpMap.end(); ++it)
		{
            serializedWeights.append(static_cast<double>(it->first));
            serializedWeights.append(it->second);
		}
	}

	MPlug pwPlug = defNode.findPlug("pointWeights");
	MFnDoubleArrayData pwData;
	MObject data = pwData.create(serializedWeights);
	pwPlug.setMObject(data);
	
	/*
	tc::Vector min = grid.m_boundingBox.first;
	tc::Vector max = grid.m_boundingBox.second;
	for (unsigned int cellXId = 0; cellXId < grid.m_xDim; ++cellXId)
	{
		for (unsigned int cellYId = 0; cellYId < grid.m_yDim; ++cellYId)
		{
			for (unsigned int cellZId = 0; cellZId < grid.m_zDim; ++cellZId)
			{
				tc::Cell& cell = grid.m_grid[grid.linearCellCords(cellXId, cellYId, cellZId)];
				if ((cell.tag == tc::Cell::kIN) || (cell.tag == tc::Cell::kBORDER) )
				{
					tc::Vector cellMin(min.x + cellXId*grid.m_cellDimension, min.y + cellYId*grid.m_cellDimension, min.z + cellZId*grid.m_cellDimension);
					tc::Vector cellMax(cellMin.x + grid.m_cellDimension, cellMin.y + grid.m_cellDimension, cellMin.z + grid.m_cellDimension);
					tc::Vector cellCenter = cellMin + (cellMax - cellMin)*0.5;
					MString weightsStr;

					if (cell.tag == tc::Cell::kBORDER)
					{
						weightsStr += "border_";
					}
					if (cell.tag == tc::Cell::kOUT)
					{
						weightsStr += "out_";
					}
					if (cell.tag == tc::Cell::kIN)
					{
						weightsStr += "in_";
					}
					if (cell.tag == tc::Cell::kUNDEFINED)
					{
						weightsStr += "undef_";
					}
					weightsStr += cellXId;
					weightsStr += "_";
					weightsStr += cellYId;
					weightsStr += "_";
					weightsStr += cellZId;

					std::cout << "cell " << cellXId << " " << cellYId << " " << cellZId << " ---> ";
					for (std::map<unsigned int, double>::iterator it = cell.weights.begin(); it != cell.weights.end(); ++it)
					{
						if (it->second > 0.01)
						{
							std::cout << " " << it->first << ":" << it->second << " ";
							
							MString ppp = "_vtx";
							ppp += it->first;
							if (it->first == 1)
							{
								MString command = "{";
								command += "string $puppa[] = `polyCube - w 1.0 - h 1.0 - d 1.0 -n " + weightsStr + ppp + "`;";
								command += "setAttr($puppa[0] + \".tx\")";  command += cellCenter.x; command += ";";
								command += "setAttr($puppa[0] + \".ty\")";  command += cellCenter.y; command += ";";
								command += "setAttr($puppa[0] + \".tz\")";  command += cellCenter.z; command += ";";
								command += "setAttr($puppa[0] + \".sx\")";  command += it->second;  command += ";";
								command += "setAttr($puppa[0] + \".sy\")";  command += it->second; command += ";";
								command += "setAttr($puppa[0] + \".sz\")";  command += it->second; command += ";";
								command += "}";

								MGlobal::executeCommand(command, false, false);
							}
							
						}
					}
					
					std::cout << std::endl;
				}
			}
		}
	}
	*/
	timer.endTimer();
	double eTime = timer.elapsedTime();
	MString etimeStr;
	etimeStr += eTime;
	MGlobal::displayInfo("Harmonic weights computed in " + etimeStr + " seconds");
	return status;
}

void* ComputeWeightsCmd::creator()
{
	return new ComputeWeightsCmd();
}