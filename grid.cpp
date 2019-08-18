#include "grid.h"
#include "cell.h"
#include <limits>
#include <list>
#include <set>
#include <sstream>
#include <tbb/parallel_for.h>

using namespace tc;

#ifdef MAYA
tbb::queuing_mutex ParallelSolver::m_mutex;
#endif

void ParallelSolver::operator()(const tbb::blocked_range<size_t>& range) const
{
		
	for (size_t pp = range.begin(); pp != range.end(); ++pp)
	{
		//getPointCell
		double VoxX = (points[pp].x - grid.m_boundingBox.first.x) / grid.m_cellDimension;
		int Xvox = static_cast<int>(floor(VoxX));

		double VoxY = (points[pp].y - grid.m_boundingBox.first.y) / grid.m_cellDimension;
		int Yvox = static_cast<int>(floor(VoxY));

		double VoxZ = (points[pp].z - grid.m_boundingBox.first.z) / grid.m_cellDimension;
		int Zvox = static_cast<int>(floor(VoxZ));

		unsigned int pointCellId = grid.linearCellCords(Xvox, Yvox, Zvox);

		std::vector<unsigned int> boundaryOfPoints;
		std::vector<Vector> innerCells;
		for (unsigned int cellXId = 0; cellXId < grid.m_xDim; ++cellXId)
		{
			for (unsigned int cellYId = 0; cellYId < grid.m_yDim; ++cellYId)
			{
				for (unsigned int cellZId = 0; cellZId < grid.m_zDim; ++cellZId)
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(cellXId, cellYId, cellZId)];
					if (pointCellId != grid.linearCellCords(cellXId, cellYId, cellZId))
					{
						if ((currCell.tag == Cell::kIN))
							innerCells.push_back(Vector(cellXId, cellYId, cellZId));
					}
				}
			}
		}

		for (unsigned int i = 0; i < iteration; ++i)
		{
			for (int j = 0; j < innerCells.size(); ++j)
			{

				std::map<unsigned int, double> weights;
				unsigned numCell = 1;
				Vector &cellId = innerCells[j];
				unsigned int xId = static_cast<unsigned int>(cellId.x);
				unsigned int yId = static_cast<unsigned int>(cellId.y);
				unsigned int zId = static_cast<unsigned int>(cellId.z);

				Cell &cell = grid.m_grid[grid.linearCellCords(xId, yId, zId)];

				if (xId > 0)
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId - 1, yId, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (xId < (grid.m_xDim - 1))
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId + 1, yId, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				if (yId > 0)
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId, yId - 1, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (yId < (grid.m_yDim - 1))
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId, yId + 1, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				if (zId > 0)
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId, yId, zId - 1)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (zId < (grid.m_zDim - 1))
				{
					Cell &currCell = grid.m_grid[grid.linearCellCords(xId, yId, zId + 1)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				cell.weights[pp] = cell.weights[pp] / static_cast<double>(numCell);
			}
		}
#ifdef MAYA
		{
			tbb::queuing_mutex::scoped_lock lock(m_mutex);
			MProgressWindow::advanceProgress(1);
		}
#endif
	}
}

Grid::Grid():
	m_cellDimension(0.1),
	m_xDim(1),
	m_yDim(1),
	m_zDim(1),
	m_threshold(0.00001)
{

}

Grid::Grid(double cellDimension):
	m_cellDimension(cellDimension),
	m_xDim(1),
	m_yDim(1),
	m_zDim(1),
	m_threshold(0.00001)
{

}

Grid::~Grid()
{

}

int Grid::linearCellCords(unsigned int x, unsigned int y, unsigned int z)
{
	x = x < 0 ? 0 : (x >= m_xDim ? m_xDim - 1 : x);
	y = y < 0 ? 0 : (y >= m_yDim ? m_yDim - 1 : y);
	z = z < 0 ? 0 : (z >= m_zDim ? m_zDim - 1 : z);
	return x + y * m_xDim + z * m_xDim * m_yDim;
}

bool Grid::addBoundary(const std::vector<Vector>& points, const std::vector<unsigned int>& faceVtx, const std::vector<unsigned int> numVtxPerFace)
{
	// compute bounding box
	Vector minP(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
	Vector maxP(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());

	for (std::vector<Vector>::const_iterator it = points.begin(); it != points.end(); ++it)
	{
		if (it->x < minP.x) minP.x = it->x;
		if (it->y < minP.y) minP.y = it->y;
		if (it->z < minP.z) minP.z = it->z;

		if (it->x > maxP.x) maxP.x = it->x;
		if (it->y > maxP.y) maxP.y = it->y;
		if (it->z > maxP.z) maxP.z = it->z;
	}


	Vector center((minP.x + maxP.x) * 0.5, (minP.y + maxP.y) * 0.5, (minP.z + maxP.z) * 0.5);

	minP = center + (minP - center) * 1.1;
	maxP = center + (maxP - center) * 1.1;

	m_boundingBox.first = minP;
	m_boundingBox.second = maxP;
		
	m_xDim = static_cast<unsigned int>(ceil((maxP.x - minP.x) / m_cellDimension));
	m_yDim = static_cast<unsigned int>(ceil((maxP.y - minP.y) / m_cellDimension));
	m_zDim = static_cast<unsigned int>(ceil((maxP.z - minP.z) / m_cellDimension));

	if (m_xDim < 1) m_xDim = 1;
	if (m_yDim < 1) m_yDim = 1;
	if (m_zDim < 1) m_zDim = 1;

	m_grid.resize(m_xDim * m_yDim * m_zDim);

	//build acceleration structure
	unsigned int vtxCounter = 0;

#ifdef MAYA
	MProgressWindow::reserve();
	MProgressWindow::setInterruptable(false);
	MString title = "Voxelization...";
	MProgressWindow::setTitle(title);
	MProgressWindow::setProgressMin(0);
	MProgressWindow::setProgressMax(numVtxPerFace.size());
	MProgressWindow::setProgress(0);
	MProgressWindow::startProgress();
#endif
	
	for (int faceId = 0; faceId < numVtxPerFace.size(); ++faceId)
	{
		if ((numVtxPerFace[faceId] == 3) || (numVtxPerFace[faceId] == 4))
		{
			Vector v1 = points[faceVtx[vtxCounter]];
			Vector v2 = points[faceVtx[vtxCounter + 1]];
			Vector v3 = points[faceVtx[vtxCounter + 2]];

			Vector faceMin(v1);
			Vector faceMax(v1);

			if (v2.x < faceMin.x) faceMin.x = v2.x;
			if (v2.y < faceMin.y) faceMin.y = v2.y;
			if (v2.z < faceMin.z) faceMin.z = v2.z;

			if (v3.x < faceMin.x) faceMin.x = v3.x;
			if (v3.y < faceMin.y) faceMin.y = v3.y;
			if (v3.z < faceMin.z) faceMin.z = v3.z;

			if (v2.x > faceMax.x) faceMax.x = v2.x;
			if (v2.y > faceMax.y) faceMax.y = v2.y;
			if (v2.z > faceMax.z) faceMax.z = v2.z;

			if (v3.x > faceMax.x) faceMax.x = v3.x;
			if (v3.y > faceMax.y) faceMax.y = v3.y;
			if (v3.z > faceMax.z) faceMax.z = v3.z;

			int minXvox = static_cast<int>(floor((faceMin.x - minP.x) / m_cellDimension));
			int minYvox = static_cast<int>(floor((faceMin.y - minP.y) / m_cellDimension));
			int minZvox = static_cast<int>(floor((faceMin.z - minP.z) / m_cellDimension));

			int maxXvox = static_cast<int>(floor((faceMax.x - minP.x) / m_cellDimension));
			int maxYvox = static_cast<int>(floor((faceMax.y - minP.y) / m_cellDimension));
			int maxZvox = static_cast<int>(floor((faceMax.z - minP.z) / m_cellDimension));

			

			for (unsigned int cellXId = minXvox; cellXId <= maxXvox; ++cellXId)
			{
				for (unsigned int cellYId = minYvox; cellYId <= maxYvox; ++cellYId)
				{
					for (unsigned int cellZId = minZvox; cellZId <= maxZvox; ++cellZId)
					{
						Cell &cell = m_grid[linearCellCords(cellXId, cellYId, cellZId)];
						Vector cellMin(minP.x + cellXId*m_cellDimension, minP.y + cellYId*m_cellDimension, minP.z + cellZId*m_cellDimension);
						Vector cellMax(cellMin.x + m_cellDimension, cellMin.y + m_cellDimension, cellMin.z + m_cellDimension);
						Vector cellCenter = cellMin + (cellMax - cellMin)*0.5;
						if (voxelTriangleIntersection(v1, v2, v3, cellMin, cellMax))
						{
							Vector baryCoord;
							Vector closestPoint = ClosestPoint(v1, v2, v3, cellCenter, baryCoord);

							cell.tag = Cell::kBORDER;
							
							if (baryCoord.x > 0.000001)
							{
								cell.weights[faceVtx[vtxCounter]] = baryCoord.x;
							}

							if (baryCoord.y > 0.000001)
							{
								cell.weights[faceVtx[vtxCounter + 1]] = baryCoord.y;
							}

							if (baryCoord.z > 0.000001)
							{
								cell.weights[faceVtx[vtxCounter + 2]] = baryCoord.z;
							}
						}
					}
				}
			}
			
			if (numVtxPerFace[faceId] == 4)
			{
				Vector v1 = points[faceVtx[vtxCounter + 2]];
				Vector v2 = points[faceVtx[vtxCounter + 3]];
				Vector v3 = points[faceVtx[vtxCounter]];

				Vector faceMin(v1);
				Vector faceMax(v1);

				if (v2.x < faceMin.x) faceMin.x = v2.x;
				if (v2.y < faceMin.y) faceMin.y = v2.y;
				if (v2.z < faceMin.z) faceMin.z = v2.z;

				if (v3.x < faceMin.x) faceMin.x = v3.x;
				if (v3.y < faceMin.y) faceMin.y = v3.y;
				if (v3.z < faceMin.z) faceMin.z = v3.z;

				if (v2.x > faceMax.x) faceMax.x = v2.x;
				if (v2.y > faceMax.y) faceMax.y = v2.y;
				if (v2.z > faceMax.z) faceMax.z = v2.z;

				if (v3.x > faceMax.x) faceMax.x = v3.x;
				if (v3.y > faceMax.y) faceMax.y = v3.y;
				if (v3.z > faceMax.z) faceMax.z = v3.z;

				int minXvox = static_cast<int>(floor((faceMin.x - minP.x) / m_cellDimension));
				int minYvox = static_cast<int>(floor((faceMin.y - minP.y) / m_cellDimension));
				int minZvox = static_cast<int>(floor((faceMin.z - minP.z) / m_cellDimension));

				int maxXvox = static_cast<int>(floor((faceMax.x - minP.x) / m_cellDimension));
				int maxYvox = static_cast<int>(floor((faceMax.y - minP.y) / m_cellDimension));
				int maxZvox = static_cast<int>(floor((faceMax.z - minP.z) / m_cellDimension));



				for (unsigned int cellXId = minXvox; cellXId <= maxXvox; ++cellXId)
				{
					for (unsigned int cellYId = minYvox; cellYId <= maxYvox; ++cellYId)
					{
						for (unsigned int cellZId = minZvox; cellZId <= maxZvox; ++cellZId)
						{
							Cell &cell = m_grid[linearCellCords(cellXId, cellYId, cellZId)];
							Vector cellMin(minP.x + cellXId*m_cellDimension, minP.y + cellYId*m_cellDimension, minP.z + cellZId*m_cellDimension);
							Vector cellMax(cellMin.x + m_cellDimension, cellMin.y + m_cellDimension, cellMin.z + m_cellDimension);
							Vector cellCenter = cellMin + (cellMax - cellMin)*0.5;
							if (voxelTriangleIntersection(v1, v2, v3, cellMin, cellMax))
							{
								Vector baryCoord;
								Vector closestPoint = ClosestPoint(v1, v2, v3, cellCenter, baryCoord);

								cell.tag = Cell::kBORDER;
								
								if (baryCoord.x > 0.000001)
								{
									cell.weights[faceVtx[vtxCounter + 2]] = baryCoord.x;
								}

								if (baryCoord.y > 0.000001)
								{
									cell.weights[faceVtx[vtxCounter + 3]] = baryCoord.y;
								}

								if (baryCoord.z > 0.000001)
								{
									cell.weights[faceVtx[vtxCounter]] = baryCoord.z;
								}		
							}
						}
					}
				}
			}
			
		}
		vtxCounter += numVtxPerFace[faceId];
#ifdef MAYA
		MProgressWindow::advanceProgress(1);
#endif
	}
	
	// set cell containing vertices
	for (unsigned int i = 0; i < points.size(); ++i)
	{

		double VoxX = (points[i].x - minP.x) / m_cellDimension;
		int Xvox = static_cast<int>(floor(VoxX));

		double VoxY = (points[i].y - minP.y) / m_cellDimension;
		int Yvox = static_cast<int>(floor(VoxY));

		double VoxZ = (points[i].z - minP.z) / m_cellDimension;
		int Zvox = static_cast<int>(floor(VoxZ));

		Cell &cell = m_grid[linearCellCords(Xvox, Yvox, Zvox)];
		cell.tag = Cell::kBORDER;
		cell.weights[i] = 1.0;
	}

	std::list<Vector> cellsToProcess;
	cellsToProcess.push_back(Vector(0, 0, 0));
	cellsToProcess.push_back(Vector(m_xDim - 1, 0, 0));
	cellsToProcess.push_back(Vector(m_xDim - 1, m_yDim - 1, 0));
	cellsToProcess.push_back(Vector(m_xDim - 1, m_yDim - 1, m_zDim - 1));
	cellsToProcess.push_back(Vector(0, m_yDim - 1, 0));
	cellsToProcess.push_back(Vector(0, m_yDim - 1, m_zDim - 1));
	cellsToProcess.push_back(Vector(0, 0, m_zDim - 1));
	cellsToProcess.push_back(Vector(m_xDim - 1, 0, m_zDim - 1));

	while (cellsToProcess.size()>0)
	{
		Vector currCellCoord = cellsToProcess.back();
		cellsToProcess.pop_back();

		Cell &currCell = m_grid[linearCellCords(static_cast<unsigned int>(currCellCoord.x), static_cast<unsigned int>(currCellCoord.y), static_cast<unsigned int>(currCellCoord.z))];
		if (!(currCell.tag == Cell::kUNDEFINED))
			continue;
		
		currCell.tag = Cell::kOUT;

		if (static_cast<unsigned int>(currCellCoord.x) < (m_xDim-1))
			cellsToProcess.push_back(currCellCoord + Vector(1.0, 0.0, 0.0));

		if (static_cast<unsigned int>(currCellCoord.y) < (m_yDim - 1))
			cellsToProcess.push_back(currCellCoord + Vector(0.0, 1.0, 0.0));

		if (static_cast<unsigned int>(currCellCoord.z) < (m_zDim - 1))
			cellsToProcess.push_back(currCellCoord + Vector(0.0, 0.0, 1.0));

		if (static_cast<unsigned int>(currCellCoord.x) > 0)
			cellsToProcess.push_back(currCellCoord - Vector(1.0, 0.0, 0.0));

		if (static_cast<unsigned int>(currCellCoord.y) > 0)
			cellsToProcess.push_back(currCellCoord - Vector(0.0, 1.0, 0.0));

		if (static_cast<unsigned int>(currCellCoord.z) > 0)
			cellsToProcess.push_back(currCellCoord - Vector(0.0, 0.0, 1.0));
	}

	for (unsigned int i = 0; i < m_grid.size(); ++i)
	{
		Cell &currCell = m_grid[i];
		if (currCell.tag == Cell::kUNDEFINED)
			currCell.tag = Cell::kIN;
	}

#ifdef MAYA
	MProgressWindow::endProgress();
#endif

	return true;
}

void Grid::solveLaplace(const std::vector<Vector>& points, unsigned int iteration)
{
	for (unsigned int pp = 0; pp < points.size(); ++pp)
	{
		//getPointCell
		double VoxX = (points[pp].x - m_boundingBox.first.x) / m_cellDimension;
		int Xvox = static_cast<int>(floor(VoxX));

		double VoxY = (points[pp].y - m_boundingBox.first.y) / m_cellDimension;
		int Yvox = static_cast<int>(floor(VoxY));

		double VoxZ = (points[pp].z - m_boundingBox.first.z) / m_cellDimension;
		int Zvox = static_cast<int>(floor(VoxZ));

		unsigned int pointCellId = linearCellCords(Xvox, Yvox, Zvox);

		std::vector<unsigned int> boundaryOfPoints;
		std::vector<Vector> innerCells;
		for (unsigned int cellXId = 0; cellXId < m_xDim; ++cellXId)
		{
			for (unsigned int cellYId = 0; cellYId < m_yDim; ++cellYId)
			{
				for (unsigned int cellZId = 0; cellZId < m_zDim; ++cellZId)
				{
					Cell &currCell = m_grid[linearCellCords(cellXId, cellYId, cellZId)];
					if (pointCellId != linearCellCords(cellXId, cellYId, cellZId))
					{
						if ((currCell.tag == Cell::kIN))
							innerCells.push_back(Vector(cellXId, cellYId, cellZId));
					}
				}
			}
		}

		for (unsigned int i = 0; i < iteration; ++i)
		{
			for (int j = 0; j < innerCells.size(); ++j)
			{
				
				std::map<unsigned int, double> weights;
				unsigned numCell = 1;
				Vector &cellId = innerCells[j];
				unsigned int xId = static_cast<unsigned int>(cellId.x);
				unsigned int yId = static_cast<unsigned int>(cellId.y);
				unsigned int zId = static_cast<unsigned int>(cellId.z);

				Cell &cell = m_grid[linearCellCords(xId, yId, zId)];

				if (xId > 0)
				{
					Cell &currCell = m_grid[linearCellCords(xId - 1, yId, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (xId < (m_xDim - 1))
				{
					Cell &currCell = m_grid[linearCellCords(xId + 1, yId, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				if (yId > 0)
				{
					Cell &currCell = m_grid[linearCellCords(xId, yId - 1, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (yId < (m_yDim - 1))
				{
					Cell &currCell = m_grid[linearCellCords(xId, yId + 1, zId)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				if (zId > 0)
				{
					Cell &currCell = m_grid[linearCellCords(xId, yId, zId - 1)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}


				if (zId < (m_zDim - 1))
				{
					Cell &currCell = m_grid[linearCellCords(xId, yId, zId + 1)];
					cell.weights[pp] += currCell.weights[pp];
					numCell += 1;
				}

				cell.weights[pp] = cell.weights[pp] / static_cast<double>(numCell);
			}
		}
	}	
}


void Grid::parallelSolveLaplace(const std::vector<Vector>& points, unsigned int iteration)
{ 

	//fill all the weights before execute the parallel solve
	for (unsigned int cellId = 0; cellId < m_grid.size(); ++cellId)
	{
		Cell &currCell = m_grid[cellId];
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			if (currCell.weights[i] == 0)
			{
				continue;
			}
			 
		}
	}
#ifdef MAYA
	MProgressWindow::reserve();
	MProgressWindow::setInterruptable(false);
	MString title = "Solving weights...";
	MProgressWindow::setTitle(title);
	MProgressWindow::setProgressMin(0);
	MProgressWindow::setProgressMax(points.size());
	MProgressWindow::setProgress(0);
	MProgressWindow::startProgress();
#endif
	ParallelSolver parallelData(*this, points, iteration);
	
	tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()), parallelData);
#ifdef MAYA
	MProgressWindow::endProgress();
#endif
}

const Cell& Grid::getCell(const Vector& pt)
{
	double VoxX = (pt.x - m_boundingBox.first.x) / m_cellDimension;
	int Xvox = static_cast<int>(floor(VoxX));

	double VoxY = (pt.y - m_boundingBox.first.y) / m_cellDimension;
	int Yvox = static_cast<int>(floor(VoxY));

	double VoxZ = (pt.z - m_boundingBox.first.z) / m_cellDimension;
	int Zvox = static_cast<int>(floor(VoxZ));

	return m_grid[linearCellCords(Xvox, Yvox, Zvox)];
}

std::string Grid::serialise()
{
	std::stringstream outString;

	outString << m_xDim << " " << m_yDim << " " << m_zDim << " ";

	outString << m_cellDimension << " ";

	outString << m_boundingBox.first.x << " ";
	outString << m_boundingBox.first.y << " ";
	outString << m_boundingBox.first.z << " ";

	outString << m_boundingBox.second.x << " ";
	outString << m_boundingBox.second.y << " ";
	outString << m_boundingBox.second.z << " ";

	for (unsigned int i = 0; i < m_grid.size(); ++i)
	{
		Cell &cell = m_grid[i];
		if (cell.tag == Cell::kUNDEFINED) outString << 0 << " ";
		if (cell.tag == Cell::kOUT) outString << 1 << " ";
		if (cell.tag == Cell::kIN) outString << 2 << " ";
		if (cell.tag == Cell::kBORDER) outString << 3 << " ";
		if (cell.tag == Cell::kSOURCE) outString << 4 << " ";

		unsigned int weightsCounter = 0;
		for (std::map<unsigned int, double>::iterator it = cell.weights.begin(); it != cell.weights.end(); ++it)
		{
			if (it->second > m_threshold)
				weightsCounter += 1;
		}
		outString << weightsCounter << " ";
		for (std::map<unsigned int, double>::iterator it = cell.weights.begin(); it != cell.weights.end(); ++it)
		{
			if (it->second > m_threshold)
				outString << it->first << " " << it->second << " ";
		}
	}

	return outString.str();
}

bool Grid::deserialise(const std::string& data)
{
	std::stringstream inString;
	inString << data;
	inString.exceptions(std::ios::failbit | std::ios::badbit);
	try
	{
		inString >> m_xDim;
		inString >> m_yDim;
		inString >> m_zDim;

		inString >> m_cellDimension;

		inString >> m_boundingBox.first.x;
		inString >> m_boundingBox.first.y;
		inString >> m_boundingBox.first.z;

		inString >> m_boundingBox.second.x;
		inString >> m_boundingBox.second.y;
		inString >> m_boundingBox.second.z;

		m_grid.clear();
		m_grid.resize(m_xDim*m_yDim*m_zDim);

		for (unsigned int i = 0; i < m_grid.size(); ++i)
		{
			Cell &cell = m_grid[i];
			int tag = 0;
			inString >> tag;
			if (tag == 0) cell.tag = Cell::kUNDEFINED;
			if (tag == 1) cell.tag = Cell::kOUT;
			if (tag == 2) cell.tag = Cell::kIN;
			if (tag == 3) cell.tag = Cell::kBORDER;
			if (tag == 4) cell.tag = Cell::kSOURCE;

			unsigned int wsize = 0;
			inString >> wsize;
			for (unsigned int it = 0; it < wsize; ++it)
			{
				unsigned int key = 0;
				double weight = 0.0;
				inString >> key;
				inString >> weight;
				cell.weights[key] = weight;
			}
		}
	}
	catch (...)
	{
		return false;
	}
	return true;
}


void Grid::interpWeights(double a, double b, double f, double& wOut)
{
	double f_ = 1.f - f;
	//double total = 0.f;
	wOut = a * f_ + b*f;
}


double Grid::getWeight(const Vector& pos, unsigned int p)
{
	Vector relpos = (Vector(pos - m_boundingBox.first) * (1.f / m_cellDimension)) - Vector(0.5f, 0.5f, 0.5f);

	int xi = static_cast<int>(relpos.x);
	int yi = static_cast<int>(relpos.y);
	int zi = static_cast<int>(relpos.z);

	if (relpos.x < 0.f)	--xi;
	if (relpos.y < 0.f)	--yi;
	if (relpos.z < 0.f)	--zi;

	unsigned int ccs[8] = {0,0,0,0,0,0,0,0};
	ccs[0] = linearCellCords(xi, yi, zi);
    ccs[1] = linearCellCords(xi, yi + 1, zi);
    ccs[2] = linearCellCords(xi + 1, yi + 1, zi);
    ccs[3] = linearCellCords(xi + 1, yi, zi);

    ccs[4] = linearCellCords(xi, yi, zi + 1);
    ccs[5] = linearCellCords(xi, yi + 1, zi + 1);
    ccs[6] = linearCellCords(xi + 1, yi + 1, zi + 1);
    ccs[7] = linearCellCords(xi + 1, yi, zi + 1);
	

	double w[8];
	for (unsigned int i = 0; i<8; ++i)
	{
		Cell& cell = m_grid[ccs[i]];
		w[i] = cell.weights[p];
	}

	double xf = relpos.x - xi;
	double yf = relpos.y - yi;
	double zf = relpos.z - zi;

	double w_[6];
	double wfinal;
	interpWeights(w[0], w[3], xf, w_[0]);
	interpWeights(w[1], w[2], xf, w_[1]);
	interpWeights(w[4], w[7], xf, w_[2]);
	interpWeights(w[5], w[6], xf, w_[3]);

	interpWeights(w_[0], w_[1], yf, w_[4]);
	interpWeights(w_[2], w_[3], yf, w_[5]);

	interpWeights(w_[4], w_[5], zf, wfinal);

	return wfinal;

}

std::vector<std::map<unsigned int, double> > Grid::getWeights(const std::vector<Vector>& points)
{
	std::vector<std::map<unsigned int, double> > OutWeights;
	OutWeights.resize(points.size());

	for (unsigned int v = 0; v < points.size(); ++v)
	{
		if ((points[v].x < m_boundingBox.first.x) || (points[v].x > m_boundingBox.second.x) ||
			(points[v].y < m_boundingBox.first.y) || (points[v].y > m_boundingBox.second.y) ||
			(points[v].z < m_boundingBox.first.z) || (points[v].z > m_boundingBox.second.z))
		{
			continue;
		}
		const tc::Cell& cell = getCell(points[v]);

		std::map<unsigned int, double> weights = cell.weights;
		for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
		{
			it->second = getWeight(tc::Vector(points[v]), it->first);
		}

		//normalize weights
		double totalWeight = 0.0;
		for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
			totalWeight += it->second;

		for (std::map<unsigned int, double>::iterator it = weights.begin(); it != weights.end(); ++it)
			it->second = it->second / totalWeight;

		OutWeights[v] = weights;
	}

	return OutWeights;
}