#pragma once

#include <vector>
#include <map>
#include <tbb/blocked_range.h>
#include <tbb/queuing_mutex.h>
#include "mathUtils.h"
#include "intersect.h"
#ifdef MAYA
#include <maya/MProgressWindow.h>
#include <maya/MString.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

namespace tc
{
	class Cell;

	class Grid;

	class ParallelSolver
	{
	public:
		ParallelSolver(Grid& g, const std::vector<Vector>& pts,
			unsigned int iter
			) : grid(g), points(pts), iteration(iter) 
{}



		~ParallelSolver(){}

		void operator()(const tbb::blocked_range<size_t>& range) const;

	private:
		Grid& grid;

		const std::vector<Vector>& points;

		unsigned int iteration;

#ifdef MAYA
		static tbb::queuing_mutex m_mutex;
#endif
	};

	class Grid
	{

	public:

		Grid();

		Grid(double cellDimension);

		~Grid();

		inline void setThreshold(double value) { m_threshold = value; }

		bool addBoundary(const std::vector<Vector>& points, const std::vector<unsigned int>& faceVtx, const std::vector<unsigned int> numVtxPerFace);

		void solveLaplace(const std::vector<Vector>& points, unsigned int iteration = 50);

		void parallelSolveLaplace(const std::vector<Vector>& points, unsigned int iteration = 50);

		const Cell& getCell(const Vector& pt);

		double getWeight(const Vector& pt, unsigned int p);

		std::vector<std::map<unsigned int, double> > getWeights(const std::vector<Vector>& points);

		std::string serialise();

		bool deserialise(const std::string& data);

	public:

		int linearCellCords(unsigned int x, unsigned int y, unsigned int z);

		void interpWeights(double a, double b, double f, double& wOut);

	public:

		std::vector<Cell> m_grid;

		double m_cellDimension;

		std::pair<Vector, Vector> m_boundingBox;

		unsigned int m_xDim;

		unsigned int m_yDim;

		unsigned int m_zDim;

		friend class ParallelSolver;

		double m_threshold;

	};

}