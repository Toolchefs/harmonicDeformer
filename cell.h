#pragma once

#include <map>

namespace tc
{
	class Cell
	{
	public:

		enum TYPE
		{
			kUNDEFINED = 0,
			kOUT,
			kIN,
			kBORDER,
			kSOURCE,
		};

		Cell();

		~Cell();

		std::map<unsigned int, double> weights;

		TYPE tag;
	};
}