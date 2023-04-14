#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>

namespace Astar {
	using Path = std::vector<int>;
	
	struct Position {
		Position() = default;
		Position(int first, int second) : x{first}, y{second} {};
		int x;
		int y;
	};

	struct Node {
		Node() = default;
		Node(Position p, float f) : position{p}, fCost{f} {};
		Position position {};
		float fCost {};
		float gCost {};
		int parent {};
	};

	class AstarManager {
	public:
		AstarManager() = default;
		AstarManager(std::size_t width, std::size_t height) : mWidth{width}, mHeight{height}, mSize{width*height} {};
		Path findPath(Position source, Position goal);

	private:
		std::size_t mWidth;
		std::size_t mHeight;
		std::size_t mSize;
		const std::size_t verticalCost {10};
		const std::size_t diagonalCost {14};

		int getIndex(Position& p);
		float getHeuristicDistance(Position from, Position to);
	};
}