#include "AstarManager.h"

namespace Astar {
	constexpr bool operator > (const Node& first, const Node& second) {
		return first.fCost > second.fCost;
	}

	int AstarManager::getIndex(Position& p) {
		return p.x + p.y * mWidth;
	}

	float AstarManager::getHeuristicDistance(Position from, Position to) {
		float dx = abs(from.x - to.x);
		float dy = abs(from.y - to.y);
		return verticalCost * (dx + dy) + (diagonalCost - 2 * verticalCost) * std::min(dx, dy);
	}

	Path AstarManager::findPath(Position source, Position goal) {
		static int debugIt = 0;
		
		std::vector<Node> allNodes;
		allNodes.resize(mSize);
		for(int i = 0; i < mSize; i++) {
			int x = i % mWidth;
			int y = i / mWidth;
			Node* node = &allNodes.at(x + y * mWidth);
			node->position.x = x;
			node->position.y = y;
		}

		std::vector<bool> isClosed;
		isClosed.resize(mSize);

		std::vector<Node> openNodes;
		openNodes.emplace_back(Position{source.x, source.y}, 0);

		while(!openNodes.empty()) {
			debugIt++;

			std::pop_heap(openNodes.begin(), openNodes.end(), std::greater<>{});
			Position* p = &openNodes.back().position;
			Node* current = &allNodes.at(getIndex(*p));

			if(getIndex(current->position) == getIndex(goal)) {
				std::cout << "Found path to goal after " << debugIt << " iterations\n";
				return {};
			}

			isClosed.at(getIndex(current->position)) = true;
			openNodes.pop_back();
			
			for(int y = -1; y <= 1; y++) {
				for(int x = -1; x <= 1; x++) {
					if(x == 0 && y == 0) continue;

					Position nPos {current->position.x + x, current->position.y + y};

					if(nPos.x < 0 || nPos.y < 0 || nPos.x >= mWidth || nPos.y >= mHeight) continue;

					Node* neighbour = &allNodes.at(getIndex(nPos));

					bool isDiagonal = x != 0 && y != 0;

					float calcGCost = current->gCost + (isDiagonal ? diagonalCost : verticalCost);

					bool open {};
					bool closed {isClosed.at(getIndex(neighbour->position))};

					auto foundOpen = std::find_if(openNodes.begin(), openNodes.end(), [&](const Node& n){
						return neighbour->position.x == n.position.x && neighbour->position.y == n.position.y;
					});

					open = foundOpen != openNodes.end();

					if(open && calcGCost < neighbour->gCost) {
						openNodes.erase(foundOpen);
						open = false;
					}

					if(closed && calcGCost < neighbour->gCost) {
						isClosed.at(getIndex(neighbour->position)) = false;
						closed = false;
					}

					if(!open && !closed) {
						neighbour->gCost = calcGCost;
						neighbour->fCost = neighbour->gCost + getHeuristicDistance(neighbour->position, goal);
						neighbour->parent = getIndex(current->position);
						openNodes.emplace_back(neighbour->position, neighbour->fCost);
						std::push_heap(openNodes.begin(), openNodes.end(), std::greater<>{});
					}
				}
			}
		}

		std::cout << "Didn't find path to goal after " << debugIt << " iterations\n";
		return {};
	}
}