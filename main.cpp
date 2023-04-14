#include "AstarManager.h"

using Position = Astar::Position;

int main() {
	Astar::AstarManager astarManager(100, 100);
	astarManager.findPath(Position(5, 5), Position(95, 95));

	return 0;
}