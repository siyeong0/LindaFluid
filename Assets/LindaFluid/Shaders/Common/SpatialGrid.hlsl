#ifndef SPATIAL_GRID_INCLUDE
#define SPATIAL_GRID_INCLUDE

static const int2 SPATIAL_OFFSETS[9] =
{
	int2(-1, 1),
	int2(0, 1),
	int2(1, 1),
	int2(-1, 0),
	int2(0, 0),
	int2(1, 0),
	int2(-1, -1),
	int2(0, -1),
	int2(1, -1),
};

static const uint HASH_K1 = 15823;
static const uint HASH_K2 = 9737333;

int2 getCellCoordFromPosition(float2 position, float radius)
{
	return (int2) floor(position / radius);
}

uint getHashFromCellPos(int2 cell)
{
	cell = (uint2) cell;
	return (cell.x * HASH_K1 + cell.y * HASH_K2);
}

uint getKeyFromHash(uint hash, uint tableSize)
{
	return hash % tableSize;
}

#endif