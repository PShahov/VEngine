#include "Octree.h"

//unsigned int Octree::FloatsPerOctree = 12;
//float Octree::InitialSize = 32.0f;

Vector3 Octree::DIAGONAL_DIRECTIONS[8] =
{
	Vector3(-1,1,-1), //left top far
	Vector3(1,1,-1), //right top far
	Vector3(1,1,1), //right top near
	Vector3(-1,1,1), //left top near

	Vector3(-1,-1,-1), //left bottom far
	Vector3(1,-1,-1), // right bottom far
	Vector3(1,-1,1), //right bottom near
	Vector3(-1,-1,1), //left bottom near
};

int Octree::octCreated = 0;
int Octree::octDeleted = 0;


bool Octree::operator==(Octree b) {
	if (!IsLeaf() || !b.IsLeaf()) return false;
	if (GetState(VoxelStateIndex::FillState) == true || b.GetState(VoxelStateIndex::FillState) == true) {
		if (State != b.State) return false;
		if (Color != b.Color) return false;
	}

	return true;
}
bool Octree::operator!=(Octree b) {
	return !(*this == b);
}

Octree::Octree(int index, Vector3 position, VoxelColor color, char state, Octree* parent)
{
	Index = index;
	Position = position;
	Color = color;
	State = state;
	Parent = parent;
	octCreated++;
}

Octree::~Octree()
{
	octDeleted++;
	RemoveLeafs();
}

bool Octree::IsLeaf()
{
	return !GetState(VoxelStateIndex::Divided);
}

bool Octree::IsRoot()
{
	return Index == 0;
}
bool Octree::GetState(char index) {
	return (State & 1 << (int)index) != 0;
}
void Octree::SetState(bool value, char index) {
	if (value)
		State = (char)(State | 1 << (int)index);
	else
		State = (char)(State & ~(1 << (int)index));
}
float Octree::GetNodeSize() {
	return Octree::GetSizeByIndex(Index);
}
float Octree::GetSizeByIndex(int index) {
	return INITIAL_SIZE / (std::powf(2, index));
}
int Octree::GetOctreeSize() {
	int count = 1;
	if (IsLeaf()) return count;
	for (int i = 0; i < 8; i++)
		count += Leafs[i]->GetOctreeSize();

	return count;
}
int Octree::GetLowestIndex() {
	int lIndex = Index;

	if (IsLeaf()) return lIndex;
	for (int i = 0; i < 8; i++) {
		int cIndex = Leafs[i]->GetLowestIndex();
		if (lIndex < cIndex) lIndex = cIndex;
	}

	return lIndex;
}
int Octree::GetLeafsCount() {
	int count = 0;
	if (IsLeaf()) return 1;
	for (int i = 0; i < 8; i++)
		count += Leafs[i]->GetLeafsCount();

	return count;
}

void Octree::OptimizeToTop() {
	Octree::OptimizeToTop(this);
}
void Octree::DivideLowestLeafs()
{
	if (!IsLeaf())
		for (int i = 0; i < 8; i++)
			Leafs[i]->DivideLowestLeafs();
	else
		DivideNode();
}
void Octree::OptimizeToTop(Octree* oct) {

	if (oct->IsLeaf()) return;
	for (int i = 1; i < 8; i++) {
		if (oct->Leafs[0] != oct->Leafs[i]) return;
	}

	oct->SetState(false, VoxelStateIndex::Divided);
	oct->SetState(oct->Leafs[0]->GetState(VoxelStateIndex::FillState), VoxelStateIndex::FillState);
	oct->Color = oct->Leafs[0]->Color;

	oct->RemoveLeafs();

	if (!oct->IsRoot()) oct->Parent->OptimizeToTop();
}

void Octree::OptimizeToBottom() {
	Octree::OptimizeToBottom(this);
}
void Octree::OptimizeToBottom(Octree* oct) {

	if (oct->IsLeaf()) return;

	for (int i = 0; i < 8; i++) {
		oct->Leafs[i]->OptimizeToBottom();
	}

	for (int i = 1; i < 8; i++) {
		Octree a = *oct->Leafs[0];
		Octree b = *oct->Leafs[i];
		if (a != b) return;
	}

	oct->SetState(false, VoxelStateIndex::Divided);
	oct->SetState(oct->Leafs[0]->GetState(VoxelStateIndex::FillState), VoxelStateIndex::FillState);
	oct->Color = oct->Leafs[0]->Color;

	oct->RemoveLeafs();
}

bool Octree::DivideNode() {
	if (!IsLeaf() || Index >= MAX_INDEX) return false;

	float offset = GetSizeByIndex(Index + 2);

	for (int i = 0; i < 8; i++) {
		Vector3 pos = Position + (DIAGONAL_DIRECTIONS[i] * offset);
		Leafs[i] = new Octree(Index + 1, pos, Color, State, this);
	}
	SetState(true, VoxelStateIndex::Divided);

	return true;
}

void Octree::RemoveLeafs()
{
	if (!IsLeaf()) {
		for (int i = 0; i << 8; i++) {
			delete Leafs[i];
		}
	}
}

void Octree::AverageColor() {
	if (IsLeaf()) return;

	int filledLeafs = 0;
	int r = 0, g = 0, b = 0, a = 0;

	for (int i = 0; i < 8; i++) {
		if (Leafs[i]->IsLeaf() == false) Leafs[i]->AverageColor();
		if (Leafs[i]->GetState(VoxelStateIndex::FillState) && (int(Leafs[i]->Color.Color[3])) > 0) {
			filledLeafs++;
			r += Leafs[i]->Color.Color[0];
			g += Leafs[i]->Color.Color[1];
			b += Leafs[i]->Color.Color[2];
			a += Leafs[i]->Color.Color[3];
		}
	}

	if (filledLeafs >= 4) {
		r = std::min(255, r / filledLeafs);
		g = std::min(255, g / filledLeafs);
		b = std::min(255, b / filledLeafs);
		a = std::min(255, a / filledLeafs);

		Color = VoxelColor(r, g, b, a);
	}
	else {
		Color = VoxelColor(255, 0, 0, 0);
	}
}
void Octree::AverageFillState() {
	if (IsLeaf()) return;
	int filledLeafs = 0;
	for (int i = 0; i < 8; i++) {
		if (Leafs[i]->IsLeaf() == false)
			Leafs[i]->AverageFillState();
		if (Leafs[i]->GetState(VoxelStateIndex::FillState))
			filledLeafs++;
	}
	SetState(filledLeafs >= 4, VoxelStateIndex::FillState);
}

bool Octree::IsPointInside(Vector3 point) {
	/*point.Abs();
	Vector3 corner = Vector3(GetNodeSize() / 2);

	return point.x <= corner.x && point.y <= corner.y && point.z <= corner.z;*/

	Vector3 side = (Vector3(1, 1, 1) * (GetNodeSize() / 2));
	Vector3 minCorn = Position - side;
	Vector3 maxCorn = Position + side;
	return point.x <= maxCorn.x && point.x >= minCorn.x && point.y <= maxCorn.y && point.y >= minCorn.y && point.z <= maxCorn.z && point.z >= minCorn.z;
}

Octree* Octree::OctreeWithPointInside(Vector3 point) {
	if (IsPointInside(point) == false) return NULL;

	if (IsLeaf()) return this;

	int childNodeIndex = 0;
	Vector3 relativePoint = (point - Position);


	if (relativePoint.y < 0) childNodeIndex += 4;   // if -
	if (relativePoint.z > 0) {
		childNodeIndex += 2;   // if +
	}
	else {
		relativePoint.x *= -1;
	}
	if (relativePoint.x < 0) childNodeIndex += 1;   // if -

	return Leafs[childNodeIndex]->OctreeWithPointInside(point);
}

void Octree::OctreeToArray(float* data, int ptr) {
	int octreeSize = GetOctreeSize();

	data[ptr] = *(float*)&octreeSize;
	data[ptr + 1] = GetOctreeData();
	data[ptr + 2] = Color.ToFloat();

	if (!IsLeaf()) {

		int floatForSkip = 0;
		for (int i = 0; i < 8; i++) {
			int s = Leafs[i]->GetOctreeSize();
			data[ptr + 4 + i] = *(float*)&floatForSkip;
			floatForSkip += s;
		}

		ptr += Octree::FLOAT_PER_OCTREE;

		for (int i = 0; i < 8; i++) {
			Leafs[i]->OctreeToArray(data, ptr);
			ptr += Leafs[i]->GetOctreeSize() * Octree::FLOAT_PER_OCTREE;
		}

	}
}

float Octree::GetOctreeData(char nodeRelativePositionIndex) {
	return VoxelColor(State, Index, IsLeaf() ? 255 : 0, nodeRelativePositionIndex).ToFloat();
}