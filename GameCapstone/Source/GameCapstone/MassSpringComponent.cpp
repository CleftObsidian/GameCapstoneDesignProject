// Fill out your copyright notice in the Description page of Project Settings.
#include "MassSpringComponent.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "StaticMeshResources.h"

// S Y S T E M //////////////////////////////////////////////////////////////////////////////////////
mass_spring_system::mass_spring_system(
	unsigned int n_points,
	unsigned int n_springs,
	float time_step,
	EdgeList spring_list,
	VectorXf rest_lengths,
	VectorXf stiffnesses,
	VectorXf masses,
	VectorXf fext,
	float damping_factor
)
	: n_points(n_points), n_springs(n_springs),
	time_step(time_step), spring_list(spring_list),
	rest_lengths(rest_lengths), stiffnesses(stiffnesses), masses(masses),
	fext(fext), damping_factor(damping_factor) {}

// S O L V E R //////////////////////////////////////////////////////////////////////////////////////
//MassSpringSolver::MassSpringSolver(TSharedPtr<mass_spring_system> system, float* vbuff)
//	:s_system(system), current_state(vbuff, system->n_points * 3),
//	prev_state(current_state), 
//	spring_directions(system->n_springs * 3)
//{
//
//	float h2 = system->time_step * system->time_step; // shorthand
//
//	// compute M, L, J
//	TripletList LTriplets, JTriplets;
//	// L
//	L.resize(3 * system->n_points, 3 * system->n_points);
//	unsigned int k = 0; // spring counter
//	for (Edge& i : system->spring_list) {
//		for (int j = 0; j < 3; j++) {
//			LTriplets.push_back(
//				Triplet(3 * i.first + j, 3 * i.first + j, 1 * system->stiffnesses[k]));
//			LTriplets.push_back(
//				Triplet(3 * i.first + j, 3 * i.second + j, -1 * system->stiffnesses[k]));
//			LTriplets.push_back(
//				Triplet(3 * i.second + j, 3 * i.first + j, -1 * system->stiffnesses[k]));
//			LTriplets.push_back(
//				Triplet(3 * i.second + j, 3 * i.second + j, 1 * system->stiffnesses[k]));
//		}
//		k++;
//	}
//	L.setFromTriplets(LTriplets.begin(), LTriplets.end());
//
//	// J
//	J.resize(3 * system->n_points, 3 * system->n_springs);
//	k = 0; // spring counter
//	for (Edge& i : system->spring_list) {
//		for (unsigned int j = 0; j < 3; j++) {
//			JTriplets.push_back(
//				Triplet(3 * i.first + j, 3 * k + j, 1 * system->stiffnesses[k]));
//			JTriplets.push_back(
//				Triplet(3 * i.second + j, 3 * k + j, -1 * system->stiffnesses[k]));
//		}
//		k++;
//	}
//	J.setFromTriplets(JTriplets.begin(), JTriplets.end());
//
//	// M
//	TripletList MTriplets;
//	M.resize(3 * system->n_points, 3 * system->n_points);
//	for (unsigned int i = 0; i < system->n_points; i++) {
//		for (int j = 0; j < 3; j++) {
//			MTriplets.push_back(Triplet(3 * i + j, 3 * i + j, system->masses[i]));
//		}
//	}
//	M.setFromTriplets(MTriplets.begin(), MTriplets.end());
//
//	// pre-factor system matrix
//	SparseMatrix A = M + h2 * L;
//	system_matrix.compute(A);
//}
//
//void MassSpringSolver::globalStep() {
//	float h2 = s_system.Get()->time_step * s_system.Get()->time_step; // shorthand
//
//	// compute right hand side
//	VectorXf b = inertial_term
//		+ h2 * J * spring_directions
//		+ h2 * s_system.Get()->fext;
//
//	current_state = system_matrix.solve(b); // Origin code
//}
//
//void MassSpringSolver::localStep() {
//	unsigned int j = 0;
//	for (Edge& i : s_system.Get()->spring_list) {
//		Vector3f p12(
//			current_state[3 * i.first + 0] - current_state[3 * i.second + 0],
//			current_state[3 * i.first + 1] - current_state[3 * i.second + 1],
//			current_state[3 * i.first + 2] - current_state[3 * i.second + 2]
//		);
//
//		p12.normalize();
//		spring_directions[3 * j + 0] = s_system.Get()->rest_lengths[j] * p12[0];
//		spring_directions[3 * j + 1] = s_system.Get()->rest_lengths[j] * p12[1];
//		spring_directions[3 * j + 2] = s_system.Get()->rest_lengths[j] * p12[2];
//		j++;
//	}
//}
//
//void MassSpringSolver::solve(unsigned int n) {
//	float a = s_system.Get()->damping_factor; // shorthand
//
//	// update inertial term
//	inertial_term = M * ((a + 1) * (current_state)- a * prev_state);
//
//	// save current state in previous state
//	prev_state = current_state;
//
//	// perform steps
//	for (unsigned int i = 0; i < n; i++) {
//		localStep();
//		globalStep();
//	}
//}
//
//void MassSpringSolver::timedSolve(unsigned int ms) {
//	// TODO
//}
//
//Map& MassSpringSolver::GetCurrentState()
//{
//	return current_state;
//}

void UMassSpringComponent::MassSpringSolver()
{
	current_state = VectorXf::Map(vbuff.Get(), m_system->n_points * 3);
	prev_state = current_state;
	spring_directions = VectorXf(m_system->n_springs * 3);

	float h2 = m_system->time_step * m_system->time_step; // shorthand

	// compute M, L, J
	TripletList LTriplets, JTriplets;
	// L
	L.resize(3 * m_system->n_points, 3 * m_system->n_points);
	unsigned int k = 0; // spring counter
	for (Edge& i : m_system->spring_list) {
		for (int j = 0; j < 3; j++) {
			LTriplets.push_back(
				Triplet(3 * i.first + j, 3 * i.first + j, 1 * m_system->stiffnesses[k]));
			LTriplets.push_back(
				Triplet(3 * i.first + j, 3 * i.second + j, -1 * m_system->stiffnesses[k]));
			LTriplets.push_back(
				Triplet(3 * i.second + j, 3 * i.first + j, -1 * m_system->stiffnesses[k]));
			LTriplets.push_back(
				Triplet(3 * i.second + j, 3 * i.second + j, 1 * m_system->stiffnesses[k]));
		}
		k++;
	}
	L.setFromTriplets(LTriplets.begin(), LTriplets.end());

	// J
	J.resize(3 * m_system->n_points, 3 * m_system->n_springs);
	k = 0; // spring counter
	for (Edge& i : m_system->spring_list) {
		for (unsigned int j = 0; j < 3; j++) {
			JTriplets.push_back(
				Triplet(3 * i.first + j, 3 * k + j, 1 * m_system->stiffnesses[k]));
			JTriplets.push_back(
				Triplet(3 * i.second + j, 3 * k + j, -1 * m_system->stiffnesses[k]));
		}
		k++;
	}
	J.setFromTriplets(JTriplets.begin(), JTriplets.end());

	// M
	TripletList MTriplets;
	M.resize(3 * m_system->n_points, 3 * m_system->n_points);
	for (unsigned int i = 0; i < m_system->n_points; i++) {
		for (int j = 0; j < 3; j++) {
			MTriplets.push_back(Triplet(3 * i + j, 3 * i + j, m_system->masses[i]));
		}
	}
	M.setFromTriplets(MTriplets.begin(), MTriplets.end());

	// pre-factor system matrix
	SparseMatrix A = M + h2 * L;
	system_matrix.compute(A);
}

void UMassSpringComponent::globalStep() {
	float h2 = m_system.Get()->time_step * m_system.Get()->time_step; // shorthand

	// compute right hand side
	VectorXf b = inertial_term
		+ h2 * J * spring_directions
		+ h2 * m_system.Get()->fext;

	current_state = system_matrix.solve(b); // Origin code
}

void UMassSpringComponent::localStep() {
	unsigned int j = 0;
	for (Edge& i : m_system.Get()->spring_list) {
		Vector3f p12(
			current_state[3 * i.first + 0] - current_state[3 * i.second + 0],
			current_state[3 * i.first + 1] - current_state[3 * i.second + 1],
			current_state[3 * i.first + 2] - current_state[3 * i.second + 2]
		);

		p12.normalize();
		spring_directions[3 * j + 0] = m_system.Get()->rest_lengths[j] * p12[0];
		spring_directions[3 * j + 1] = m_system.Get()->rest_lengths[j] * p12[1];
		spring_directions[3 * j + 2] = m_system.Get()->rest_lengths[j] * p12[2];
		j++;
	}
}

void UMassSpringComponent::solve(unsigned int n) {
	float a = m_system.Get()->damping_factor; // shorthand

	// update inertial term
	inertial_term = M * ((a + 1) * (current_state)- a * prev_state);

	// save current state in previous state
	prev_state = current_state;

	// perform steps
	for (unsigned int i = 0; i < n; i++) {
		localStep();
		globalStep();
	}
}

// B U I L D E R ////////////////////////////////////////////////////////////////////////////////////
void MassSpringBuilder::uniformGrid(
	unsigned int n,
	float time_step,
	float rest_length,
	float stiffness,
	float mass,
	float damping_factor,
	float gravity) 
{
	// n must be odd
	assert(n % 2 == 1);

	// shorthand
	const double root2 = 1.41421356237;

	// compute n_points and n_springs
	unsigned int n_points = n * n;
	unsigned int n_springs = (n - 1) * (5 * n - 2);

	// build mass list
	VectorXf masses(mass * VectorXf::Ones(n_springs));

	// build spring list and spring parameters
	EdgeList spring_list(n_springs);
	structI.reserve(2 * (n - 1) * n);
	shearI.reserve(2 * (n - 1) * (n - 1));
	bendI.reserve(n * (n - 1));

	VectorXf rest_lengths(n_springs);
	VectorXf stiffnesses(n_springs);
	unsigned int k = 0; // spring counter
	for (unsigned int i = 0; i < n; i++) {
		for (unsigned int j = 0; j < n; j++) {
			// bottom right corner
			if (i == n - 1 && j == n - 1) {
				continue;
			}

			if (i == n - 1) {
				// structural spring
				spring_list[k] = Edge(n * i + j, n * i + j + 1);
				rest_lengths[k] = rest_length;
				stiffnesses[k] = stiffness;
				structI.push_back(k++);

				// bending spring
				if (j % 2 == 0) {
					spring_list[k] = Edge(n * i + j, n * i + j + 2);
					rest_lengths[k] = 2 * rest_length;
					stiffnesses[k] = stiffness;
					bendI.push_back(k++);
				}
				continue;
			}

			// right edge
			if (j == n - 1) {
				// structural spring
				spring_list[k] = Edge(n * i + j, n * (i + 1) + j);
				rest_lengths[k] = rest_length;
				stiffnesses[k] = stiffness;
				structI.push_back(k++);

				// bending spring
				if (i % 2 == 0) {
					spring_list[k] = Edge(n * i + j, n * (i + 2) + j);
					rest_lengths[k] = 2 * rest_length;
					stiffnesses[k] = stiffness;
					bendI.push_back(k++);
				}
				continue;
			}

			// structural springs
			spring_list[k] = Edge(n * i + j, n * i + j + 1);
			rest_lengths[k] = rest_length;
			stiffnesses[k] = stiffness;
			structI.push_back(k++);

			spring_list[k] = Edge(n * i + j, n * (i + 1) + j);
			rest_lengths[k] = rest_length;
			stiffnesses[k] = stiffness;
			structI.push_back(k++);

			// shearing springs
			spring_list[k] = Edge(n * i + j, n * (i + 1) + j + 1);
			rest_lengths[k] = root2 * rest_length;
			stiffnesses[k] = stiffness;
			shearI.push_back(k++);

			spring_list[k] = Edge(n * (i + 1) + j, n * i + j + 1);
			rest_lengths[k] = root2 * rest_length;
			stiffnesses[k] = stiffness;
			shearI.push_back(k++);

			// bending springs
			if (j % 2 == 0) {
				spring_list[k] = Edge(n * i + j, n * i + j + 2);
				rest_lengths[k] = 2 * rest_length;
				stiffnesses[k] = stiffness;
				bendI.push_back(k++);
			}
			if (i % 2 == 0) {
				spring_list[k] = Edge(n * i + j, n * (i + 2) + j);
				rest_lengths[k] = 2 * rest_length;
				stiffnesses[k] = stiffness;
				bendI.push_back(k++);
			}
		}
	}

	// compute external forces
	VectorXf fext = Vector3f(0, 0, -gravity).replicate(n_points, 1);

	result = new mass_spring_system(n_points, n_springs, time_step, spring_list, rest_lengths,
		stiffnesses, masses, fext, damping_factor);
}
MassSpringBuilder::IndexList MassSpringBuilder::getStructIndex() { return structI; }
MassSpringBuilder::IndexList MassSpringBuilder::getShearIndex() { return shearI; }
MassSpringBuilder::IndexList MassSpringBuilder::getBendIndex() { return bendI; }
mass_spring_system* MassSpringBuilder::getResult() { return result; }


// C O N S T R A I N T //////////////////////////////////////////////////////////////////////////////
CgNode::CgNode(mass_spring_system* system, float* vbuff) : system(system), vbuff(vbuff) {}

// point node
CgPointNode::CgPointNode(mass_spring_system* system, float* vbuff) : CgNode(system, vbuff) {}
bool CgPointNode::accept(CgNodeVisitor& visitor) { return visitor.visit(*this); }

// spring node
CgSpringNode::CgSpringNode(mass_spring_system* system, float* vbuff) : CgNode(system, vbuff) {}
bool CgSpringNode::accept(CgNodeVisitor& visitor) {
	for (CgNode* child : children) {
		if (!child->accept(visitor)) return false;
	}
	return visitor.visit(*this);
}
void CgSpringNode::addChild(CgNode* node) { children.push_back(node); }
void CgSpringNode::removeChild(CgNode* node) {
	children.erase(find(children.begin(), children.end(), node));
}

// root node
CgRootNode::CgRootNode(mass_spring_system* system, float* vbuff) : CgSpringNode(system, vbuff) {}
void CgRootNode::satisfy() { return; }
bool CgRootNode::accept(CgNodeVisitor& visitor) {
	for (CgNode* child : children) {
		if (!child->accept(visitor)) return false;
	}
	return true;
}

// point fix node
CgPointFixNode::CgPointFixNode(mass_spring_system* system, float* vbuff)
	: CgPointNode(system, vbuff) {}
bool CgPointFixNode::query(unsigned int i) const { return fix_map.find(3 * i) != fix_map.end(); }
void CgPointFixNode::satisfy() {
	for (auto fix : fix_map)
		for (int i = 0; i < 3; i++)
			vbuff[fix.first + i] = fix.second[i];
}
void CgPointFixNode::fixPoint(unsigned int i) {
	assert(i >= 0 && i < system->n_points);
	fix_map[3 * i] = Vector3f(vbuff[3 * i], vbuff[3 * i + 1], vbuff[3 * i + 2]);
}
void CgPointFixNode::releasePoint(unsigned int i) { fix_map.erase(3 * i); }

// spring deformation node
CgSpringDeformationNode::CgSpringDeformationNode(mass_spring_system* system, float* vbuff,
	float tauc, unsigned int n_iter) : CgSpringNode(system, vbuff), tauc(tauc), n_iter(n_iter) {}
void CgSpringDeformationNode::satisfy() {
	for (int k = 0; k < int(n_iter); k++) {
		for (unsigned int i : items) {
			Edge spring = system->spring_list[i];
			CgQueryFixedPointVisitor visitor;

			Vector3f p12(
				vbuff[3 * spring.first + 0] - vbuff[3 * spring.second + 0],
				vbuff[3 * spring.first + 1] - vbuff[3 * spring.second + 1],
				vbuff[3 * spring.first + 2] - vbuff[3 * spring.second + 2]
			);

			float len = p12.norm();
			float rlen = system->rest_lengths[i];
			float diff = (len - (1 + tauc) * rlen) / len;
			float rate = (len - rlen) / rlen;

			// check deformation
			if (rate <= tauc) continue;

			// check if points are fixed
			float f1, f2;
			f1 = f2 = 0.5f;

			// if first point is fixed
			if (visitor.queryPoint(*this, spring.first)) { f1 = 0.0f; f2 = 1.0f; }

			// if second point is fixed
			if (visitor.queryPoint(*this, spring.second)) {
				f1 = (f1 != 0.0f ? 1.0f : 0.0f);
				f2 = 0.0f;
			}

			for (int j = 0; j < 3; j++) {
				vbuff[3 * spring.first + j] -= p12[j] * f1 * diff;
				vbuff[3 * spring.second + j] += p12[j] * f2 * diff;
			}
		}
	}
}
void CgSpringDeformationNode::addSprings(std::vector<unsigned int> springs) {
	items.insert(springs.begin(), springs.end());
}

// sphere collision node
CgSphereCollisionNode::CgSphereCollisionNode(mass_spring_system* system, float* vbuff,
	float radius, Vector3f center) : CgPointNode(system, vbuff), radius(radius), center(center) {}
bool CgSphereCollisionNode::query(unsigned int i) const { return false; }
void CgSphereCollisionNode::satisfy() {
	for (int i = 0; i < int(system->n_points); i++) {
		Vector3f p(
			vbuff[3 * i + 0] - center[0],
			vbuff[3 * i + 1] - center[1],
			vbuff[3 * i + 2] - center[2]
		);

		if (p.norm() < radius) {
			p.normalize();
			p = radius * p;
		}
		else continue;

		for (int j = 0; j < 3; j++) {
			vbuff[3 * i + j] = p[j] + center[j];
		}
	}
}

// node visitor
bool CgNodeVisitor::visit(CgPointNode& node) { return true; }
bool CgNodeVisitor::visit(CgSpringNode& node) { return true; }

// query fixed point visitor
bool CgQueryFixedPointVisitor::visit(CgPointNode& node) {
	queryResult = node.query(i);
	return !queryResult;
}
bool CgQueryFixedPointVisitor::queryPoint(CgNode& root, unsigned int _i) {
	this->i = _i;
	root.accept(*this);
	return queryResult;
}

// satisfy visitor
bool CgSatisfyVisitor::visit(CgPointNode& node) { node.satisfy(); return true; }
bool CgSatisfyVisitor::visit(CgSpringNode& node) { node.satisfy(); return true; }
void CgSatisfyVisitor::satisfy(CgNode& root) { root.accept(*this); }

UMassSpringComponent::UMassSpringComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = false; 
	bWantsInitializeComponent = true; // 이걸 true해야 InitializeComponent 함수가 호출됨.
	bTickInEditor = true;

	// SM Init
	m_sm = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ClothStaticMesh"));

	m_smData.vb = nullptr;
	m_smData.cvb = nullptr;
	m_smData.smvb = nullptr;
	m_smData.ib = nullptr;

	SubstepTime = 0.02f;
	At = 0.0f;

	bShowStaticMesh = true;
}

void UMassSpringComponent::OnRegister()
{
	Super::OnRegister();

	UWorld* world = GetWorld();
	if (world->IsPlayInEditor())
	{
		StaticToProcedural();
		InitCloth();
		m_sm->SetVisibility(bShowStaticMesh);
		UE_LOG(LogTemp, Error, TEXT("Mass Spring On Register"));
	}
}

void UMassSpringComponent::StaticToProcedural()
{
	UStaticMesh* sm = m_sm->GetStaticMesh();
	if (sm == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("No Static Mesh"));
		return;
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Mesh Founded"));
	}
	FStaticMeshLODResources* lod0 = *(sm->RenderData->LODResources.GetData());
	
	m_smData.vb = &(lod0->VertexBuffers.PositionVertexBuffer); // Position Vertex Buffer (Position)
	m_smData.smvb = &(lod0->VertexBuffers.StaticMeshVertexBuffer); // Static Mesh Buffer (Static Mesh)
	m_smData.cvb = &(lod0->VertexBuffers.ColorVertexBuffer); // Color Vertex Buffer (Color)
	m_smData.ib = &(lod0->IndexBuffer); // Tri Index Buffer (Index)

	m_smData.vert_count = m_smData.vb->GetNumVertices(); // Vertex Counts
	m_smData.ind_count = m_smData.ib->GetNumIndices(); // Index Counts
	m_smData.tri_count = m_smData.ind_count / 3; // Triangle Counts
	particleCount = m_smData.vert_count;
	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("%d"), m_smData.vert_count));

	UE_LOG(LogTemp, Warning, TEXT("DBG::Static Mesh Vertex Count == %d | Index Count = %d"), m_smData.vert_count, m_smData.ind_count);

	// Initialize smData Arrays
	m_smData.Pos.AddDefaulted(m_smData.vert_count);
	m_smData.Col.AddDefaulted(m_smData.vert_count);
	m_smData.Normal.AddDefaulted(m_smData.vert_count);
	m_smData.Tang.AddDefaulted(m_smData.vert_count);
	m_smData.UV.AddDefaulted(m_smData.vert_count);
	m_smData.Ind.AddDefaulted(m_smData.ind_count);
	m_smData.Tris.AddDefaulted(m_smData.tri_count);
	Particles.AddDefaulted(particleCount);

	ClearAllMeshSections(); // Delete previous mesh data

	m_smData.has_uv = m_smData.smvb->GetNumTexCoords() != 0;
	m_smData.has_col = lod0->bHasColorVertexData;

	// m_smData Buffer -> Array Deserialization
	for (int32 i = 0; i < m_smData.vert_count; ++i)
	{
		m_smData.Pos[i] = m_smData.vb->VertexPosition(i); // Pass Verts Without Component Location Offset initally.
		UE_LOG(LogClass, Log, TEXT("Index(%d) : (%s)"), i, *m_smData.vb->VertexPosition(i).ToString());

		m_smData.Normal[i] = m_smData.smvb->VertexTangentZ(i);
		m_smData.Tang[i] = FProcMeshTangent(FVector(m_smData.smvb->VertexTangentX(i).X, m_smData.smvb->VertexTangentX(i).Y, m_smData.smvb->VertexTangentX(i).Z), false);
		m_smData.has_col == true ? m_smData.Col[i] = m_smData.cvb->VertexColor(i) : m_smData.Col[i] = FColor(255, 255, 255);
		m_smData.has_uv == true ? m_smData.UV[i] = m_smData.smvb->GetVertexUV(i, 0) : m_smData.UV[i] = FVector2D(0.0f); // Only support 1 UV Channel fnow.

		// Particle Initialize
		FVector vertPtPos = GetComponentLocation() + m_smData.vb->VertexPosition(i); // Pts With Component Location Offset
		Particles[i].Position = vertPtPos;
		Particles[i].PrevPosition = vertPtPos;
		Particles[i].ID = i;
		lod0->bHasColorVertexData == true ? Particles[i].Col = m_smData.cvb->VertexColor(i) : Particles[i].Col = FColor(255, 255, 255);

	}

	for (int32 i = 0; i < m_smData.ind_count; i++)
	{
		m_smData.Ind[i] = static_cast<int32>(m_smData.ib->GetIndex(i));
	}

	// Create Cloth Mesh
	CreateMeshSection(0, m_smData.Pos, m_smData.Ind, m_smData.Normal, m_smData.UV, m_smData.Col, m_smData.Tang, false);
	bShowStaticMesh = false;
	m_sm->SetVisibility(bShowStaticMesh);
	clothStateExists = true;
}

void UMassSpringComponent::SetParticle(VectorXf& Current_Particles)
{
	// Mass Spring으로부터 Solve()된 Particle 정보(current_state)를 Get
	// 이를 Cloth의 Particles Array에 저장 (Particle Position)
	// Mesh에 Update
	int ParticleNum = 0; // Check용
	for (int32 pt = 0, i = 0; pt < particleCount; pt++, i += 3)
	{
		FClothParticle& Particle = Particles[pt];
		//VectorXf vectorXf = Current_Particles;

		FVector NewPosition = FVector(
			Current_Particles[i],
			Current_Particles[i + 1],
			Current_Particles[i + 2]);

		Particle.PrevPosition = Particle.Position;
		Particle.Position = NewPosition;
		ParticleNum += 1;
	}
	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("Particle은 %d, New ParticleNum is %d"), particleCount, ParticleNum));
}

void UMassSpringComponent::TickUpdateCloth()
{
	// Mesh를 현재 지정된 Particles의 position에 따라 Update

	check(particleCount == m_smData.vert_count);
	//GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("Mesh Update 시작")));

	// Update PM Position
	TArray<FVector> UpdtPos; UpdtPos.AddDefaulted(particleCount);
	TArray<FColor> UpdtCol; UpdtCol.AddDefaulted(particleCount);
	TArray<FProcMeshTangent> UpdtTang; UpdtTang.AddDefaulted(particleCount);
	TArray<FVector> UpdtNorm; UpdtNorm.AddDefaulted(particleCount);

	// Update Tangents from cur Particles -- 일단 Pass
	//UpdateTangents(UpdtTang, UpdtNorm);

	// Copy Normals for Pressure Use
	Normals = UpdtNorm;

	// Update From Particle Attribs. 
	if (!m_smData.has_col)
	{
		for (int32 i = 0; i < particleCount; ++i)
		{
			UpdtPos[i] = Particles[i].Position - GetComponentLocation(); // Subtract Comp Translation Off as is added to ProcMesh Verts internally. 
		}
		UpdateMeshSection(0, UpdtPos, UpdtNorm, m_smData.UV, m_smData.Col, UpdtTang); // No Colour, Use SM Colour. 
		GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, FString::Printf(TEXT("TickUpdateCloth()::particleCount은 %d"), particleCount));
	}
	else if (m_smData.has_col)
	{
		for (int32 i = 0; i < particleCount; ++i)
		{
			UpdtPos[i] = Particles[i].Position - GetComponentLocation(); // Subtract Comp Translation Off as is added to ProcMesh Verts internally. 
			UpdtCol[i] = Particles[i].Col;
		}
		UpdateMeshSection(0, UpdtPos, UpdtNorm, m_smData.UV, UpdtCol, UpdtTang); // Use Particle Colour --> Vertex Colour. 
	}
}

void UMassSpringComponent::InitCloth()
{
	// InitCloth
//static const int n = Cloth->GetVertexNum() + 1; // must be odd, n * n = n_vertices | 61
	static const int n = 7; // must be odd, n * n = n_vertices | 61
	static const float w = 400.0f; // width | 2.0f
	static const float h = 0.008f; // time step, smaller for better results | 0.008f = 0.016f/2
	static const float r = w / (n - 1) * 1.05f; // spring rest legnth
	static const float k = 1.0f; // spring stiffness | 1.0f;
	static const float m = 0.25f / (n * n); // point mass | 0.25f
	static const float a = 0.993f; // damping, close to 1.0 | 0.993f
	static const float g = 9.8f * m; // gravitational force | 9.8f

	// generate mesh -- origin code
	// MeshBuilder meshBuilder;
	// meshBuilder.uniformGrid(w, n);
	// g_clothMesh = meshBuilder.getResult();

	//-- MODIFY --//
	//-- TODO? --//
	MassSpringBuilder* massSpringBuilder = new MassSpringBuilder();
	massSpringBuilder->uniformGrid(
		n,
		h,
		r,
		k,
		m,
		a,
		g
	);
	m_system = MakeShareable(massSpringBuilder->getResult());

	// initialize mass spring solver
	// Cloth의 Position을 받아와서 PariclePos에 저장. 이를 vbuff에 할당
	// 
	//float* vbuff = (float*)&Cloth->GetVertexBuffer()->VertexPosition(0);

	float ParticlePos[147];
	for (int32 i = 0, j = 0; j < Particles.Num(); i += 3)
	{
		ParticlePos[i] = Particles[j].Position.X;
		ParticlePos[i + 1] = Particles[j].Position.Y;
		ParticlePos[i + 2] = Particles[j].Position.Z;
		j++;
	}
	vbuff = MakeShareable((float*)&ParticlePos);

	//-- TODO? --//

	// initialize mass spring solver
	MassSpringSolver();
	current_state = Map(ParticlePos, m_system->n_points * 3);
	prev_state = current_state;

	// deformation constraint parameters
	const float tauc = 0.12f; // critical spring deformation | 0.12f
	const unsigned int deformIter = 15; // number of iterations | 15

	// sphere collision constraint parameters
	const float radius = 0.64f; // sphere radius | 0.64f
	const Eigen::Vector3f center(0, 0, -1);// sphere center | (0, 0, -1)

	// initialize constraints
	// sphere collision constraint
	//CgSphereCollisionNode* sphereCollisionNode =
	//	new CgSphereCollisionNode(m_system.Get(), vbuff.Get(), radius, center);

	// spring deformation constraint
	CgSpringDeformationNode* deformationNode =
		new CgSpringDeformationNode(m_system.Get(), vbuff.Get(), tauc, deformIter);
	deformationNode->addSprings(massSpringBuilder->getShearIndex());
	deformationNode->addSprings(massSpringBuilder->getStructIndex());

	// fix top corners
	//CgPointFixNode* cornerFixer = new CgPointFixNode(m_system.Get(), vbuff.Get());
	//cornerFixer->fixPoint(0);
	//cornerFixer->fixPoint(n - 1);
	//
	// initialize user interaction
	//

	// build constraint graph
	m_cgRootNode = MakeShareable(new CgRootNode(m_system.Get(), vbuff.Get()));

	// first layer
	m_cgRootNode->addChild(deformationNode);
	//m_cgRootNode->addChild(sphereCollisionNode);

	// second layer
	//deformationNode->addChild(cornerFixer);

	UE_LOG(LogTemp, Warning, TEXT("DBG::InitCloth Mass Spring System"));
	IsInit = true;
	PrimaryComponentTick.bCanEverTick = true;
}

void UMassSpringComponent::AnimateCloth(int value)
{
	// solve two time-steps
	solve(m_iter);
	solve(m_iter);

	// fix points
	CgSatisfyVisitor visitor;
	visitor.satisfy(m_cgRootNode.ToSharedRef().Get());

	GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Yellow, FString::Printf(TEXT("AnimateCloth")));

	//-- TODO: Light에 대한 Update --//
	// update normals
	//Cloth->get
	//g_clothMesh->update_normals();
	//g_clothMesh->release_face_normals();
}

// Called every frame
void UMassSpringComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	Dt = DeltaTime;
	St = FMath::Max(SubstepTime, 0.01f); // Clamp to min substeptime. 

	if (bDoSimulate)
	{

		At += Dt;
		while (At > St)
		{
			AnimateCloth(0);
			SetParticle(current_state);
			At -= St;
		}


			TickUpdateCloth();
	}

	UWorld* world = GetWorld();
	for (int32 i = 0; i < m_smData.vert_count - 1; ++i)
	{
		DrawDebugString(world, Particles[i].Position, FString::Printf(TEXT("%d"), i), this->GetAttachmentRootActor() ,FColor::Magenta, 1.0f, true, 1.0f);
		DrawDebugSphere(world, Particles[i].Position, 2.5f, 3, FColor(0, 255, 0));
		DrawDebugLine(world, Particles[i].Position, Particles[i + 1].Position, FColor(255, 0, 0));
	}
}

void UMassSpringComponent::UpdateOnceCloth()
{
	AnimateCloth(0);
	SetParticle(current_state);
	TickUpdateCloth();

	UWorld* world = GetWorld();
	for (int32 i = 0; i < m_smData.vert_count - 1; ++i)
	{
		DrawDebugString(world, Particles[i].Position, FString::Printf(TEXT("%d"), i), this->GetAttachmentRootActor(), FColor::Magenta, 5.0f, false, 1.0f);
		DrawDebugSphere(world, Particles[i].Position, 2.5f, 3, FColor(0, 255, 0));
		DrawDebugLine(world, Particles[i].Position, Particles[i + 1].Position, FColor(255, 0, 0));
	}
}

