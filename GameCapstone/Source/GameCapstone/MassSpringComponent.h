// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ThirdParty\Eigen\Dense"
#include "ThirdParty\Eigen\Sparse"
#include "ThirdParty\Eigen\src\SparseCore\SparseMatrix.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "MassSpringComponent.generated.h"

// Mass-spring Ststem struct
struct mass_spring_system
{
	typedef Eigen::SparseMatrix<float> SparseMatrix;
	typedef Eigen::VectorXf VectorXf;
	typedef std::pair<unsigned int, unsigned int> Edge;
	typedef std::vector<Edge> EdgeList;

	// parameters
	unsigned int n_points; // number of points
	unsigned int n_springs; // number of springs
	float time_step; // time step
	EdgeList spring_list; // spring edge list
	VectorXf rest_lengths; // spring rest lengths
	VectorXf stiffnesses; // spring stiffnesses
	VectorXf masses; // point masses
	VectorXf fext; // external forces
	float damping_factor; // damping factor

	mass_spring_system(
		unsigned int n_points,       // number of points
		unsigned int n_springs,      // number of springs
		float time_step,             // time step
		EdgeList spring_list,        // spring edge list
		VectorXf rest_lengths,       // spring rest lengths
		VectorXf stiffnesses,        // spring stiffnesses
		VectorXf masses,             // point masses
		VectorXf fext,               // external forces
		float damping_factor         // damping factor
	);
};

typedef Eigen::Map<Eigen::VectorXf> Map;

// Mass-Spring System Solver class
//class MassSpringSolver {
//private:
//	typedef Eigen::Vector3f Vector3f;
//	typedef Eigen::VectorXf VectorXf;
//	typedef Eigen::SparseMatrix<float> SparseMatrix;
//	typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > Cholesky;
//	typedef Eigen::Map<Eigen::VectorXf> Map;
//	typedef std::pair<unsigned int, unsigned int> Edge;
//	typedef Eigen::Triplet<float> Triplet;
//	typedef std::vector<Triplet> TripletList;
//
//	// system
//	TSharedPtr<mass_spring_system> s_system;
//	Cholesky system_matrix;
//
//	// M, L, J matrices
//	SparseMatrix M;
//	SparseMatrix L;
//	SparseMatrix J;
//
//	// state
//	// Map current_state; // q(n), current state -- Public
//	//VectorXf prev_state; // q(n - 1), previous state -- Public
//	VectorXf spring_directions; // d, spring directions 
//	VectorXf inertial_term; // M * y, y = (a + 1) * q(n) - a * q(n - 1)
//
//	// steps
//	void globalStep();
//	void localStep();
//
//public:
//	MassSpringSolver(TSharedPtr<mass_spring_system> system, float* vbuff);
//
//	Map current_state; // q(n), current state
//	VectorXf prev_state; // q(n - 1), previous state
//	// solve iterations
//	void solve(unsigned int n);
//	void timedSolve(unsigned int ms);
//
//	Map& GetCurrentState();
//};


// Mass-Spring System Builder Class
class MassSpringBuilder
{
private:
	typedef Eigen::Vector3f Vector3f;
	typedef Eigen::VectorXf VectorXf;
	typedef std::pair<unsigned int, unsigned int> Edge;
	typedef std::vector<Edge> EdgeList;
	typedef Eigen::Triplet<float> Triplet;
	typedef std::vector<Triplet> TripletList;
	typedef std::vector<unsigned int> IndexList;

	IndexList structI, shearI, bendI;
	mass_spring_system* result;

public:
	void uniformGrid(
		unsigned int n,          // grid width
		float time_step,         // time step
		float rest_length,       // spring rest length (non-diagonal)
		float stiffness,         // spring stiffness
		float mass,              // node mass
		float damping_factor,    // damping factor
		float gravity            // gravitationl force (-z axis)
	);


	// indices
	IndexList getStructIndex(); // structural springs
	IndexList getShearIndex(); // shearing springs
	IndexList getBendIndex(); // bending springs

	mass_spring_system* getResult();
};

// Constraint Graph
class CgNodeVisitor; // Constraint graph node visitor

// Constraint graph node
class CgNode {
protected:
	mass_spring_system* system;
	float* vbuff;

public:
	CgNode(mass_spring_system* system, float* vbuff);

	virtual void satisfy() = 0; // satisfy constraint
	virtual bool accept(CgNodeVisitor& visitor) = 0; // accept visitor
};

// point constraint node
class CgPointNode : public CgNode {
public:
	CgPointNode(mass_spring_system* system, float* vbuff);
	virtual bool query(unsigned int i) const = 0; // check if point with index i is constrained
	virtual bool accept(CgNodeVisitor& visitor);
};

// spring constraint node
class CgSpringNode : public CgNode {
protected:
	typedef std::vector<CgNode*> NodeList;
	NodeList children;

public:
	CgSpringNode(mass_spring_system* system, float* vbuff);

	virtual bool accept(CgNodeVisitor& visitor);
	void addChild(CgNode* node);
	void removeChild(CgNode* node);
};

// root node 
class CgRootNode : public CgSpringNode {
public:
	CgRootNode(mass_spring_system* system, float* vbuff);

	virtual void satisfy();
	virtual bool accept(CgNodeVisitor& visitor);
};

class CgPointFixNode : public CgPointNode {
protected:
	typedef Eigen::Vector3f Vector3f;
	std::unordered_map<unsigned int, Vector3f> fix_map;
public:
	CgPointFixNode(mass_spring_system* system, float* vbuff);
	virtual void satisfy();

	virtual bool query(unsigned int i) const;
	virtual void fixPoint(unsigned int i); // add point at index i to list
	virtual void releasePoint(unsigned int i); // remove point at index i from list
};

// spring node
class CgSpringDeformationNode : public CgSpringNode {
private:
	typedef std::pair<unsigned int, unsigned int> Edge;
	typedef Eigen::Vector3f Vector3f;
	std::unordered_set<unsigned int> items;
	float tauc; // critical deformation rate
	unsigned int n_iter; // number of iterations

public:
	CgSpringDeformationNode(mass_spring_system* system, float* vbuff, float tauc, unsigned int n_iter);
	virtual void satisfy();

	void addSprings(std::vector<unsigned int> springs);
};

// sphere collision node
class CgSphereCollisionNode : public CgPointNode {
private:
	typedef Eigen::Vector3f Vector3f;

	float radius;
	Vector3f center;

public:
	CgSphereCollisionNode(mass_spring_system* system, float* vbuff, float radius, Vector3f center);
	virtual bool query(unsigned int i) const;
	virtual void satisfy();
};

// node visitor
class CgNodeVisitor {
public:

	virtual bool visit(CgPointNode& node);
	virtual bool visit(CgSpringNode& node);
};

// fixed point query visitor
class CgQueryFixedPointVisitor : public CgNodeVisitor {
private:
	unsigned int i;
	bool queryResult;
public:
	virtual bool visit(CgPointNode& node);

	bool queryPoint(CgNode& root, unsigned int i);
};

// satisfy visitor
class CgSatisfyVisitor : public CgNodeVisitor {
public:
	virtual bool visit(CgPointNode& node);
	virtual bool visit(CgSpringNode& node);

	void satisfy(CgNode& root);
};

struct FClothParticle
{
	FClothParticle()
		: Position(0, 0, 0)
		, PrevPosition(0, 0, 0)
		, Force(0, 0, 0)
		, Col(255, 255, 255, 255)
		, ID(-1)
		, C_idx(-1)
		, state(1)
		, conCount(0)
	{}
	FVector Position;
	FVector PrevPosition;
	FVector Force;
	FColor Col;

	int32 ID;
	uint32 C_idx;
	int8 state, conCount;
};

// Mass-Spring System
UCLASS(hidecategories = (Object, Physics, Activation, Collision, Navigation, Transform, "Components|Activation"), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering)
class GAMECAPSTONE_API UMassSpringComponent : public UProceduralMeshComponent
{
	GENERATED_BODY()

private:
	typedef Eigen::Vector3f Vector3f;
	typedef Eigen::VectorXf VectorXf;
	typedef Eigen::SparseMatrix<float> SparseMatrix;
	typedef Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > Cholesky;
	typedef Eigen::Map<Eigen::VectorXf> Map;
	typedef std::pair<unsigned int, unsigned int> Edge;
	typedef Eigen::Triplet<float> Triplet;
	typedef std::vector<Triplet> TripletList;

	// -- Mass Spring Solver -- //
	Cholesky system_matrix;
	// M, L, J matrices
	SparseMatrix M;
	SparseMatrix L;
	SparseMatrix J;
	// state
	// Map current_state; // q(n), current state -- Public
	//VectorXf prev_state; // q(n - 1), previous state -- Public
	VectorXf spring_directions; // d, spring directions 
	VectorXf inertial_term; // M * y, y = (a + 1) * q(n) - a * q(n - 1)
	// steps
	void globalStep();
	void localStep();

	// --- Cloth Data ---
	// TArray<FClothParticle> Particles; // Public
	TArray<FVector> Normals;
	TArray<FClothParticle*> VolSamplePts;
	float restVolume, curVolume, deltaVolume;
	int32 particleCount;

	// --- State Flags --- 
	bool clothStateExists, world_collided;

public:
	TSharedPtr<mass_spring_system> m_system;
	//TSharedPtr<MassSpringSolver> m_solver;

	float arr[5] = { 0,0,0,0,0 };
	float* ptr = arr;
	TSharedPtr<float> vbuff;

	//Map current_state; // q(n), current state
	VectorXf current_state;
	VectorXf prev_state; // q(n - 1), previous state

	// Constraint Graph
	CgRootNode* m_cgRootNode;
	TArray<FClothParticle> Particles;
	struct
	{
		// SM Deserialized
		TArray<FVector>            Pos;
		TArray<FColor>             Col;
		TArray<FVector>            Normal;
		TArray<FProcMeshTangent>   Tang;
		TArray<FVector2D>          UV;
		TArray<int32>              Ind;
		TArray<FIntVector>         Tris;

		// Vert Shared Tris
		TArray<int32>* vtris;

		// SM Buffer Ptrs
		FPositionVertexBuffer* vb;
		FStaticMeshVertexBuffer* smvb;
		FColorVertexBuffer* cvb;
		FRawStaticIndexBuffer* ib;

		int32 vert_count, ind_count, adj_count, tri_count;
		bool has_uv, has_col;
	} m_smData;

	// -- Mesh Properties
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
		UStaticMeshComponent* m_sm;
	UPROPERTY(EditAnywhere, Category = "Cloth Simulation")
		bool bShowStaticMesh;
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Cloth Simulation")
		void StaticToProcedural();
	void OnRegister() override;
	void TickUpdateCloth();

	// -- Mass Spring
	// Sets default values for this component's properties		
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cloth Simulation")
		bool bDoSimulate = false;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Simulation", meta = (ClampMin = "0.005", UIMin = "0.005", UIMax = "0.1"))
		float SubstepTime;

	UMassSpringComponent(const FObjectInitializer& ObjectInitializer);
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Cloth Simulation")
		void InitCloth();

	static const int m_iter = 1; // iterations per time step | 10
	float At = 0.0f, Dt, St;
	bool IsInit = false; // √ ±‚»≠

	void SetParticle(VectorXf& Current_Particles);
	
	// --- Cloth Solver Methods ---
	void MassSpringSolver();
	// solve iterations
	void solve(unsigned int n);
	void AnimateCloth(int value);

	

	// --- UActorComponent Overrides ---
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;



	

	

protected:
	virtual void BeginPlay() override;
};