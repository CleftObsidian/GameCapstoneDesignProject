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
#include "ClothMeshComponent.generated.h"

/**
 * 
 */

struct FClothConstraint;

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

UCLASS()
class GAMECAPSTONE_API UClothMeshComponent : public UProceduralMeshComponent
{
	GENERATED_BODY()

private:
	typedef Eigen::VectorXf VectorXf;
	typedef Eigen::Map<Eigen::VectorXf> Map;

	// --- Cloth Data ---
	// TArray<FClothParticle> Particles; // Public
	TArray<FClothConstraint> Constraints;
	TArray<FVector> Normals;
	TArray<FClothParticle*> VolSamplePts;
	float restVolume, curVolume, deltaVolume;
	int32 particleCount;

	// --- State Flags --- 
	bool clothStateExists, world_collided;

public:
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

	// Mesh Properties
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite)
	UStaticMeshComponent* m_sm;
	UPROPERTY(EditAnywhere, Category = "Cloth Simulation")
	bool bShowStaticMesh;

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Cloth Simulation")
	void StaticToProcedural();

	// Get buffer
	FPositionVertexBuffer* GetVertexBuffer();
	FStaticMeshVertexBuffer* GetMeshVertexBuffer();
	FColorVertexBuffer* GetColorVertexBuffer();
	FRawStaticIndexBuffer* GetIndexBuffer();

	unsigned int GetVertexNum();

	unsigned int GetVBuffLen();
	unsigned int GetNBuffLen();
	unsigned int GetTBuffLen();
	unsigned int GetIBuffLen();

	TArray<FClothParticle> GetParticle();
	void SetParticle(Map& Current_Particles);

	UClothMeshComponent(const FObjectInitializer& ObjectInitializer);

	void OnRegister() override;

	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
	// --- Cloth Solver Methods ---
	void TickUpdateCloth();

	//Map Current_Particles;

};

struct FClothConstraint
{
	FClothConstraint(FClothParticle& Pt_0, FClothParticle& Pt_1, UClothMeshComponent* cloth)
		: Pt0(Pt_0), Pt1(Pt_1), Cloth(cloth)
	{
		// Get Particles Corresponding Vertices Orginal Postions and Rest Length.
		orgP0 = Cloth->m_smData.Pos[Pt_0.ID]; orgP1 = Cloth->m_smData.Pos[Pt_1.ID];
		restLength = (orgP1 - orgP0).Size();
		// ID to Identify Particle/Vertex ID Pair of Constraint. 
		conID = Pt_0.ID * Pt_1.ID;
	}
	FClothConstraint() = delete;

	FClothParticle& Pt0, & Pt1;
	FVector orgP0, orgP1;
	float restLength;
	int32 conID;
	UClothMeshComponent* Cloth;
};