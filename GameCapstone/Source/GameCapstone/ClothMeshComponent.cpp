// Fill out your copyright notice in the Description page of Project Settings.


#include "ClothMeshComponent.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Math/RandomStream.h"
#include "WorldCollision.h"
#include "Engine/CollisionProfile.h"
#include "Stats/Stats.h"
#include "StaticMeshResources.h"


UClothMeshComponent::UClothMeshComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	bTickInEditor = true;

	m_sm = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ClothStaticMesh"));
	m_smData.vb = nullptr;
	m_smData.cvb = nullptr;
	m_smData.smvb = nullptr;
	m_smData.ib = nullptr;

	bShowStaticMesh = true;
}

void UClothMeshComponent::OnRegister()
{
	Super::OnRegister();
	UWorld* world = GetWorld();
	if (world->IsPlayInEditor())
	{
		StaticToProcedural();
	}

	m_sm->SetVisibility(bShowStaticMesh);
}

void UClothMeshComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	UWorld* world = GetWorld();
	for (int32 i = 0; i < m_smData.vert_count - 1; ++i)
	{
		DrawDebugSphere(world, Particles[i].Position, 2.5f, 3, FColor(0, 255, 0));
		DrawDebugLine(world, Particles[i].Position, Particles[i + 1].Position, FColor(255, 0, 0));
	}
}

void UClothMeshComponent::StaticToProcedural()
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

TArray<FClothParticle> UClothMeshComponent::GetParticle()
{
	return Particles;
}


void UClothMeshComponent::SetParticle(VectorXf& Current_Particles)
{
	// Mass Spring으로부터 Solve()된 Particle 정보(current_state)를 Get
	// 이를 Cloth의 Particles Array에 저장 (Particle Position)
	// Mesh에 Update
	for (int32 pt = 0; pt < particleCount; pt++)
	{
		FClothParticle& Particle = Particles[pt];
		//VectorXf vectorXf = Current_Particles;

		FVector NewPosition;
		int ParticleNum= 0; // Check용
		for (int32 i = pt; i < Current_Particles.size() - 2; i+=3)
		{
			NewPosition = FVector(
				Current_Particles[i], 
				Current_Particles[i + 1], 
				Current_Particles[i + 2]);

			Particle.PrevPosition = Particle.Position;
			Particle.Position = NewPosition;
			ParticleNum += 1;
		}
		GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("Particle은 %d, 생성 ParticleNum은 %d"),particleCount, ParticleNum));
	}
}

FPositionVertexBuffer* UClothMeshComponent::GetVertexBuffer()
{
	return m_smData.vb;
}

FStaticMeshVertexBuffer* UClothMeshComponent::GetMeshVertexBuffer()
{
	return m_smData.smvb;
}

FColorVertexBuffer* UClothMeshComponent::GetColorVertexBuffer()
{
	return m_smData.cvb;
}

FRawStaticIndexBuffer* UClothMeshComponent::GetIndexBuffer()
{
	return m_smData.ib;
}

unsigned int UClothMeshComponent::GetVertexNum()
{
	return (unsigned int)m_smData.vert_count;
}

unsigned int UClothMeshComponent::GetVBuffLen()
{
	return (unsigned int)m_smData.vert_count * 3;
}

unsigned int UClothMeshComponent::GetNBuffLen()
{
	return (unsigned int)m_smData.vert_count * 3;
}
unsigned int UClothMeshComponent::GetTBuffLen()
{
	return (unsigned int)m_smData.vert_count * 2;
}
unsigned int UClothMeshComponent::GetIBuffLen()
{
	return (unsigned int)m_smData.ind_count;
}

void UClothMeshComponent::TickUpdateCloth()
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