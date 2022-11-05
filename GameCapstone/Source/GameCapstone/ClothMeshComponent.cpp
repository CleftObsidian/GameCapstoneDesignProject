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