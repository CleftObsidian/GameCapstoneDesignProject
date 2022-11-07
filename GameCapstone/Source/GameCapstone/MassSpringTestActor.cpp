// Fill out your copyright notice in the Description page of Project Settings.


#include "MassSpringTestActor.h"
// Sets default values
AMassSpringTestActor::AMassSpringTestActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	MassSpring = CreateDefaultSubobject<UMassSpringComponent>(TEXT("MS"));
	Cloth = CreateDefaultSubobject<UClothMeshComponent>(TEXT("ClothMesh"));

}

// Called when the game starts or when spawned
void AMassSpringTestActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AMassSpringTestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	if (MassSpring->bSimulate)
	{
		Cloth->StaticToProcedural();

		static const int n = Cloth->GetVertexNum() + 1; // must be odd, n * n = n_vertices | 61
		static const float w = 2.0f; // width | 2.0f
		static const float h = 0.008f; // time step, smaller for better results | 0.008f = 0.016f/2
		static const float r = w / (n - 1) * 1.05f; // spring rest legnth
		static const float k = 1.0f; // spring stiffness | 1.0f;
		static const float m = 0.25f / (n * n); // point mass | 0.25f
		static const float a = 0.993f; // damping, close to 1.0 | 0.993f
		static const float g = 9.8f * m; // gravitational force | 9.8f

		MassSpringBuilder *massSpringBuilder = new MassSpringBuilder();
		massSpringBuilder->uniformGrid(
			n,
			h,
			r,
			k,
			m,
			a,
			g
		);

		MassSpring->system = massSpringBuilder->getResult();
		//float* vbuff = (float*)&m_Mesh->GetVertexBuffer()->VertexPosition(0);
		float* vbuff = (float*)&Cloth->GetVertexBuffer()->VertexPosition(0);
		MassSpring->m_solver = new MassSpringSolver(MassSpring->system, vbuff);
	}
}

