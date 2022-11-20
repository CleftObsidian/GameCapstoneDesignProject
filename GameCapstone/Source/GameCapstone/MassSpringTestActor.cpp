// Fill out your copyright notice in the Description page of Project Settings.


#include "MassSpringTestActor.h"


// Sets default values
AMassSpringTestActor::AMassSpringTestActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	MassSpring = CreateDefaultSubobject<UMassSpringComponent>(TEXT("MS"));
	Cloth = CreateDefaultSubobject<UClothMeshComponent>(TEXT("ClothMesh"));

	//solved_current_state = MassSpring->m_solver->GetCurrentState();

	bStartSimulate = false;
	SubstepTime = 0.02f;
	At = 0.0f;
}

void AMassSpringTestActor::InitCloth()
{
	//Cloth->StaticToProcedural();

	//static const int n = Cloth->GetVertexNum() + 1; // must be odd, n * n = n_vertices | 61
	static const int n = 7; // must be odd, n * n = n_vertices | 61
	static const float w = 2.0f; // width | 2.0f
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
	MassSpring->system = massSpringBuilder->getResult();

	// initialize mass spring solver
	// Cloth의 Position을 받아와서 PariclePos에 저장. 이를 vbuff에 할당
	// 
	//float* vbuff = (float*)&Cloth->GetVertexBuffer()->VertexPosition(0);

	float ParticlePos[147];
	for (int32 i = 0,  j = 0; j < Cloth->GetParticle().Num(); i += 3)
	{
		ParticlePos[i] = Cloth->GetParticle()[j].Position.X;
		ParticlePos[i + 1] = Cloth->GetParticle()[j].Position.Y;
		ParticlePos[i + 2] = Cloth->GetParticle()[j].Position.Z;
		j++;
	}
	float* vbuff = (float*) &ParticlePos;

	//-- TODO? --//

	// initialize mass spring solver
	MassSpring->m_solver = new MassSpringSolver(MassSpring->system, vbuff);
	//MassSpring->m_solver->current_state = Map(ParticlePos, MassSpring->system->n_points * 3);
	MassSpring->m_solver->prev_state = MassSpring->m_solver->current_state;

	// deformation constraint parameters
	const float tauc = 0.12f; // critical spring deformation | 0.12f
	const unsigned int deformIter = 15; // number of iterations | 15

	// sphere collision constraint parameters
	const float radius = 0.64f; // sphere radius | 0.64f
	const Eigen::Vector3f center(0, 0, -1);// sphere center | (0, 0, -1)

	// initialize constraints
	// sphere collision constraint
	CgSphereCollisionNode* sphereCollisionNode =
		new CgSphereCollisionNode(MassSpring->system, vbuff, radius, center);

	// spring deformation constraint
	CgSpringDeformationNode* deformationNode =
		new CgSpringDeformationNode(MassSpring->system, vbuff, tauc, deformIter);
	deformationNode->addSprings(massSpringBuilder->getShearIndex());
	deformationNode->addSprings(massSpringBuilder->getStructIndex());

	// fix top corners
	CgPointFixNode* cornerFixer = new CgPointFixNode(MassSpring->system, vbuff);
	cornerFixer->fixPoint(0);
	cornerFixer->fixPoint(n - 1);
	//
	// initialize user interaction
	//

	// build constraint graph
	m_cgRootNode = new CgRootNode(MassSpring->system, vbuff);

	// first layer
	m_cgRootNode->addChild(deformationNode);
	m_cgRootNode->addChild(sphereCollisionNode);

	// second layer
	deformationNode->addChild(cornerFixer);
}

void AMassSpringTestActor::AnimateCloth(int value)
{
	if (value == 0)
	{
		MassSpring->m_solver->current_state = MassSpring->m_solver->prev_state;
		value++;
	}

	// solve two time-steps
	MassSpring->m_solver->solve(m_iter);
	MassSpring->m_solver->solve(m_iter); 

	solved_current_state = MassSpring->m_solver->GetCurrentState();

	// fix points
	CgSatisfyVisitor visitor;
	visitor.satisfy(*m_cgRootNode);

	GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, FString::Printf(TEXT("vertex position(%d, %d, %d)"), 
		Cloth->Particles[0].Position.X, 
		Cloth->Particles[0].Position.Y,
		Cloth->Particles[0].Position.Z));

	// GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Yellow, FString::Printf(TEXT("AnimateCloth")));

	//-- TODO: Light에 대한 Update --//
	// update normals
	//Cloth->get
	//g_clothMesh->update_normals();
	//g_clothMesh->release_face_normals();
}

// Called when the game starts or when spawned
void AMassSpringTestActor::BeginPlay()
{
	Super::BeginPlay();
	InitCloth();
}

// Called every frame
void AMassSpringTestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	Dt = DeltaTime;
	St = FMath::Max(SubstepTime, 0.01f); // Clamp to min substeptime. 

	if (MassSpring->bDoSimulate)
	{

		//At += Dt;
		//while (At > St)
		{
			//InitCloth();
			AnimateCloth(num);
			Cloth->SetParticle(solved_current_state);
			num++;
			//At -= St;
		}
		Cloth->TickUpdateCloth();
	}
	else
	{
		// sleep
	}
}

