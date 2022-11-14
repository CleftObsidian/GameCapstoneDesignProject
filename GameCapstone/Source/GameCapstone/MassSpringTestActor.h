// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ClothMeshComponent.h"
#include "MassSpringComponent.h"
#include "MassSpringTestActor.generated.h"

UCLASS()
class GAMECAPSTONE_API AMassSpringTestActor : public AActor
{
	GENERATED_BODY()
private:
	typedef Eigen::VectorXf VectorXf;
	typedef Eigen::Map<Eigen::VectorXf> Map;

public:
	// Sets default values for this actor's properties
	AMassSpringTestActor();
	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "MassSpring")
		UMassSpringComponent* MassSpring;

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "ClothMesh")
		UClothMeshComponent* Cloth;

	// --- MassSpring - Properties - Cloth Simulation ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MassSpring")
		bool bStartSimulate;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Simulation", meta = (ClampMin = "0.005", UIMin = "0.005", UIMax = "0.1"))
		float SubstepTime;

	// Procedual Mesh
	// UClothMeshComponent* m_Mesh;

	// Mass Spring System
	//static mass_spring_system* m_system;
	//static UMassSpringComponent* m_solver;

	// Animation
	static const int m_fps = 60; // frames per second  | 60
	static const int m_iter = 5; // iterations per time step | 10
	static const int m_frame_time = 15; // approximate time for frame calculations | 15
	static const int m_animation_timer = (int)((1.0f / m_fps) * 1000 - m_frame_time);

	float Dt, At, St; // Delta, Accumulated, Substep Time

	unsigned int num = 0; // √ ±‚»≠

	// Constraint Graph
	CgRootNode* m_cgRootNode;

	void InitCloth();
	void AnimateCloth(int value);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
