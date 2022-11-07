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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Procedual Mesh
	UClothMeshComponent* m_Mesh;

	// Mass Spring System
	static mass_spring_system* m_system;
	static UMassSpringComponent* m_solver;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
