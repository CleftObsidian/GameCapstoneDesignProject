// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
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


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
