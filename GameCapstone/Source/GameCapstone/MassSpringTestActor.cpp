// Fill out your copyright notice in the Description page of Project Settings.
#include "MassSpringTestActor.h"


// Sets default values
AMassSpringTestActor::AMassSpringTestActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	MassSpring = CreateDefaultSubobject<UMassSpringComponent>(TEXT("MassSpring"));
	MassSpring->SetupAttachment(RootComponent);

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
}

