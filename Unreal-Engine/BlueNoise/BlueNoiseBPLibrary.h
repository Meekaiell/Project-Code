// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Kismet/BlueprintFunctionLibrary.h"
#include "BlueNoiseBPLibrary.generated.h"

/*
*	Function library class.
*	Each function in it is expected to be static and represents blueprint node that can be called in any blueprint.
*
*	When declaring function you can define metadata for the node. Key function specifiers will be BlueprintPure and BlueprintCallable.
*	BlueprintPure - means the function does not affect the owning object in any way and thus creates a node without Exec pins.
*	BlueprintCallable - makes a function which can be executed in Blueprints - Thus it has Exec pins.
*	DisplayName - full name of the node, shown when you mouse over the node and in the blueprint drop down menu.
*				Its lets you name the node using characters not allowed in C++ function names.
*	CompactNodeTitle - the word(s) that appear on the node.
*	Keywords -	the list of keywords that helps you to find node when you search for it using Blueprint drop-down menu.
*				Good example is "Print String" node which you can find also by using keyword "log".
*	Category -	the category your node will be under in the Blueprint drop-down menu.
*
*	For more info on custom blueprint nodes visit documentation:
*	https://wiki.unrealengine.com/Custom_Blueprint_Node_Creation
*/


USTRUCT(BlueprintType)
struct FBlueNoiseStream
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
		FRandomStream Stream;
	UPROPERTY(BlueprintReadWrite)
		FVector2D Point;


	FBlueNoiseStream(const FRandomStream StreamIn, const FVector2D PointIn): Stream(StreamIn), Point(PointIn) {}

	FBlueNoiseStream()
	{
		Point = FVector2D::ZeroVector;
	}
};

USTRUCT(BlueprintType)
struct FBlueNoise
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
		TArray<FVector2D> CandidateArray;
};
UCLASS()
class UBlueNoiseBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_UCLASS_BODY()

	UFUNCTION(BlueprintPure, meta = (DisplayName = "Blue Noise Point In Range", Keywords = "Blue Noise Point In Range"), Category = "BlueNoise")
		static FVector BlueNoise(UPARAM(ref) FBlueNoise &BlueNoise, UPARAM(ref) FBlueNoiseStream &BlueNoiseStream, const FVector2D Min = FVector2D::ZeroVector, const FVector2D Max = FVector2D::ZeroVector);

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Set Blue Noise Stream Seed", Keywords = "Set Blue Noise Stream Seed"), Category = "BlueNoise")
		static void SetBlueNoiseStreamSeed(UPARAM(ref) FBlueNoiseStream &BlueNoiseStream, const int InSeed);

	UFUNCTION()
		static float ToroidalDistance(FVector2D Vec1In, FVector2D Vec2In);

	
};

