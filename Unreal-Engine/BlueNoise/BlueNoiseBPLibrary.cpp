// Copyright Epic Games, Inc. All Rights Reserved.

#include "BlueNoiseBPLibrary.h"
#include "BlueNoise.h"
#include "UTree.h"

UBlueNoiseBPLibrary::UBlueNoiseBPLibrary(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {}


/*
 * The algorithm is as follows:

//1. Place a random sample point as the first sample point.
//2. Generate some number of random dots as candidates to be the next sample point.
//3. Whichever of these dots is farthest away from the closest existing sample point is the winner. Place that dot as the new sample point.
//4. GOTO 2 and Repeat until you have as many sample points as you want.
//
//The algorithm is pretty simple, but there are two other important details that are needed to give you good results:
//
//1. When calculating distance between dots, you need to consider wrap around. More info on how to do that here: Calculating the Distance Between Points in “Wrap Around” (Toroidal) Space.
//2. The number of candidates you generate should scale up with the number of existing sample points. As the original paper describing this technique says,
//	doing that helps ensure that the statistics of your sample points stay constant as the number of sample points changes.
 */
FVector UBlueNoiseBPLibrary::BlueNoise(FBlueNoise& BlueNoise, FBlueNoiseStream& BlueNoiseStream, const FVector2D Min, const FVector2D Max)
{
	float BestDistance = -INFINITY;
	float ToroidalDistance = -INFINITY;
	FVector BestCandidate = FVector::ZeroVector;

	// 1. Place a random sample point as the first sample point.
	BlueNoiseStream.Point = { BlueNoiseStream.Stream.FRandRange(Min.X, Max.X),BlueNoiseStream.Stream.FRandRange(Min.Y, Max.Y) };

	//2. Generate some number of random dots as candidates to be the next sample point.
	for (int i = 0; i < BlueNoise.CandidateArray.Num() * 5; ++i)
	{
		FVector2D CandidatePoint = { BlueNoiseStream.Stream.FRandRange(Min.X, Max.X),BlueNoiseStream.Stream.FRandRange(Min.Y, Max.Y) };

		float MinDistance = INFINITY;
		for (auto points : BlueNoise.CandidateArray)
		{
			ToroidalDistance = UBlueNoiseBPLibrary::ToroidalDistance(points, CandidatePoint);
			if (ToroidalDistance < MinDistance)
				MinDistance = ToroidalDistance;
		}
		//3. Whichever of these dots is farthest away from the closest existing sample point is the winner. Place that dot as the new sample point.
		if (MinDistance > BestDistance)
		{
			BestDistance = MinDistance;
			BestCandidate = { CandidatePoint.X, CandidatePoint.Y, 0.0f };

		}
		//4. GOTO 2 and Repeat until you have as many sample points as you want.
	}
	BlueNoise.CandidateArray.Add({ BestCandidate.X, BestCandidate.Y });


	return BestCandidate;
}

void UBlueNoiseBPLibrary::SetBlueNoiseStreamSeed(FBlueNoiseStream& BlueNoiseStream, const int InSeed)
{
	BlueNoiseStream.Stream.Initialize(InSeed);
}

float UBlueNoiseBPLibrary::ToroidalDistance(FVector2D Vec1In, FVector2D Vec2In)
{
	float Dx = FMath::Abs(Vec2In.X - Vec1In.X);
	float Dy = FMath::Abs(Vec2In.Y - Vec1In.Y);

	if (Dx > 0.5f)
		Dx = 1.0f - Dx;
	if (Dy > 0.5f)
		Dy = 1.0f - Dy;

	return FMath::Sqrt(FMath::Square(Dx) + FMath::Square(Dy));
}
